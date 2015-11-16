#include "manycal/ArrayCalibrator.h"
#include "manycal/ManycalCommon.h"

#include "argus_utils/GeometryUtils.h"
#include "argus_utils/MatrixUtils.h"
#include "argus_utils/MapUtils.hpp"

#include <boost/foreach.hpp>

using namespace argus_utils;
using namespace argus_msgs;
using namespace extrinsics_array;
using namespace fiducials;

namespace manycal
{

// TODO Load priors for cameras and fiducials?

ArrayCalibrator::ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), lookup(), 
fiducialManager( lookup ), extrinsicsManager( lookup )
{
	slam = std::make_shared<isam::Slam>();
	
	// Set up fiducial info lookup
	std::string lookupNamespace;
	ph.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
	lookup.SetLookupNamespace( lookupNamespace );
	
	// Grab the list of objects to be calibrated
	// TODO Allow configuration of each target, subtargets?
	std::vector<std::string> targets;
	if( !ph.getParam( "calibration_targets", targets ) )
	{
		ROS_ERROR_STREAM( "Calibration targets must be specified." );
	}
	
	BOOST_FOREACH( const std::string& targetName, targets )
	{
		if( targetRegistry.count( targetName ) > 0 )
		{
			ROS_WARN_STREAM( "Duplicate target specified: " << targetName );
			continue;
		}
		ROS_INFO_STREAM( "Registering target " << targetName );
		TargetRegistration& registration = targetRegistry[ targetName ];
		
		std::string targetNamespace;
		if( !lookup.ReadNamespace( targetName, targetNamespace ) )
		{
			ROS_WARN_STREAM( "Could not retrieve namespace info for: " << targetName );
			continue;
		}
		
		std::string odometryTopic;
		std::string odometryKey = targetNamespace + "odometry_topic";
		if( nodeHandle.getParam( odometryKey, odometryTopic ) )
		{
			ROS_WARN_STREAM( "Could not read odometry_topic for: " << targetName 
			    << " at path: " << odometryKey );
			continue;
		}
		// TODO Specify buffer size?
		registration.odometrySub = nodeHandle.subscribe( odometryTopic, 
		                                                 10,
		                                                 &ArrayCalibrator::OdometryCallback,
		                                                 this );
		
		std::string detectionTopic;
		std::string detectionKey = targetNamespace + "detection_topic";
		if( nodeHandle.getParam( detectionKey, detectionTopic ) )
		{
			registration.detectionSub = nodeHandle.subscribe( detectionTopic,
			                                                  10,
			                                                  &ArrayCalibrator::DetectionCallback,
			                                                  this );
		}
		
	}
}

ArrayCalibrator::~ArrayCalibrator()
{
// 	slam->batch_optimization();
// 	slam->write( std::cout );
}

// TODO Change this back to relative_pose so we can catch skipped time steps
void ArrayCalibrator::OdometryCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
	std::string frameName = msg->header.frame_id;
	
	// If we haven't seen the frame yet, initialize it
	if( frameRegistry.count( frameName ) == 0 )
	{
		// We can't add odometry after initialization because it's zero time difference
		// NOTE We could just let poses->AddOdometry return nullptr, but this is clearer
		InitializeFrame( frameName, msg->header.stamp );
		return; 
	}
	
	// Now parse message fields
	PoseSE3 displacement = MsgToPose( msg->pose.pose );
	PoseSE3::CovarianceMatrix cov;
	ParseMatrix<PoseSE3::CovarianceMatrix>( msg->pose.covariance, cov );
	
	// Add odometry to graph
	FrameRegistration& registration = frameRegistry[ frameName ];
	registration.poses->AddOdometry( msg->header.stamp, isam::PoseSE3( displacement ),
	                                 isam::Covariance( cov ) );
}

void ArrayCalibrator::DetectionCallback( const ImageFiducialDetections::ConstPtr& msg )
{
	std::string cameraName = msg->header.frame_id;
	
	// If we haven't seen this camera before, initialize it
	if( cameraRegistry.count( cameraName ) == 0 )
	{
		if( !InitializeCamera( cameraName, msg->header.stamp ) )
		{
			return;
		}
	}
	CameraRegistration& cameraRegistration = cameraRegistry[ cameraName ];
	
	// NOTE We don't have to check if the frame exists because initializing the camera
	// automatically registers the frame if it doesn't already exist
	std::string cameraFrameName = extrinsicsManager.GetReferenceFrame( cameraName );
	FrameRegistration& cameraFrameRegistration = frameRegistry[ cameraFrameName ];
	ros::Time latestCamTime = get_highest_key( cameraFrameRegistration.poses->timeSeries );
	
	// If the message postdates the last camera odometry, we have to discard it
	if( msg->header.stamp > latestCamTime )
	{
		ROS_WARN_STREAM( "Dropping detection from " << cameraName << " at time "
		    << msg->header.stamp << " after latest camera odometry at " << latestCamTime );
		return;
	}
		
	BOOST_FOREACH( const FiducialDetection& detection, msg->detections )
	{
		std::string fiducialName = detection.name;
		if( fiducialRegistry.count( fiducialName ) == 0 )
		{
			if( !InitializeFiducial( fiducialName, msg->header.stamp ) )
			{
				continue;
			}
		}
		FiducialRegistration& fiducialRegistration = fiducialRegistry[ fiducialName ];
		
		std::string fiducialFrameName = extrinsicsManager.GetReferenceFrame( fiducialName );
		FrameRegistration& fiducialFrameRegistration = frameRegistry[ fiducialFrameName ];
		ros::Time latestFiducialTime = get_highest_key( fiducialFrameRegistration.poses->timeSeries );
		
		ROS_INFO_STREAM( "Message stamp: " << msg->header.stamp << " last cam: " 
		    << latestCamTime << " last fid: " << latestFiducialTime );
		
		// If the message postdates the latest fiducial odometry, we have to discard it
		if( msg->header.stamp > latestFiducialTime )
		{
			ROS_WARN_STREAM( "Dropping detection from " << cameraName << " at time "
			    << msg->header.stamp << " after latest fiducial odometry at " << latestFiducialTime );
			return;
		}
		
		CreateObservationFactor( cameraRegistration, cameraFrameRegistration,
		                         fiducialRegistration, fiducialFrameRegistration,
		                         detection, msg->header.stamp );
		
	}
	
	
}

void ArrayCalibrator::CreateObservationFactor( CameraRegistration& camera,
                                               FrameRegistration& cameraFrame,
                                               FiducialRegistration& fiducial,
                                               FrameRegistration& fiducialFrame,
                                               const FiducialDetection& detection, 
                                               ros::Time t )
{
	Eigen::VectorXd detectionVector = Eigen::VectorXd( 2*detection.points.size() );
	for( unsigned int i = 0; i < detection.points.size(); i++ )
	{
		detectionVector.block<2,1>(2*i,0) = Eigen::Vector2d( detection.points[i].x, detection.points[i].y );
	}
	isam::FiducialDetection det( detectionVector );
	isam::Noise cov = isam::Covariance( 10 * isam::eye( 2*detection.points.size() ) );
	
	// TODO Pull these flags from the parameter server?
	isam::FiducialFactor::Properties props; 
	props.optCamReference = true;
	props.optCamIntrinsics = camera.optimizeIntrinsics;
	props.optCamExtrinsics = camera.optimizeExtrinsics;
	props.optFidReference = true;
	props.optFidIntrinsics = fiducial.optimizeIntrinsics;
	props.optFidExtrinsics = fiducial.optimizeExtrinsics;
	
	isam::PoseSE3_Node::Ptr cameraFramePose = cameraFrame.poses->GetNodeInterpolate( t );
	isam::PoseSE3_Node::Ptr fiducialFramePose = fiducialFrame.poses->GetNodeInterpolate( t );
	
	if( !cameraFramePose || !fiducialFramePose )
	{
		ROS_WARN_STREAM( "Split odometry failed." );
		return;
	}
	
	// camRef, camInt, camExt, fidRef, fidInt, fidExt
 	isam::FiducialFactor::Ptr factor = std::make_shared<isam::FiducialFactor>
 	    ( cameraFramePose.get(), camera.intrinsics.get(), camera.extrinsics.get(),
	      fiducialFramePose.get(), fiducial.intrinsics.get(), fiducial.extrinsics.get(),
	      det, cov, props );
	slam->add_factor( factor.get() );
	observations.push_back( factor );
	
}

bool ArrayCalibrator::InitializeFrame( const std::string& frameName, ros::Time t )
{
	if( frameRegistry.count( frameName ) > 0 ) { return true; }
	
	ROS_INFO_STREAM( "Initializing new frame: " << frameName << " at time " << t );
	
	FrameRegistration registration;
	registration.optimizePoses = true; // TODO
	registration.poses = std::make_shared<OdometryGraphSE3>( slam );
	registration.poses->Initialize( t, 
	                                isam::PoseSE3( 0, 0, 0, 1, 0, 0, 0 ),
	                                isam::Covariance( 1E3 * isam::eye(6) ) ); // TODO
	frameRegistry[ frameName ] = registration;
	return true;
}

bool ArrayCalibrator::InitializeCamera( const std::string& cameraName, ros::Time t )
{
	// Cache the camera lookup information
	if( !extrinsicsManager.HasMember( cameraName ) )
	{
		if( !extrinsicsManager.ReadMemberInformation( cameraName ) )
		{
			ROS_WARN_STREAM( "Could not retrieve extrinsics info for camera " << cameraName );
			return false;
		}
	}
	
	// Retrieve the camera extrinsics and reference frame
	const std::string& cameraFrameName = extrinsicsManager.GetReferenceFrame( cameraName );
	const PoseSE3& cameraExtrinsics = extrinsicsManager.GetExtrinsics( cameraName );
	
	if( frameRegistry.count( cameraFrameName ) == 0 )
	{
		InitializeFrame( cameraFrameName, t );
	}
	
	ROS_INFO_STREAM( "Registering camera " << cameraName );
	CameraRegistration registration;
	registration.extrinsics = std::make_shared <isam::PoseSE3_Node>();
	registration.extrinsics->init( cameraExtrinsics );
	
	registration.intrinsics = std::make_shared <isam::MonocularIntrinsics_Node>();
	isam::MonocularIntrinsics intrinsics( 1.0, 1.0, Eigen::Vector2d(0,0) );
	registration.intrinsics->init( intrinsics );
	
	registration.optimizeExtrinsics = true; // TODO
	registration.optimizeIntrinsics = false; // TODO
	
	cameraRegistry[ cameraName ] = registration;
	
	return true;
}

bool ArrayCalibrator::InitializeFiducial( const std::string& fidName, ros::Time t )
{
	// Cache the fiducial lookup information
	if( !fiducialManager.HasFiducial( fidName ) )
	{
		if( !fiducialManager.ReadFiducialInformation( fidName, false ) )
		{
			ROS_WARN_STREAM( "Could not retrieve fiducial info for " << fidName );
			return false;
		}
	}
	
	// Retrieve the fiducial extrinsics, intrinsics, and reference frame
	const Fiducial& fiducial = fiducialManager.GetIntrinsics( fidName );
	const std::string& fiducialFrameName = extrinsicsManager.GetReferenceFrame( fidName );
	const PoseSE3& fiducialExtrinsics = extrinsicsManager.GetExtrinsics( fidName );
	
	if( frameRegistry.count( fiducialFrameName ) == 0 )
	{
		InitializeFrame( fiducialFrameName, t );
	}
	
	ROS_INFO_STREAM( "Registering fiducial " << fidName );
	FiducialRegistration registration;
	registration.extrinsics = std::make_shared <isam::PoseSE3_Node>();
	registration.extrinsics->init( fiducialExtrinsics );
	
	std::vector <isam::Point3d> pts;
	pts.reserve( fiducial.points.size() );
	for( unsigned int i = 0; i < fiducial.points.size(); i++ )
	{
		pts.push_back( MsgToIsam( fiducial.points[i] ) );
	}
	isam::FiducialIntrinsics intrinsics( pts );
	registration.intrinsics = std::make_shared <isam::FiducialIntrinsics_Node>( intrinsics.name(), intrinsics.dim() );
	registration.intrinsics->init( intrinsics );
	
	registration.optimizeExtrinsics = true;
	registration.optimizeIntrinsics = false;
	
	fiducialRegistry[ fidName ] = registration;
	
	return true;
}

}
