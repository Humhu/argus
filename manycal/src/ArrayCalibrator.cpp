#include "manycal/ArrayCalibrator.h"
#include "manycal/ManycalCommon.h"

#include "argus_utils/GeometryUtils.h"
#include "argus_utils/MatrixUtils.h"
#include "argus_utils/MapUtils.hpp"

#include <boost/foreach.hpp>

using namespace argus_utils;
using namespace argus_msgs;
using namespace extrinsics_array;
using namespace fiducial_array;

namespace manycal
{

// TODO Load priors for cameras and fiducials?

ArrayCalibrator::ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), fidManager( nodeHandle ), camManager( nodeHandle )
{
	slam = std::make_shared<isam::Slam>();
	
	// Set up fiducial info lookup
	std::string lookupNamespace;
	if( !ph.getParam( "lookup_namespace", lookupNamespace ) ) 
	{
		ROS_ERROR_STREAM( "Lookup namespace must be specified." );
	}
	
	if( lookupNamespace.back() != '/' ) { lookupNamespace += "/"; }
	fidManager.SetLookupNamespace( lookupNamespace + "fiducials" );
	camManager.SetLookupNamespace( lookupNamespace + "cameras" );
	
	// TODO Think about how these subscriptions should actually work for the calibrator 
	// to be able to run without having to reconfigure the entire system
	detectionSub = nodeHandle.subscribe( "detections", 10, 
	                                     &ArrayCalibrator::DetectionCallback, 
	                                     this );
	relPoseSub = nodeHandle.subscribe( "relative_poses", 10, 
	                                   &ArrayCalibrator::RelPoseCallback, 
	                                   this );
}

ArrayCalibrator::~ArrayCalibrator() {}

void ArrayCalibrator::RelPoseCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
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
		InitializeCamera( cameraName, msg->header.stamp );
	}
	CameraRegistration& cameraRegistration = cameraRegistry[ cameraName ];
	
	// NOTE We don't have to check if the frame exists because initializing the camera
	// automatically registers the frame if it doesn't already exist
	std::string cameraFrameName = camManager.GetParentArray( cameraName ).GetReferenceFrame();
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
			InitializeFiducial( fiducialName, msg->header.stamp );
		}
		FiducialRegistration& fiducialRegistration = fiducialRegistry[ fiducialName ];
		
		std::string fiducialFrameName = fidManager.GetParentArray( detection.name ).GetReferenceFrame();
		FrameRegistration& fiducialFrameRegistration = frameRegistry[ fiducialFrameName ];
		ros::Time latestFiducialTime = get_highest_key( fiducialFrameRegistration.poses->timeSeries );
		
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
	
	// TODO Pull these flags from the parameter server?
	isam::FiducialFactor::Properties props; 
	props.optCamReference = true;
	props.optCamIntrinsics = camera.optimizeIntrinsics;
	props.optCamExtrinsics = camera.optimizeExtrinsics;
	props.optFidReference = true;
	props.optFidIntrinsics = fiducial.optimizeIntrinsics;
	props.optFidExtrinsics = fiducial.optimizeExtrinsics;
	
	// TODO Check tail?
// 	if( cameraFrame.poses->
	
	// camRef, camInt, camExt, fidRef, fidInt, fidExt
// 	isam::FiducialFactor::Ptr factor = std::make_shared<isam::FiducialFactor>
// 	    ( camera.optimizeExtrinsics, 
}

void ArrayCalibrator::InitializeFrame( const std::string& frameName, ros::Time t )
{
	if( frameRegistry.count( frameName ) > 0 ) { return; }
	
	FrameRegistration registration;
	registration.optimizePoses = true; // TODO
	registration.poses = std::make_shared<OdometryGraphSE3>( slam );
 	registration.poses->Initialize( t, isam::PoseSE3( 0, 0, 0, 1, 0, 0, 0 ),
	                                isam::Covariance( 1E3 * isam::eye(6) ) ); // TODO
	frameRegistry[ frameName ] = registration;
}

void ArrayCalibrator::InitializeCamera( const std::string& cameraName, ros::Time t )
{
	// Cache the camera lookup information
	if( !camManager.HasMember( cameraName ) )
	{
		if( !camManager.ReadMemberInformation( cameraName ) )
		{
			ROS_WARN_STREAM( "Could not retrieve extrinsics info for camera " << cameraName );
			return;
		}
	}
	
	// Retrieve the camera extrinsics and reference frame
	const ExtrinsicsArray& cameraArray = camManager.GetParentArray( cameraName );
	const std::string& cameraFrameName = cameraArray.GetReferenceFrame();
	const PoseSE3& cameraExtrinsics = cameraArray.GetPose( cameraName );
	
	if( frameRegistry.count( cameraFrameName ) == 0 )
	{
		InitializeFrame( cameraFrameName, t );
	}
	
	CameraRegistration registration;
	registration.extrinsics = std::make_shared <isam::PoseSE3_Node>();
	registration.extrinsics->init( cameraExtrinsics );
	
	registration.intrinsics = std::make_shared <isam::MonocularIntrinsics_Node>();
	isam::MonocularIntrinsics intrinsics( 1.0, 1.0, Eigen::Vector2d(0,0) );
	registration.intrinsics->init( intrinsics );
	
	registration.optimizeExtrinsics = true; // TODO
	registration.optimizeIntrinsics = false; // TODO
	
	cameraRegistry[ cameraName ] = registration;
}

void ArrayCalibrator::InitializeFiducial( const std::string& fidName, ros::Time t )
{
	// Cache the fiducial lookup information
	if( !fidManager.HasMember( fidName ) )
	{
		if( !fidManager.ReadMemberInformation( fidName, false ) )
		{
			ROS_WARN_STREAM( "Could not retrieve fiducial info for " << fidName );
			return;
		}
	}
	
	// Retrieve the fiducial extrinsics, intrinsics, and reference frame
	const FiducialArray& fiducialArray = fidManager.GetParentFiducialArray( fidName );
	const Fiducial& fiducial = fiducialArray.GetFiducial( fidName );
	const std::string& fiducialFrameName = fiducialArray.GetReferenceFrame();
	const PoseSE3& fiducialExtrinsics = fiducialArray.GetPose( fidName );
	
	if( frameRegistry.count( fiducialFrameName ) == 0 )
	{
		InitializeFrame( fiducialFrameName, t );
	}
	
	FiducialRegistration registration;
	registration.extrinsics = std::make_shared <isam::PoseSE3_Node>();
	registration.extrinsics->init( fiducialExtrinsics );
	
// 	std::vector <isam::Point3d> pts;
// 	pts.reserve( fiducial.points.size() );
// 	for( unsigned int i = 0; i < fiducial.points.size(); i++ )
// 	{
// 		pts.push_back( MsgToIsam( fiducial.points[i] ) );
// 	}
// 	isam::FiducialIntrinsics intrinsics( pts );
// 	registration.intrinsics = std::make_shared <isam::FiducialIntrinsics_Node>( intrinsics.name(), intrinsics.dim() );
// 	registration.intrinsics->init( intrinsics );
	
	registration.optimizeExtrinsics = true;
	registration.optimizeIntrinsics = false;
	
	fiducialRegistry[ fidName ] = registration;
}

}
