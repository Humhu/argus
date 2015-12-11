#include "manycal/ArrayCalibrator.h"
#include "manycal/ManycalCommon.h"

#include "argus_utils/GeometryUtils.h"
#include "argus_utils/MatrixUtils.h"
#include "argus_utils/MapUtils.hpp"

#include "fiducials/FiducialCommon.h"
#include "fiducials/PoseEstimation.h"

#include <boost/foreach.hpp>

using namespace argus_utils;
using namespace argus_msgs;
using namespace extrinsics_array;
using namespace fiducials;

namespace manycal
{

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
	
	ph.param( "target_num_observations", targetNumObservations, 100 );
	
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
		
		std::string odometryTopic = targetNamespace + "odometry";
		// TODO Specify buffer size?
		registration.odometrySub = nodeHandle.subscribe( odometryTopic, 
		                                                 10,
		                                                 &ArrayCalibrator::OdometryCallback,
		                                                 this );
		
		std::string detectionTopic = targetNamespace + "detections_processed";
		registration.detectionSub = nodeHandle.subscribe( detectionTopic,
		                                                  10,
		                                                  &ArrayCalibrator::DetectionCallback,
		                                                  this );
	}
}

ArrayCalibrator::~ArrayCalibrator()
{
	WriteResults(); 
}

void ArrayCalibrator::WriteResults()
{
	isam::Covariances covs = slam->covariances();
	std::list<isam::Node*> nodes;
	
	Eigen::MatrixXd allCovs = covs.marginal( slam->get_nodes() );
	ROS_INFO_STREAM( "All covs: " << std::endl << allCovs );
	
	BOOST_FOREACH( const CameraRegistry::value_type& item, cameraRegistry )
	{
		const std::string& camName = item.first;
		const CameraRegistration& camReg = item.second;
		if( camReg.optimizeExtrinsics )
		{
			ROS_INFO_STREAM( "Writing extrinsics for camera: " << camName );
			extrinsicsManager.GetInfo( camName ).extrinsics = camReg.extrinsics->value().pose;
			extrinsicsManager.WriteMemberInfo( camName, true );
			
			nodes.clear();
			nodes.push_back( camReg.extrinsics.get() );
			Eigen::MatrixXd S = covs.marginal( nodes );
			ROS_INFO_STREAM( "Covariance: " << std::endl << S );
		}
		
		if( camReg.optimizeIntrinsics )
		{
			ROS_INFO_STREAM( "Writing intrinsics for camera: " << camName );
			// TODO
		}
	}
	
	BOOST_FOREACH( const FiducialRegistry::value_type& item, fiducialRegistry )
	{
		const std::string& fidName = item.first;
		if( item.second.optimizeExtrinsics )
		{
			ROS_INFO_STREAM( "Writing extrinsics for fiducial: " << fidName );
			extrinsicsManager.GetInfo( fidName ).extrinsics = item.second.extrinsics->value().pose;
			extrinsicsManager.WriteMemberInfo( fidName, true );
		}
		if( item.second.optimizeIntrinsics )
		{
			ROS_INFO_STREAM( "Writing intrinsics for fiducial: " << fidName );
			// TODO
		}
	}
}

// TODO Change this back to relative_pose so we can catch skipped time steps?
void ArrayCalibrator::OdometryCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
	std::string frameName = msg->header.frame_id;
	
	// Don't initialize frames on odometry callbacks, save it for detections	
	if( frameRegistry.count( frameName ) == 0 ) { return; }
	
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

	ros::Time timestamp = msg->header.stamp;
	std::string camName = msg->header.frame_id;
	if( cameraRegistry.count( camName ) == 0 ) { InitializeCamera( camName ); }
	CameraRegistration& camReg = cameraRegistry[ camName ];
	
	const extrinsics_array::ExtrinsicsInfo& camInfo = extrinsicsManager.GetInfo( camName );
	if( frameRegistry.count( camInfo.referenceFrame ) == 0 ) 
	{ 
		CreateFrame( camInfo.referenceFrame, timestamp ); 
	}
	FrameRegistration& camFrameReg = frameRegistry[ camInfo.referenceFrame ];
	
	BOOST_FOREACH( const FiducialDetection& detection, msg->detections )
	{
		std::string fidName = detection.name;
		if( fiducialRegistry.count( fidName ) == 0 ) { InitializeFiducial( fidName ); }
		FiducialRegistration& fidReg = fiducialRegistry[ fidName ];
		
		const extrinsics_array::ExtrinsicsInfo& fidInfo = extrinsicsManager.GetInfo( fidName );
		if( frameRegistry.count( fidInfo.referenceFrame ) == 0 ) 
		{ 
			CreateFrame( fidInfo.referenceFrame, timestamp ); 
		}
		FrameRegistration& fidFrameReg = frameRegistry[ fidInfo.referenceFrame ];
		
		// If one of the two frames is uninitialized, we initialize it using PNP
		if( !camFrameReg.initialized || !fidFrameReg.initialized )
		{
			PoseSE3 relPose = EstimateArrayPose( MsgToPoints( detection.points ),
			                                     nullptr,
			                                     MatrixToPoints( fidReg.intrinsics->value().matrix() ) );
			
			ROS_INFO_STREAM( "Fiducial detected at: " << relPose );
			
			if( !camFrameReg.initialized )
			{
				isam::PoseSE3_Node::Ptr fidFrameNode = fidFrameReg.poses->GetNodeInterpolate( timestamp );
				if( !fidFrameNode ) 
				{ 
					ROS_WARN_STREAM( "Could not interpolate fiducial frame pose." );
					continue; 
				}
				PoseSE3 fidFramePose = fidFrameNode->value().pose;
				PoseSE3 camFramePose = fidFramePose * fidInfo.extrinsics 
					* relPose.Inverse() * camInfo.extrinsics.Inverse();
				camFrameReg.poses->Initialize( timestamp, camFramePose );
				camFrameReg.poses->AddPrior( timestamp, 
				                             camFramePose, 
				                             isam::Covariance( 1E6 * isam::eye(6) ) );
				camFrameReg.initialized = true;
			}
			else
			{
				isam::PoseSE3_Node::Ptr camFrameNode = camFrameReg.poses->GetNodeInterpolate( timestamp );
				if( !camFrameNode ) { 
					ROS_WARN_STREAM( "Could not interpolate camera frame pose." );
					continue; 
				}
				PoseSE3 camFramePose = camFrameNode->value().pose;
				PoseSE3 fidFramePose = camFramePose * camInfo.extrinsics 
					* relPose * fidInfo.extrinsics.Inverse();
				ROS_INFO_STREAM( "cam pose: " << camFramePose );
				ROS_INFO_STREAM( "fid pose: " << fidFramePose );
				fidFrameReg.poses->Initialize( timestamp, fidFramePose );
				fidFrameReg.poses->AddPrior( timestamp, fidFramePose, isam::Covariance( 1E6 * isam::eye(6) ) );
				fidFrameReg.initialized = true;
			}
		}
		
		// If the message postdates the last camera odometry, we have to discard it
		ros::Time latestCamTime = get_highest_key( camFrameReg.poses->timeSeries );
		if( msg->header.stamp > latestCamTime ) { return; }
			
		// If the message postdates the latest fiducial odometry, we have to discard it
		ros::Time latestFiducialTime = get_highest_key( fidFrameReg.poses->timeSeries );
		if( msg->header.stamp > latestFiducialTime ) { return; }
		
		CreateObservationFactor( camReg, camFrameReg,
		                         fidReg, fidFrameReg,
		                         detection, msg->header.stamp );
	}
	
// 	ROS_INFO_STREAM( "Running optimization..." );
	slam->batch_optimization();
// 	std::cout << "===== Iteration: " << observations.size() << std::endl;
// 	slam->write( std::cout );
	
	char buff[50];
	sprintf( buff, "%3.2f", observations.size() * 100.0 / targetNumObservations );
	std::string percentage( buff );
	
	ROS_INFO_STREAM( "Observations: [" << observations.size() << "/" << targetNumObservations 
	    << "] (" << percentage << "%");
	if( observations.size() >= targetNumObservations ) {
		WriteResults();
		exit( 0 ); 
	}
}

void ArrayCalibrator::CreateObservationFactor( CameraRegistration& camera,
                                               FrameRegistration& cameraFrame,
                                               FiducialRegistration& fiducial,
                                               FrameRegistration& fiducialFrame,
                                               const FiducialDetection& detection, 
                                               ros::Time t )
{
	ROS_INFO_STREAM( "Creating factor for camera " << camera.name << " and fiducial " << fiducial.name );
	isam::FiducialDetection det = DetectionToIsam( detection );
	
	// Assuming 3% image coordinate error
	double imageCoordinateErr = std::pow( 0.03, 2 );
	isam::Noise cov = isam::Covariance( imageCoordinateErr * isam::eye( 2*detection.points.size() ) );
	
	isam::FiducialFactor::Properties props; 
	props.optCamReference = cameraFrame.optimizePoses;
	props.optCamIntrinsics = camera.optimizeIntrinsics;
	props.optCamExtrinsics = camera.optimizeExtrinsics;
	props.optFidReference = fiducialFrame.optimizePoses;
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

void ArrayCalibrator::CreateFrame( const std::string& frameName, const ros::Time& t )
{
	if( frameRegistry.count( frameName ) > 0 ) { return; }
	
	ROS_INFO_STREAM( "Creating new reference frame: " << frameName );
	FrameRegistration registration;
	registration.initialized = false;
	registration.optimizePoses = true; // TODO
	registration.poses = std::make_shared<OdometryGraphSE3>( slam );
	
	if( frameRegistry.size() == 0 )
	{
		// TODO
		ROS_INFO_STREAM( "Anchoring first reference frame: " << frameName );
		isam::PoseSE3 init( 0, 0, 0, 1, 0, 0, 0 );
		registration.poses->Initialize( t, init );
		registration.poses->AddPrior( t, init, isam::Covariance( 1E-6 * isam::eye(6) ) );
		registration.initialized = true;
	}
	
	frameRegistry[ frameName ] = registration;
}

void ArrayCalibrator::InitializeCamera( const std::string& camName )
{
	// Cache the camera lookup information
	if( !extrinsicsManager.HasMember( camName ) )
	{
		if( !extrinsicsManager.ReadMemberInfo( camName ) )
		{
			ROS_ERROR_STREAM( "Could not retrieve info for camera: " << camName );
			exit( -1 );
		}
	}
	
	// Retrieve the camera extrinsics
	const PoseSE3& cameraExtrinsicsInit = extrinsicsManager.GetInfo( camName ).extrinsics;
	
	ROS_INFO_STREAM( "Registering camera " << camName );
	CameraRegistration registration;
	registration.name = camName;
	registration.optimizeExtrinsics = true; // TODO
	registration.optimizeIntrinsics = false; // TODO
	
	registration.extrinsics = std::make_shared <isam::PoseSE3_Node>();
	registration.extrinsics->init( cameraExtrinsicsInit );
	
	// TODO Only create priors for cameras with initial guesses
	if( registration.optimizeExtrinsics )
	{
		slam->add_node( registration.extrinsics.get() );
		registration.extrinsicsPrior = std::make_shared <isam::PoseSE3_Prior>(
		    registration.extrinsics.get(), 
			cameraExtrinsicsInit,
			isam::Covariance( 1E6 * isam::eye(6) ) );
		slam->add_factor( registration.extrinsicsPrior.get() );
	}
	
	registration.intrinsics = std::make_shared <isam::MonocularIntrinsics_Node>();
	isam::MonocularIntrinsics intrinsics( 1.0, 1.0, Eigen::Vector2d(0,0) );
	registration.intrinsics->init( intrinsics );
	if( registration.optimizeIntrinsics )
	{
		slam->add_node( registration.intrinsics.get() );	
		// TODO priors
	}
	
	cameraRegistry[ camName ] = registration;
}

void ArrayCalibrator::InitializeFiducial( const std::string& fidName )
{
	// Cache the fiducial lookup information
	if( !fiducialManager.HasMember( fidName ) )
	{
		if( !fiducialManager.ReadMemberInfo( fidName, false ) ||
			!extrinsicsManager.ReadMemberInfo( fidName, false ) )
		{
			ROS_ERROR_STREAM( "Could not retrieve fiducial info for " << fidName );
			exit( -1 );
		}
	}
	
	// Retrieve the fiducial extrinsics, intrinsics, and reference frame
	const Fiducial& fiducial = fiducialManager.GetInfo( fidName );
	const PoseSE3& fiducialExtrinsics = extrinsicsManager.GetInfo( fidName ).extrinsics;

	ROS_INFO_STREAM( "Registering fiducial " << fidName );
	FiducialRegistration registration;
	registration.name = fidName;
	registration.optimizeExtrinsics = false; // TODO
	registration.optimizeIntrinsics = false;
	
	registration.extrinsics = std::make_shared <isam::PoseSE3_Node>();
	registration.extrinsics->init( fiducialExtrinsics );
	if( registration.optimizeExtrinsics )
	{
		slam->add_node( registration.extrinsics.get() );
		registration.extrinsicsPrior = std::make_shared <isam::PoseSE3_Prior>(
		    registration.extrinsics.get(), 
			isam::PoseSE3( 0, 0, 0, 1, 0, 0, 0 ),
			isam::Covariance( 1E6 * isam::eye(6) ) );
		slam->add_factor( registration.extrinsicsPrior.get() );
	}
	
	std::vector <isam::Point3d> pts;
	pts.reserve( fiducial.points.size() );
	for( unsigned int i = 0; i < fiducial.points.size(); i++ )
	{
		pts.push_back( MsgToIsam( fiducial.points[i] ) );
	}
	isam::FiducialIntrinsics intrinsics( pts );
	registration.intrinsics = std::make_shared <isam::FiducialIntrinsics_Node>( intrinsics.name(), intrinsics.dim() );
	registration.intrinsics->init( intrinsics );
	if( registration.optimizeIntrinsics )
	{
		slam->add_node( registration.intrinsics.get() );
		// TODO priors
	}
	
	fiducialRegistry[ fidName ] = registration;
}

}
