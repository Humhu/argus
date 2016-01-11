#include "extrinsics_array/ExtrinsicsCalibrationParsers.h"
#include "manycal/CameraArrayCalibrator.h"
#include "fiducials/PoseEstimation.h"
#include <boost/foreach.hpp>

using namespace argus_msgs;
using namespace argus_utils;
using namespace fiducials;

namespace manycal
{

CameraArrayCalibrator::CameraArrayCalibrator( const ros::NodeHandle& nh,
                                              const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), fiducialManager( lookupInterface ),
extrinsicsManager( lookupInterface )
{
	if( !privHandle.getParam( "reference_frame", referenceFrame ) )
	{
		ROS_ERROR_STREAM( "Must specify array reference frame." );
		exit( -1 );
	}
	
	slam = std::make_shared<isam::Slam>();
	
	std::string lookupNamespace;
	privHandle.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
	lookupInterface.SetLookupNamespace( lookupNamespace );
	
	writeServer = privHandle.advertiseService( "write_results", 
	                                           &CameraArrayCalibrator::WriteResults,
	                                           this );

	detSub = nodeHandle.subscribe( "detections", 10, &CameraArrayCalibrator::DetectionCallback, this );
}
	
bool CameraArrayCalibrator::WriteResults( WriteCalibration::Request& req,
                                          WriteCalibration::Response& res )
{
	YAML::Node yaml;

	BOOST_FOREACH( const CameraRegistry::value_type& item, cameraRegistry )
	{
		const std::string& name = item.first;
		const CameraRegistration& registration = item.second;
		PoseSE3 extrinsics = registration.extrinsics->value().pose;
		ROS_INFO_STREAM( "Camera " << name << " pose " << extrinsics );
		
		extrinsicsManager.GetInfo( name ).extrinsics = extrinsics;
		extrinsicsManager.GetInfo( name ).referenceFrame = referenceFrame;
		extrinsicsManager.WriteMemberInfo( name );

		YAML::Node node;
		PopulateExtrinsicsCalibration( extrinsicsManager.GetInfo( name ), node );
		yaml[name] = node;
	}

	std::ofstream resultsFile( req.calibrationPath );
	if( !resultsFile.is_open() )
	{
		ROS_ERROR_STREAM( "Could not open results file at: " << req.calibrationPath );
		return false;
	}
	ROS_INFO_STREAM( "Writing results to " << req.calibrationPath );
	resultsFile << yaml;
	return true;
}

void CameraArrayCalibrator::ProcessCache()
{
	std::vector <ImageFiducialDetections::ConstPtr> recache;
	BOOST_FOREACH( const ImageFiducialDetections::ConstPtr& msg, cachedObservations )
	{
		if( !ProcessDetection( msg ) )
		{
			recache.push_back( msg );
		}
	}
	cachedObservations = recache;
	slam->batch_optimization();
}

void CameraArrayCalibrator::DetectionCallback( const ImageFiducialDetections::ConstPtr& msg )
{
	cachedObservations.push_back( msg );
	ProcessCache();
}

bool CameraArrayCalibrator::ProcessDetection( const ImageFiducialDetections::ConstPtr& msg )
{
	// Register all new fiducials
	BOOST_FOREACH( const FiducialDetection& detection, msg->detections )
	{
		const std::string& fiducialName = detection.name;
		if( fiducialRegistry.count( fiducialName ) == 0 )
		{
			if( !fiducialManager.CheckMemberInfo( fiducialName ) ) 
			{ 
				ROS_WARN_STREAM( "Could not find info for detected fiducial " << fiducialName );
				continue; 
			}
			RegisterFiducial( fiducialName );
		}
	}
	
	// Attempt to initialize the camera if new
	const std::string& cameraName = msg->header.frame_id;
	if( cameraRegistry.count( cameraName ) == 0 )
	{
		if( !InitializeCamera( msg ) ) { return false; }
	}
	const CameraRegistration& camReg = cameraRegistry[ cameraName ];
	
	// Process detections
	const ros::Time now = msg->header.stamp;
	BOOST_FOREACH( const FiducialDetection& detection, msg->detections )
	{
		FiducialRegistration& fidReg = fiducialRegistry[ detection.name ];
		if( fidReg.poses.count( now ) == 0 )
		{
			PoseSE3 cameraPose = camReg.extrinsics->value().pose;
			PoseSE3 relPose = EstimateArrayPose( MsgToPoints( detection.points ),
		                                         nullptr,
		                                         MatrixToPoints( fidReg.intrinsics->value().matrix() ) );
			PoseSE3 fiducialPose = cameraPose * relPose;
			ROS_INFO_STREAM( "Initializing fiducial at pose: " << fiducialPose );
			isam::PoseSE3_Node::Ptr fidNode = std::make_shared<isam::PoseSE3_Node>();
			fidNode->init( isam::PoseSE3( fiducialPose ) );
			fidReg.poses[ now ] = fidNode;
			slam->add_node( fidNode.get() );
		}
		
		isam::PoseSE3_Node::Ptr fidNode = fidReg.poses[ now ];
		
		double imageCoordinateErr = std::pow( 0.03, 2 );
		isam::Noise cov = isam::Covariance( imageCoordinateErr * isam::eye( 2*detection.points.size() ) );
		
		isam::FiducialFactor::Properties props;
		props.optCamReference = true; // Reference is actually extrinsics here
		props.optCamIntrinsics = false;
		props.optCamExtrinsics = false;
		props.optFidReference = true;
		props.optFidIntrinsics = false;
		props.optFidExtrinsics = false;
		
		isam::FiducialFactor::Ptr factor = std::make_shared<isam::FiducialFactor>
			( camReg.extrinsics.get(), camReg.intrinsics.get(), nullptr,
			  fidNode.get(), fidReg.intrinsics.get(), nullptr,
			  DetectionToIsam( detection ), cov, props );
		ROS_INFO_STREAM( "Adding detection" );
		slam->add_factor( factor.get() );
		observations.push_back( factor );
	}
	
	return true;
}

// Attempt to initialize a camera from the detection
bool CameraArrayCalibrator::InitializeCamera( const ImageFiducialDetections::ConstPtr& msg )
{
	const std::string& cameraName = msg->header.frame_id;
	if( cameraRegistry.count( cameraName ) > 0 ) { return true; }
	
	// First see if this camera has an extrinsics prior
	if( extrinsicsManager.CheckMemberInfo( cameraName ) )
	{
		const PoseSE3& extrinsics = extrinsicsManager.GetInfo( cameraName ).extrinsics;
		RegisterCamera( cameraName, extrinsics, true );
	}
	// If not, see if there if any observed fiducial have poses
	else
	{
		const ros::Time& now = msg->header.stamp;
		BOOST_FOREACH( const FiducialDetection& detection, msg->detections )
		{
			if( fiducialRegistry.count( detection.name ) == 0 ) { continue; }
			
			FiducialRegistration& fidReg = fiducialRegistry[ detection.name ];
			if( fidReg.poses.count( now ) == 0 ) { continue; } 
			
			PoseSE3 fiducialPose = fidReg.poses[now]->value().pose;
			PoseSE3 relPose = EstimateArrayPose( MsgToPoints( detection.points ),
			                                     nullptr,
			                                     MatrixToPoints( fidReg.intrinsics->value().matrix() ) );
			PoseSE3 cameraPose = fiducialPose * relPose.Inverse();
			RegisterCamera( cameraName, cameraPose, false );
			break;
		}
	}
	return cameraRegistry.count( cameraName ) > 0;
}

void CameraArrayCalibrator::RegisterCamera( const std::string& name, const PoseSE3& pose,
                                            bool addPrior )
{
	if( cameraRegistry.count( name ) > 0 ) { return; }
	
	ROS_INFO_STREAM( "Registering camera " << name << " at pose " << pose );
	
	CameraRegistration registration;
	registration.extrinsics = std::make_shared<isam::PoseSE3_Node>();
	registration.extrinsics->init( isam::PoseSE3( pose ) );
	slam->add_node( registration.extrinsics.get() );
	
	registration.intrinsics = std::make_shared<isam::MonocularIntrinsics_Node>();
	registration.intrinsics->init( isam::MonocularIntrinsics( 1, 1, Eigen::Vector2d( 0, 0 ) ) );
	// Not optimizing intrinsics so don't need to add to slam
	
	if( addPrior )
	{
		ROS_INFO_STREAM( "Adding a prior for " << name );
		// TODO
		isam::Noise priorCov = isam::Covariance( isam::eye(6) );
		registration.extrinsicsPrior = 
			std::make_shared<isam::PoseSE3_Prior>( registration.extrinsics.get(),
			                                       isam::PoseSE3( pose ),
			                                       priorCov );
		slam->add_factor( registration.extrinsicsPrior.get() );
	}
	
	cameraRegistry[name] = registration;
}

void CameraArrayCalibrator::RegisterFiducial( const std::string& name )
{
	if( fiducialRegistry.count( name ) > 0 ) { return; }
	
	ROS_INFO_STREAM( "Registering fiducial " << name );
	
	FiducialRegistration registration;
	isam::FiducialIntrinsics intrinsics = FiducialToIsam( fiducialManager.GetInfo( name ) );
	registration.intrinsics = 
	    std::make_shared<isam::FiducialIntrinsics_Node>( intrinsics.name(), intrinsics.dim() );
	registration.intrinsics->init( intrinsics );
	// NOTE Not optimizing intrinsics, so we don't have to add it to the SLAM object
	
	fiducialRegistry[ name ] = registration;
}


}
