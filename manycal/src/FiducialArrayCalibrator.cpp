#include "manycal/FiducialArrayCalibrator.h"
#include "fiducials/PoseEstimation.h"

#include <boost/foreach.hpp>

using namespace fiducials;
using namespace argus_msgs;

namespace argus
{
	
FiducialArrayCalibrator::FiducialArrayCalibrator( const ros::NodeHandle& nh,
                                                  const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), fiducialManager( lookupInterface ),
extrinsicsManager( lookupInterface )
{
	if( !ph.getParam( "source_camera", sourceCamera ) )
	{
		ROS_ERROR_STREAM( "Must specify source camera for calibration." );
		exit( -1 );
	}
	
	if( !ph.getParam( "reference_frame", referenceFrame ) )
	{
		ROS_ERROR_STREAM( "Must specify reference frame ID." );
		exit( -1 );
	}
	
	double writeFrequency;
	ph.param( "write_frequency", writeFrequency, 1.0 );
	writeTimer = nodeHandle.createTimer( ros::Duration( 1.0 / writeFrequency ),
	                                     &FiducialArrayCalibrator::WriteResults,
	                                     this );

	cameraIntrinsics = std::make_shared <isam::MonocularIntrinsics_Node>();
	cameraIntrinsics->init( isam::MonocularIntrinsics( 1.0, 1.0, Eigen::Vector2d( 0, 0 ) ) );
	
	std::string lookupNamespace;
	ph.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
	lookupInterface.SetLookupNamespace( lookupNamespace );

	slam = std::make_shared<isam::Slam>();
	
	fiducialReference = std::make_shared<isam::PoseSE3_Node>();
	fiducialReference->init( isam::PoseSE3() );
	// NOTE Don't optimize this
	
	detSub = nodeHandle.subscribe( "detections", 
	                               0, 
	                               &FiducialArrayCalibrator::DetectionCallback,
	                               this );
}

FiducialArrayCalibrator::~FiducialArrayCalibrator() 
{}

void FiducialArrayCalibrator::WriteResults( const ros::TimerEvent& event ) 
{
	BOOST_FOREACH( const FiducialRegistry::value_type& item, fiducialRegistry )
	{
		const std::string& name = item.first;
		const FiducialRegistration& registration = item.second;
		PoseSE3 extrinsics = registration.extrinsics->value().pose;
		std::cout << "Fiducial " << name << " pose " << extrinsics << std::endl;
		
		ExtrinsicsInfo info;
		info.referenceFrame = referenceFrame;
		info.extrinsics = extrinsics;

		extrinsicsManager.WriteMemberInfo( name, info, true );
	}
}

void FiducialArrayCalibrator::DetectionCallback( const ImageFiducialDetections::ConstPtr& msg )
{
	if( msg->header.frame_id != sourceCamera ) 
	{
		ROS_WARN_STREAM( "Received image from non-source camera." );
		return;
	}
	
	if( msg->detections.size() < 2 ) 
	{
		ROS_INFO_STREAM( "Not enough tags in image. Skipping..." );
		return; 
	}
	
	// Initialize the camera pose for this observation using an initialized fiducial
	isam::PoseSE3_Node::Ptr cameraNode;
	BOOST_FOREACH( const FiducialDetection& detection, msg->detections )
	{
		// If the fiducial is uninitialized, see if there's a pose prior for it
		if( fiducialRegistry.count( detection.name ) == 0 )
		{
			if( HasExtrinsicsPrior( detection.name ) && HasIntrinsicsPrior( detection.name ) )
			{
				const PoseSE3& prior = extrinsicsManager.GetInfo( detection.name ).extrinsics;
				if( !RegisterFiducial( detection.name, prior, true ) ) { continue; }
			}
		}
		
		if( fiducialRegistry.count( detection.name ) > 0 )
		{
			// Initialize the first camera position based on the observation+
			const FiducialRegistration& fidReg = fiducialRegistry[ detection.name ];
			PoseSE3 relPose = EstimateArrayPose( MsgToPoints( detection.points ),
			                                     nullptr,
			                                     MatrixToPoints( fidReg.intrinsics->value().matrix() ) );
			PoseSE3 fiducialPose = fidReg.extrinsics->value().pose;
			PoseSE3 cameraPose = fiducialPose * relPose.Inverse();
			
// 			ROS_INFO_STREAM( "Initializing camera pose to: " << cameraPose );
			
			cameraNode = std::make_shared <isam::PoseSE3_Node>();
			cameraNode->init( isam::PoseSE3( cameraPose ) );
			cameraPoses.push_back( cameraNode );
			slam->add_node( cameraNode.get() );
			
			// TODO Prior?
			
			break;
		}
	}
	
	if( !cameraNode )
	{
		ROS_INFO_STREAM( "No initialized fiducials detected in image. Discarding observation." );
		return;
	}
	
	// Create observation factors for each fiducial detection
	BOOST_FOREACH( const FiducialDetection& detection, msg->detections )
	{
		// If we haven't seen this fiducial before, initialize it
		if( fiducialRegistry.count( detection.name ) == 0 )
		{
			if( !HasIntrinsicsPrior( detection.name ) ) { continue; }
			
			const Fiducial& intrinsics = fiducialManager.GetInfo( detection.name );
			PoseSE3 relPose = EstimateArrayPose( MsgToPoints( detection.points ),
			                                     nullptr,
			                                     MsgToPoints( intrinsics.points ) );
			PoseSE3 cameraPose = cameraNode->value().pose;
			PoseSE3 fiducialPose = cameraPose * relPose;
			RegisterFiducial( detection.name, fiducialPose );
		}
		const FiducialRegistration& reg = fiducialRegistry[ detection.name ];
		
		// TODO
		double imageCoordinateErr = std::pow( 0.03, 2 );
		isam::Noise cov = isam::Covariance( imageCoordinateErr * isam::eye( 2*detection.points.size() ) );
		
		isam::FiducialFactor::Properties props;
		props.optCamReference = true;
		props.optCamIntrinsics = false;
		props.optCamExtrinsics = false;
		props.optFidReference = false;
		props.optFidIntrinsics = false;
		props.optFidExtrinsics = true;
		
		isam::FiducialFactor::Ptr factor = std::make_shared<isam::FiducialFactor>
		    ( cameraNode.get(), cameraIntrinsics.get(), nullptr,
		      fiducialReference.get(), reg.intrinsics.get(), reg.extrinsics.get(),
		      DetectionToIsam( detection ), cov, props );
		slam->add_factor( factor.get() );
		observations.push_back( factor );
	}
	
	slam->batch_optimization();
}

// Check to see if there is a prior for the name
bool FiducialArrayCalibrator::HasExtrinsicsPrior( const std::string& name )
{
	if( !extrinsicsManager.HasMember( name ) )
	{
		if( !extrinsicsManager.ReadMemberInfo( name, false ) ||
		    extrinsicsManager.GetInfo( name ).referenceFrame != referenceFrame )
		{ 
			return false; 
		}
		ROS_INFO_STREAM( "Found extrinsics prior for " << name );
	}
	return true;
}

bool FiducialArrayCalibrator::HasIntrinsicsPrior( const std::string& name )
{
	if( !fiducialManager.HasMember( name ) )
	{
		if( !fiducialManager.ReadMemberInfo( name, false ) )
		{
			ROS_WARN_STREAM( "Could not retrieve fiducial info for " << name );
			return false;
		}
		ROS_INFO_STREAM( "Found intrinsics for " << name );
	}
	return true;
}

bool FiducialArrayCalibrator::RegisterFiducial( const std::string& name,
                                                const PoseSE3& pose,
                                                bool addPrior )
{
	if( fiducialRegistry.count( name ) > 0 ) { return true; }
	
	ROS_INFO_STREAM( "Registering fiducial " << name << " with pose " << pose );
	
	FiducialRegistration registration;
	isam::FiducialIntrinsics intrinsics = FiducialToIsam( fiducialManager.GetInfo( name ) );
	registration.intrinsics = 
	    std::make_shared<isam::FiducialIntrinsics_Node>( intrinsics.name(), intrinsics.dim() );
	registration.intrinsics->init( intrinsics );
	// NOTE Not optimizing intrinsics, so we don't have to add it to the SLAM object
	
	registration.extrinsics = std::make_shared<isam::PoseSE3_Node>();
	registration.extrinsics->init( isam::PoseSE3( pose ) );
	slam->add_node( registration.extrinsics.get() );
	
	if( addPrior )
	{
		isam::Noise priorCov = isam::Covariance( 1E3 * isam::eye(6) );
		registration.extrinsicsPrior = 
			std::make_shared<isam::PoseSE3_Prior>( registration.extrinsics.get(),
			                                       isam::PoseSE3( pose ),
			                                       priorCov );
		slam->add_factor( registration.extrinsicsPrior.get() );
	}

	fiducialRegistry[ name ] = registration;
	return true;
}

}
