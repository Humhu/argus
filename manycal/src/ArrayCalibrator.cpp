#include "manycal/ArrayCalibrator.h"
#include "manycal/ManycalCommon.h"
#include "extrinsics_array/ExtrinsicsCalibrationParsers.h"

#include "argus_utils/GeometryUtils.h"
#include "argus_utils/MatrixUtils.h"
#include "argus_utils/ParamUtils.h"
#include "argus_utils/MapUtils.hpp"
#include "argus_utils/YamlUtils.h"

#include "fiducials/FiducialCommon.h"
#include "fiducials/PoseEstimation.h"

#include <boost/foreach.hpp>

using namespace argus_utils;
using namespace argus_msgs;
using namespace extrinsics_array;
using namespace fiducials;
using namespace fieldtrack;

namespace manycal
{

ArrayCalibrator::ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), 
fiducialManager( lookup ), extrinsicsManager( lookup ), targetManager( lookup ),
slam( std::make_shared<isam::Slam>() )
{
	std::string lookupNamespace;
	ph.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
	lookup.SetLookupNamespace( lookupNamespace );
	
	// Parse the list of objects to be calibrated
	XmlRpc::XmlRpcValue targetsXml;
	if( !ph.getParam( "targets", targetsXml ) )
	{
		ROS_ERROR_STREAM( "Calibration targets must be specified." );
	}
	
	YAML::Node targetsYaml = XmlToYaml( targetsXml );
	YAML::Node::const_iterator iter;
	for( iter = targetsYaml.begin(); iter != targetsYaml.end(); iter++ )
	{
		const std::string& name = iter->first.as<std::string>();
		RegisterTarget( name, iter->second );
	}

	writeServer = privHandle.advertiseService( "write_results", 
	                                           &ArrayCalibrator::WriteResults,
	                                           this );
}

bool ArrayCalibrator::WriteResults( WriteCalibration::Request& req,
                                    WriteCalibration::Response& res )
{
	YAML::Node yaml;
	isam::Covariances covs = slam->covariances();
	std::list<isam::Node*> nodes;
	

	BOOST_FOREACH( const CameraRegistry::value_type& item, cameraRegistry )
	{
		const std::string& name = item.first;
		const CameraRegistration& registration = item.second;
		PoseSE3 extrinsics = registration.extrinsics->value().pose;
		ROS_INFO_STREAM( "Camera " << name << " pose " << extrinsics );
		
		extrinsicsManager.GetInfo( name ).extrinsics = extrinsics;
		extrinsicsManager.WriteMemberInfo( name );

		if( registration.optimizeExtrinsics || registration.optimizeIntrinsics )
		{
			nodes.clear();
			nodes.push_back( registration.extrinsics.get() );
			Eigen::MatrixXd S = covs.marginal( nodes );
			ROS_INFO_STREAM( "Covariance: " << std::endl << S );

			YAML::Node node;
			PopulateExtrinsicsCalibration( extrinsicsManager.GetInfo( name ), node );
			yaml[name] = node;
		}
	}

	BOOST_FOREACH( const FiducialRegistry::value_type& item, fiducialRegistry )
	{
		const std::string& name = item.first;
		const FiducialRegistration& registration = item.second;
		PoseSE3 extrinsics = registration.extrinsics->value().pose;
		ROS_INFO_STREAM( "Fiducial " << name << " pose " << extrinsics );
		
		extrinsicsManager.GetInfo( name ).extrinsics = extrinsics;
		extrinsicsManager.WriteMemberInfo( name );

		if( registration.optimizeExtrinsics || registration.optimizeIntrinsics )
		{
			nodes.clear();
			nodes.push_back( registration.extrinsics.get() );
			Eigen::MatrixXd S = covs.marginal( nodes );
			ROS_INFO_STREAM( "Covariance: " << std::endl << S );

			YAML::Node node;
			PopulateExtrinsicsCalibration( extrinsicsManager.GetInfo( name ), node );
			yaml[name] = node;
		}
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

void ArrayCalibrator::RegisterTarget( const std::string& targetName,
                                      const YAML::Node& yaml )
{
	if( targetRegistry.count( targetName ) > 0 ) { return; }

	if( !targetManager.CheckMemberInfo( targetName, true, ros::Duration( 5.0 ) ) )
	{
		ROS_ERROR_STREAM( "Could not retrieve target info for " << targetName );
		exit( -1 );
	}

	ROS_INFO_STREAM( "Registering target " << targetName );
	TargetRegistration registration;
	
	const TargetInfo& info = targetManager.GetInfo( targetName );
	
	// Create appropriate pose graph for target
	switch( info.motionMode )
	{
		case MOTION_INDEPENDENT:
			// TODO
			registration.poses = std::make_shared<JumpPoseGraphType>( slam );
			break;
		case MOTION_STATIC:
			registration.poses = std::make_shared<StaticPoseGraphType>( slam );
			break;
		case MOTION_MOVING:
			registration.poses = std::make_shared<OdometryGraphType>( slam );
			break;
		default:
			ROS_ERROR_STREAM( "Invalid motion mode for target " << targetName );
			return;
	}

	registration.initialized = false;
	registration.optimizePoses = yaml["optimize_poses"].as<bool>();

	// TODO Have all priors at same time
	ros::Time now = ros::Time::now();
	if( yaml["initial_pose"] )
	{
		PoseSE3 initialPose;
		if( !GetPoseYaml( yaml["initial_pose"], initialPose ) )
		{
			ROS_ERROR_STREAM( "Error parsing initial_pose for " << targetName );
			return;
		}

		registration.poses->CreateNode( now, initialPose );

		if( registration.optimizePoses )
		{ 
			typedef std::vector<double> Doubles;
			PoseSE3::CovarianceMatrix initialCovariance;
			if( !yaml["initial_covariance"] )
			{
				ROS_ERROR_STREAM( "Must specify initial_covariance if initial_pose"
				                  << " given and optimize_poses set." );
				return;
			}
			Doubles covValues = yaml["initial_covariance"].as<Doubles>();
			if( !ParseMatrix( covValues, initialCovariance ) )
			{
				ROS_ERROR_STREAM( "Could not parse initial covariance for " << targetName );
				return;
			}

			registration.poses->CreatePrior( now, 
			                                 isam::PoseSE3( initialPose ), 
			                                 isam::Covariance( initialCovariance ) );
			registration.initialized = true;
		}
	}

	// Subscribe to odometry and detection topics if specified in config
	if( !info.odometryTopic.empty() )
	{
		// TODO Specify buffer size?
		ROS_INFO_STREAM( "Registering to odometry topic " << info.odometryTopic );
		registration.odometrySub = nodeHandle.subscribe
		    ( info.odometryTopic, 10, &ArrayCalibrator::OdometryCallback, this );

	}
	if( !info.detectionTopic.empty() )
	{
		ROS_INFO_STREAM( "Registering to detection topic " << info.detectionTopic );
		registration.detectionSub = nodeHandle.subscribe
		    ( info.detectionTopic, 10, &ArrayCalibrator::DetectionCallback, this );
	}

	targetRegistry[ targetName ] = registration;

	typedef std::vector<std::string> Strings;
	Strings camTargets, fidTargets;
	if( yaml["camera_groups"] ) { camTargets = yaml["camera_groups"].as<Strings>(); }
	if( yaml["fiducial_groups"] ) { fidTargets = yaml["fiducial_groups"].as<Strings>(); }
	std::set<std::string> camTargetSet = make_set( camTargets );
	std::set<std::string> fidTargetSet = make_set( fidTargets );

	BOOST_FOREACH( const TargetInfo::CameraGroupRegistry::value_type& iter, 
	               info.cameraGroups )
	{
		const std::string& groupName = iter.first;
		const CameraGroupInfo& camGroup = iter.second;
		bool calibrate = camTargetSet.count( groupName ) > 0;
		BOOST_FOREACH( const std::string& camName, camGroup.cameras )
		{
			RegisterCamera( camName, calibrate );
		}
	}
	BOOST_FOREACH( const TargetInfo::FiducialGroupRegistry::value_type& iter, 
	               info.fiducialGroups )
	{
		const std::string& groupName = iter.first;
		const FiducialGroupInfo& fiducialGroup = iter.second;
		bool calibrate = fidTargetSet.count( groupName ) > 0;
		BOOST_FOREACH( const std::string& fidName, fiducialGroup.fiducials )
		{
			RegisterFiducial( fidName, calibrate );
		}
	}
}

// TODO Change this back to relative_pose so we can catch skipped time steps?
void ArrayCalibrator::OdometryCallback( const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg )
{
	std::string targetName = msg->header.frame_id;
	
	if( targetRegistry.count( targetName ) == 0 ) { return; }
	
	// Parse message fields
	PoseSE3 displacement = MsgToPose( msg->pose.pose );
	PoseSE3::CovarianceMatrix cov;
	ParseMatrix<PoseSE3::CovarianceMatrix>( msg->pose.covariance, cov );
	
	// Add odometry to graph
	TargetRegistration& registration = targetRegistry[ targetName ];

	// TODO This can fail if detections precede odometry!
	ros::Time latestTime = registration.poses->LatestIndex();
	if( latestTime > msg->header.stamp ) 
	{ 
		ROS_WARN_STREAM( "Odometry preceded!" );
		return; 
	}

	ros::Time nextTime = msg->header.stamp;
	PoseSE3 nextPose = registration.poses->RetrieveNode( latestTime )->value().pose * displacement;
	registration.poses->CreateNode( nextTime, nextPose );
	registration.poses->CreateEdge( latestTime, nextTime,
	                                isam::PoseSE3( displacement ),
	                                isam::Covariance( cov ) );
}

void ArrayCalibrator::DetectionCallback( const ImageFiducialDetections::ConstPtr& msg )
{
	ROS_INFO_STREAM( "Detection received." );

	ros::Time timestamp = msg->header.stamp;
	std::string camName = msg->header.frame_id;
	if( cameraRegistry.count( camName ) == 0 ) { return; }
	CameraRegistration& camReg = cameraRegistry[ camName ];
	
	const extrinsics_array::ExtrinsicsInfo& camInfo = extrinsicsManager.GetInfo( camName );
	
	// ALready verified target registered during registration
	TargetRegistration& camFrameReg = targetRegistry[ camInfo.referenceFrame ];
	
	BOOST_FOREACH( const FiducialDetection& detection, msg->detections )
	{
		std::string fidName = detection.name;
		if( fiducialRegistry.count( fidName ) == 0 ) { return; }
		FiducialRegistration& fidReg = fiducialRegistry[ fidName ];
		
		const extrinsics_array::ExtrinsicsInfo& fidInfo = extrinsicsManager.GetInfo( fidName );
		// Already verified target registered during registration
		TargetRegistration& fidFrameReg = targetRegistry[ fidInfo.referenceFrame ];

		bool camGrounded = camFrameReg.poses->IsGrounded( timestamp );
		bool fidGrounded = fidFrameReg.poses->IsGrounded( timestamp );

		// If one of the two frames is uninitialized, we initialize it using PNP
		if( !camGrounded || !fidGrounded )
		{
			if( !camGrounded && !fidGrounded )
			{
				ROS_WARN_STREAM( "Cannot create observation between frames "
				                 << camInfo.referenceFrame << " and " 
				                 << fidInfo.referenceFrame << " because both are ungrounded.");
				return;
			}

			PoseSE3 relPose = EstimateArrayPose( MsgToPoints( detection.points ),
		                                         nullptr,
		                                         MatrixToPoints( fidReg.intrinsics->value().matrix() ) );
		
			ROS_INFO_STREAM( "Fiducial detected at: " << relPose );
			
			if( !camGrounded )
			{
				isam::PoseSE3_Node::Ptr fidFrameNode = fidFrameReg.poses->RetrieveNode( timestamp );
				if( !fidFrameNode ) 
				{ 
					ROS_WARN_STREAM( "Could not retrieve fiducial frame node." );
					continue; 
				}
				PoseSE3 fidFramePose = fidFrameNode->value().pose;
				PoseSE3 camFramePose = fidFramePose * fidInfo.extrinsics 
					* relPose.Inverse() * camInfo.extrinsics.Inverse();
				ROS_INFO_STREAM( "Creating camera node with prior at pose " << camFramePose );
				camFrameReg.poses->CreateNode( timestamp, camFramePose );
				camFrameReg.poses->CreatePrior( timestamp, 
				                                camFramePose, 
				                                isam::Covariance( 1E6 * isam::eye(6) ) );
				camFrameReg.initialized = true;
			}
			else if( !fidGrounded )
			{
				isam::PoseSE3_Node::Ptr camFrameNode = camFrameReg.poses->RetrieveNode( timestamp );
				if( !camFrameNode ) 
				{ 
					ROS_WARN_STREAM( "Could not retrieve camera frame node." );
					continue; 
				}
				PoseSE3 camFramePose = camFrameNode->value().pose;
				PoseSE3 fidFramePose = camFramePose * camInfo.extrinsics 
					* relPose * fidInfo.extrinsics.Inverse();
				ROS_INFO_STREAM( "Creating fiducial node with prior at pose " << fidFramePose );
				fidFrameReg.poses->CreateNode( timestamp, fidFramePose );
				fidFrameReg.poses->CreatePrior( timestamp, 
				                                fidFramePose, 
				                                isam::Covariance( 1E6 * isam::eye(6) ) );
				fidFrameReg.initialized = true;
			}
		}
		
		CreateObservationFactor( camReg, camFrameReg,
		                         fidReg, fidFrameReg,
		                         detection, msg->header.stamp );
	}
	
	slam->batch_optimization();
// 	std::cout << "===== Iteration: " << observations.size() << std::endl;
// 	slam->write( std::cout );
	
	// char buff[50];
	// sprintf( buff, "%3.2f", observations.size() * 100.0 / targetNumObservations );
	// std::string percentage( buff );
	
	// ROS_INFO_STREAM( "Observations: [" << observations.size() << "/" << targetNumObservations 
	//     << "] (" << percentage << "%");
	// if( observations.size() >= targetNumObservations ) {
	// 	WriteResults();
	// 	exit( 0 ); 
	// }
}

void ArrayCalibrator::CreateObservationFactor( CameraRegistration& camera,
                                               TargetRegistration& cameraFrame,
                                               FiducialRegistration& fiducial,
                                               TargetRegistration& fiducialFrame,
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
	
	isam::PoseSE3_Node::Ptr cameraFramePose = cameraFrame.poses->RetrieveNode( t );
	isam::PoseSE3_Node::Ptr fiducialFramePose = fiducialFrame.poses->RetrieveNode( t );
	
	if( !cameraFramePose || !fiducialFramePose )
	{
		ROS_WARN_STREAM( "Could not retrieve poses to create observation." );
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

void ArrayCalibrator::RegisterCamera( const std::string& camName, bool calibrate )
{
	// Cache the camera lookup information
	if( !extrinsicsManager.CheckMemberInfo( camName, true, ros::Duration( 5.0 ) ) )
	{
		ROS_ERROR_STREAM( "Could not retrieve info for camera: " << camName );
		return;
	}
	const ExtrinsicsInfo& extrinsicsInfo = extrinsicsManager.GetInfo( camName );
	if( targetRegistry.count( extrinsicsInfo.referenceFrame ) == 0)
	{
		ROS_ERROR_STREAM( "Parent target " << extrinsicsInfo.referenceFrame
		                  << " for camera " << camName << " is not registered target." );
		return;
	}
	
	ROS_INFO_STREAM( "Registering camera " << camName );

	CameraRegistration& registration = cameraRegistry[ camName ];
	registration.name = camName;
	registration.optimizeExtrinsics = calibrate;
	registration.optimizeIntrinsics = false; // TODO
	
	registration.extrinsics = std::make_shared <isam::PoseSE3_Node>();
	registration.extrinsics->init( isam::PoseSE3( extrinsicsInfo.extrinsics ) );
	
	if( registration.optimizeExtrinsics )
	{
		slam->add_node( registration.extrinsics.get() );
		// TODO Specify prior covariance
		registration.extrinsicsPrior = std::make_shared <isam::PoseSE3_Prior>
		    ( registration.extrinsics.get(), 
		      isam::PoseSE3( extrinsicsInfo.extrinsics ),
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
}

void ArrayCalibrator::RegisterFiducial( const std::string& fidName, bool calibrate )
{
	// Cache the fiducial lookup information
	if( !fiducialManager.CheckMemberInfo( fidName, true, ros::Duration( 5.0 ) )
	    || !extrinsicsManager.CheckMemberInfo( fidName, true, ros::Duration( 5.0 ) ) )
	{
		ROS_ERROR_STREAM( "Could not retrieve info for " << fidName );
		return;
	}
	
	// Retrieve the fiducial extrinsics, intrinsics, and reference frame
	const Fiducial& fiducial = fiducialManager.GetInfo( fidName );
	const ExtrinsicsInfo& extrinsicsInfo = extrinsicsManager.GetInfo( fidName );

	if( targetRegistry.count( extrinsicsInfo.referenceFrame ) == 0 )
	{
		ROS_ERROR_STREAM( "Parent target " << extrinsicsInfo.referenceFrame 
		                  << " for fiducial " << fidName << " is not registered target." );
		return;
	}

	ROS_INFO_STREAM( "Registering fiducial " << fidName );

	FiducialRegistration& registration = fiducialRegistry[ fidName ];
	registration.name = fidName;
	registration.optimizeExtrinsics = calibrate;
	registration.optimizeIntrinsics = false;
	
	registration.extrinsics = std::make_shared <isam::PoseSE3_Node>();
	registration.extrinsics->init( isam::PoseSE3( extrinsicsInfo.extrinsics ) );
	if( registration.optimizeExtrinsics )
	{
		slam->add_node( registration.extrinsics.get() );
		registration.extrinsicsPrior = std::make_shared <isam::PoseSE3_Prior>
		    ( registration.extrinsics.get(), 
			  isam::PoseSE3( extrinsicsInfo.extrinsics ),
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
}

}
