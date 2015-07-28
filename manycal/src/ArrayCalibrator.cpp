#include "manycal/ArrayCalibrator.h"

#include "argus_common/YamlUtils.h"

#include "v4l2_cam/ListArrayCameras.h"
#include "v4l2_cam/CameraCalibration.h"

#include "atags/AtagCommon.h"

#include <boost/foreach.hpp>

using namespace isam;
using namespace argus_common;
using namespace v4l2_cam;

namespace manycal
{

	ArrayCalibrator::ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ),
		slam( std::make_shared<Slam>() ),
		poseGraph( slam )
	{
		detectionSub = nodeHandle.subscribe( "detections", 10, &ArrayCalibrator::DetectionCallback, this );
			
		// Parse cameras
		YAML::const_iterator iter;
		std::string cameraConfigPath;
		if( !privHandle.getParam( "camera_config", cameraConfigPath ) )
		{
			throw std::runtime_error( "Must specify camera configuration file." );
		}
		
		try{ cameraConfigLog = YAML::LoadFile( cameraConfigPath ); }
		catch( std::exception e )
		{
			throw std::runtime_error( "Could not load camera config file from: " + cameraConfigPath );
		}
		
		for( iter = cameraConfigLog.begin(); iter != cameraConfigLog.end(); iter++ )
		{
			std::string name = iter->first.as<std::string>();
			ROS_DEBUG_STREAM( "Parsing camera " << name << "..." );
			ParseCamera( name, iter->second );
		}
		
		// Parse tags
		std::string tagConfigPath;
		if( !privHandle.getParam( "tag_config", tagConfigPath ) )
		{
			throw std::runtime_error( "Must specify tag configuration file." );
		}
		
		try{ tagConfigLog = YAML::LoadFile( tagConfigPath ); }
		catch( std::exception e )
		{
			throw std::runtime_error( "Could not load tag config file from: " + tagConfigPath );
		}
		
		for( iter = tagConfigLog.begin(); iter != tagConfigLog.end(); iter++ )
		{
			std::string name = iter->first.as<std::string>();
			ROS_DEBUG_STREAM( "Parsing tag " << name << "..." );
			ParseTag( name, iter->second );
		}
		
		// Initialize pose graph
		ROS_DEBUG_STREAM( "Initializing pose graph..." );
		Noise prior = Covariance( 1E2*eye(6) );
		Pose3d basePose( 0, 0, 0, 0, 0, 0 );
		poseGraph.AddStatic( "base", basePose );
		poseGraph.AddStaticPrior( "base", basePose, prior );
		
		Properties slamProps;
		slamProps.quiet = true;
		slam.get_slam()->set_properties( slamProps );
	}
	
	ArrayCalibrator::~ArrayCalibrator()
	{
// 		slam.get_slam()->save( "array_calibrator_graph.isam" );
	}
	
	void ArrayCalibrator::DetectionCallback( const argus_msgs::TagDetection::ConstPtr& msg )
	{
		if( msg->isNormalized )
		{
			ROS_WARN_STREAM( "Received normalized detections! Expected unnormalized detections." );
			return;
		}
		
		// Skip cameras that are not targeted for calibration
		std::string source = msg->header.frame_id;
		if( cameraRegistry.count( source ) == 0 )
		{
			ROS_DEBUG_STREAM( "Skipping camera " << source << " not in registry." );
			return;
		}
		const CameraRegistration& camReg = cameraRegistry[ source ];
		
		// Query tag
		std::string key = GetTagKey( msg->family, msg->id );
		if( tagRegistry.count( key ) == 0 )
		{
			ROS_DEBUG_STREAM( "Skipping tag hash " << key << " not in registry." );
			return;
		}
		if( !tagRegistry[ key ].initialized )
		{
			InitializeTag( key, *msg );
		}
		const TagRegistration& tagReg = tagRegistry[ key ];
		
		// Do odometry first
		Time previousTime = poseGraph.GetDynamicSeries( key )->GetLast().time;
		Time msgTime = msg->header.stamp.toBoost();
		double dt = to_seconds( msgTime - previousTime );
		if( dt < 0 ) 
		{
			ROS_WARN( "Received observation older than latest data." ); // TODO This should be ok
			return;
		}
		
		Pose3d_Node::Ptr latest;
		if( dt < 1E-6 )
		{	// TODO Make this threshold a parameter
			// If time passed is too small, don't add new pose
			latest = poseGraph.GetDynamicSeries( key )->GetLast().point;
		}
		else
		{
			Noise transNoise = Covariance( 1E2*eye(6)*dt );
			Pose3d displacement( 0, 0, 0, 0, 0, 0 ); // TODO Zero-mean displacement assumption!
			latest = poseGraph.AddDynamicOdometry( key, msgTime,
												   displacement, transNoise );
		}

		Noise measNoise = Covariance( 10*eye(8) );
		Point2d bl( msg->corners[0].x, msg->corners[0].y );
		Point2d br( msg->corners[1].x, msg->corners[1].y );
		Point2d tr( msg->corners[2].x, msg->corners[2].y );
		Point2d tl( msg->corners[3].x, msg->corners[3].y );
		TagCorners meas( bl, br, tr, tl );
		
		Pose3d_Node::Ptr baseNode = poseGraph.GetStatic( "base" );
		
		Tag_Calibration_Factor::Properties props;
		props.optimizePose = false;
		props.optimizeCamExtrinsics = true;
		props.optimizeCamIntrinsics = true;
		props.optimizeTagLocation = true;
		props.optimizeTagStructure = false;
		Tag_Calibration_Factor::Ptr factor = std::make_shared<Tag_Calibration_Factor>(
			baseNode.get(), latest.get(), camReg.extrinsics.get(),
			camReg.intrinsics.get(), tagReg.intrinsics.get(),
			meas, measNoise, props );
		slam.add_factor( factor );

		slam.get_slam()->update();
		
		BOOST_FOREACH( TagRegistry::value_type& item, tagRegistry )
		{
			std::cout << "Tag " << item.first << " " << latest->value() << std::endl;
		}
		BOOST_FOREACH( CameraRegistry::value_type& item, cameraRegistry )
		{
			std::cout << "Camera " << item.first << " ext: " << item.second.extrinsics->value() << std::endl;
			std::cout << "Camera " << item.first << " int: " << item.second.intrinsics->value() << std::endl;
		}
		
		std::cout << std::endl;
		
	}
	
	void ArrayCalibrator::ParseCamera( const std::string& name, YAML::Node cam )
	{
		// Read extrinsics
		PoseSE3 extrinsicsPriorSE3( 0, 0, 0, 1, 0, 0, 0 );
		Eigen::MatrixXd extrinsicsCovariance = Eigen::MatrixXd::Identity( 6, 6 );
		if( cam["extrinsics"] )
		{
			YAML::Node extrinsics = cam["extrinsics"];
			YAML::Node extrinsicsCov = extrinsics["covariance"];
			GetPoseYaml( extrinsics, extrinsicsPriorSE3 );
			GetMatrixYaml( extrinsicsCov, extrinsicsCovariance );
		}
		ROS_DEBUG_STREAM( "Extrinsics prior: " << extrinsicsPriorSE3 );
		ROS_DEBUG_STREAM( "Extrinsics covariance: " << extrinsicsCovariance );
		Pose3d extrinsicsPrior( extrinsicsPriorSE3.ToTransform() );
		Noise extrinsicsNoise = Covariance( extrinsicsCovariance );
		
		Eigen::Vector2d pp;
		pp << 320, 240; // Defaults to half VGA
		MonocularIntrinsics intrinsicsPrior( 500, 500, pp );
		Eigen::MatrixXd intrinsicsCovariance = 1E6*Eigen::MatrixXd::Identity( 4, 4 );
		if( cam["intrinsics"] )
		{
			YAML::Node intrinsics = cam["intrinsics"];
			YAML::Node intrinsicsCov = intrinsics["covariance"];
		
			std::string calibPath = intrinsics["path"].as<std::string>();
			CameraCalibration calibration( calibPath );
			Eigen::Vector2d pp;
			pp << calibration.GetCx(), calibration.GetCy();
			intrinsicsPrior = MonocularIntrinsics( calibration.GetFx(), calibration.GetFy(), pp );				
			
			GetMatrixYaml( intrinsicsCov, intrinsicsCovariance );
		}
		
		ROS_DEBUG_STREAM ("Intrinsics prior: " << intrinsicsPrior.vector().transpose() );
		ROS_DEBUG_STREAM( "Intrinsics covariance: " << intrinsicsCovariance );
		Noise intrinsicsNoise = Covariance( intrinsicsCovariance );
		
		CameraRegistration registration;
		
		// Create intrinsics node
		registration.intrinsics = std::make_shared<MonocularIntrinsics_Node>();
		registration.intrinsics->init( intrinsicsPrior );
		slam.add_node( registration.intrinsics );
		
		// Create intrinsics prior
		Intrinsics_Factor::Ptr priorFactor = std::make_shared<Intrinsics_Factor>( 
			registration.intrinsics.get(), intrinsicsPrior, intrinsicsNoise );
		slam.add_factor( priorFactor );
		
		// Create extrinsics node and prior
		registration.extrinsics = poseGraph.AddStatic( name, extrinsicsPrior );
		poseGraph.AddStaticPrior( name, extrinsicsPrior, extrinsicsNoise );
		
		cameraRegistry[ name ] = registration;
	}
	
	void ArrayCalibrator::ParseTag( const std::string& name, YAML::Node tag )
	{
		unsigned int id = tag["id"].as<unsigned int>();
		std::string family = tag["family"].as<std::string>();
		
		YAML::Node intrinsics = tag["intrinsics"];
		double tsize = intrinsics["size"].as<double>();
		
		TagIntrinsics tagIntrinsics( tsize, Eigen::VectorXd::Zero(3) );
		TagIntrinsics_Node::Ptr intrinsicsNode = std::make_shared<TagIntrinsics_Node>();
		intrinsicsNode->init( tagIntrinsics );
		
		TagRegistration registration;
		registration.name = name;
		registration.id = id;
		registration.family = family;
		registration.intrinsics = intrinsicsNode;
		registration.initialized = false;
		
		std::string key = GetTagKey( family, id );
		if( tagRegistry.count( key ) > 0 )
		{
			throw std::runtime_error( "Attempted to add repeat tag hash: " + key );
		}
		tagRegistry[ key ] = registration;
		
	}
	
	// Assuming square tag with x-forward out of the tag, y left, z up
	void ArrayCalibrator::InitializeTag( const std::string& key, const argus_msgs::TagDetection& msg )
	{
		Pose3d bot = poseGraph.GetStatic( "base" )->value();
		
		const CameraRegistration& camReg = cameraRegistry[ msg.header.frame_id ];
		Pose3d ext = camReg.extrinsics->value();
		MonocularIntrinsics intr = camReg.intrinsics->value();
		Eigen::Vector2d pp = intr.principalPoint();
		
		const TagRegistration& registration = tagRegistry[ key ];
		double tsize = registration.intrinsics->value().tagSize;
		Pose3d displacement(
			atags::get_tag_pose( msg, tsize, intr.fx(), intr.fy(), pp(0), pp(1) ).ToTransform() );
		Pose3d guess( bot.wTo() * ext.wTo() * displacement.wTo() );
		
		poseGraph.AddDynamic( key, msg.header.stamp.toBoost(), guess );
		
		ROS_DEBUG_STREAM( "Initializing tag " << key << " with relative pose " 
			<< displacement << " at " << guess );
		tagRegistry[ key ].initialized = true;
	}
	
	std::string ArrayCalibrator::GetTagKey( const std::string& tagFamily, unsigned int id )
	{
		return "tag_" + tagFamily + std::to_string( id );
	}
	
}
