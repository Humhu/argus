#include "manycal/ArrayCalibrator.h"

#include "v4l2_cam/ListArrayCameras.h"

#include "atags/AtagCommon.h"

#include <boost/foreach.hpp>

using namespace isam;
using namespace argus_common;

namespace manycal
{

	ArrayCalibrator::ArrayCalibrator( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
		: nodeHandle( nh ), privHandle( ph ),
		slam( std::make_shared<Slam>() ),
		poseGraph( slam )
	{
		std::string arrayName;
		privHandle.getParam( "array_name", arrayName );
		
		detectionSub = nodeHandle.subscribe( "detections", 10, &ArrayCalibrator::DetectionCallback, this );
			
		privHandle.getParam( "tag_size", tagSize );
		
		// Initialize pose graph
		Pose3d pose; // Initializes to 0
		Noise prior = Covariance( 1E2*eye(6) );
		ros::Time now = ros::Time::now();
		poseGraph.AddDynamic( "base", now.toBoost(), pose );
		poseGraph.AddDynamicPrior( "base", now.toBoost(), pose, prior );
	}
	
	ArrayCalibrator::~ArrayCalibrator()
	{
		slam.get_slam()->save( "array_calibrator_graph.isam" );
	}
	
	void ArrayCalibrator::DetectionCallback( const argus_msgs::TagDetection::ConstPtr& msg )
	{
		if( !msg->isNormalized )
		{
			ROS_WARN_STREAM( "Received unnormalized detection message!" );
			return;
		}
		
		// Do odometry first
		Pose3d_Node::Ptr previous = poseGraph.GetDynamicSeries( "base" )->GetLast().point;
		Time previousTime = poseGraph.GetDynamicSeries( "base" )->GetLast().time;
		Time msgTime = msg->header.stamp.toBoost();
		double dt = to_seconds( msgTime - previousTime );
		Noise transNoise = Covariance( 1E2*eye(6)*dt );
		Pose3d displacement;
		Pose3d_Node::Ptr latest = poseGraph.AddDynamicOdometry( "base", msgTime,
																displacement, transNoise );
		
		std::string source = msg->header.frame_id;
		if( cameraRegistry.count( source ) == 0 )
		{
			InitializeCamera( source );
		}
		
		CameraRegistration& camReg = cameraRegistry[ source ];
		
		if( tagRegistry.count( msg->id ) == 0 )
		{
			InitializeTag( source, *msg, tagSize );
		}
		TagRegistration& tagReg = tagRegistry[ msg->id ];

		Noise measNoise = Information( 10*eye(8) );
		Point2d bl( msg->corners[0].x, msg->corners[0].y );
		Point2d br( msg->corners[1].x, msg->corners[1].y );
		Point2d tr( msg->corners[2].x, msg->corners[2].y );
		Point2d tl( msg->corners[3].x, msg->corners[3].y );
		TagCorners meas( bl, br, tr, tl );
		
		Tag_Calibration_Factor::Properties props;
		props.optimizePose = true;
		props.optimizeCamExtrinsics = true;
		props.optimizeCamIntrinsics = true;
		props.optimizeTagLocation = true;
		props.optimizeTagParameters = false;
		Tag_Calibration_Factor::Ptr factor = std::make_shared<Tag_Calibration_Factor>(
			latest.get(), tagReg.extrinsics.get(), camReg.extrinsics.get(),
			camReg.intrinsics.get(), tagReg.intrinsics.get(),
			meas, measNoise );
		slam.add_factor( factor );
		
// 		Jacobian j = factor->jacobian();
// 		std::cout << "Factor jacobian: " << std::endl << j << std::endl;
		
// 		Eigen::MatrixXd sj = matrix_of_sparseMatrix( slam.get_slam()->jacobian() );
// 		std::cout << "System jacobian: " << std::endl << sj << std::endl;
		
// 		TagCorners pred = factor->predict();
// 		std::cout << "Predicted: " << pred << std::endl;
// 		std::cout << "Received: " << meas << std::endl;
		
// 		slam.get_slam()->write( std::cout );
		slam.get_slam()->batch_optimization();
		BOOST_FOREACH( TagRegistry::value_type& item, tagRegistry )
		{
			std::cout << "Tag " << item.first << " " << item.second.extrinsics->value() << std::endl;
		}
		BOOST_FOREACH( CameraRegistry::value_type& item, cameraRegistry )
		{
			std::cout << "Camera " << item.first << " " << item.second.extrinsics->value() << std::endl;
		}
		std::cout << "Current pose " << poseGraph.GetDynamicSeries( "base" )->GetLast().point->value() << std::endl;
		
		std::cout << std::endl;
		
	}
	
	void ArrayCalibrator::InitializeCamera( const std::string& name )
	{
		// NOTE The tag detections will already be in undistorted normalized camera coordinates (??) TODO
		CameraRegistration registration;
		Eigen::Vector2d principalPoint;
		principalPoint << 0.0, 0.0;
		
		// Create intrinsics node
		registration.intrinsics = std::make_shared<MonocularIntrinsics_Node>();
		MonocularIntrinsics intrinsics( 1.0, 1.0, principalPoint );
		registration.intrinsics->init( intrinsics );
		slam.add_node( registration.intrinsics );
		
		// Create intrinsics prior
		Intrinsics_Factor::Ptr priorFactor = std::make_shared<Intrinsics_Factor>( 
			registration.intrinsics.get(), intrinsics, Covariance( 1E3*eye(4) ) );
		slam.add_factor( priorFactor );
		
		// Create extrinsics node and prior
		Pose3d pose( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 );
		registration.extrinsics = poseGraph.AddStatic( name, pose );		
		poseGraph.AddStaticPrior( name, pose, Covariance( 1E6*eye(6) ) );
		
		cameraRegistry[ name ] = registration;
		
		ROS_INFO_STREAM( "Initializing camera " << name );
	}
	
	// Assuming square tag with x-forward out of the tag, y left, z up
	void ArrayCalibrator::InitializeTag( const std::string& camName, const argus_msgs::TagDetection& msg,
										 double size )
	{
		Pose3d bot = poseGraph.GetDynamic( "base", msg.header.stamp.toBoost() )->value();
		Pose3d ext = cameraRegistry.at(camName).extrinsics->value();
		Pose3d displacement( atags::get_tag_pose( msg, size ).ToTransform() );
		Pose3d guess( bot.wTo() * ext.wTo() * displacement.wTo() );
		
		TagRegistration registration;
		registration.intrinsics = std::make_shared<TagIntrinsics_Node>();
		TagIntrinsics tagIntrinsics( size );
		registration.intrinsics->init( tagIntrinsics );
		
		Tag_Intrinsics_Factor::Ptr intrinsicsPrior = std::make_shared<Tag_Intrinsics_Factor>( 
			registration.intrinsics.get(), tagIntrinsics, Covariance( 0.1*eye(1) ) );
		slam.add_factor( intrinsicsPrior );
		
		std::stringstream ss;
		ss << "Tag" << msg.id;
		Noise prior = Information( eye(6) );
		registration.extrinsics = poseGraph.AddStatic( ss.str(), guess );
// 		poseGraph.AddStaticPrior( ss.str(), guess, prior );
		
		tagRegistry[ msg.id ] = registration;
		
		ROS_INFO_STREAM( "Initializing tag " << msg.id << " with relative pose " 
			<< displacement << " at " << guess );
	}
	
}
