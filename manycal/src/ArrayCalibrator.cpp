#include "manycal/ArrayCalibrator.h"
#include "manycal/ManycalCommon.h"
#include "extrinsics_array/ExtrinsicsCalibrationParsers.h"

#include "argus_utils/geometry/GeometryUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/MapUtils.hpp"
#include "argus_utils/utils/YamlUtils.h"

#include "camplex/FiducialCommon.h"

#include <boost/foreach.hpp>

namespace argus
{
ArrayCalibrator::ArrayCalibrator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _nodeHandle( nh ), _privHandle( ph )
{
	GetParam( ph, "extrinsics_init_cov", _extInitCov, PoseSE3::CovarianceMatrix::Identity() );

	// Parse the list of objects to be calibrated
	YAML::Node targets;
	GetParamRequired( ph, "targets", targets );
	YAML::Node::const_iterator iter;
	for( iter = targets.begin(); iter != targets.end(); ++iter )
	{
		const std::string& name = iter->first.as<std::string>();
		ROS_INFO_STREAM( "Parsing target " << name );
		ros::NodeHandle fh( ph.resolveName( "targets/" + name ) );
		_targetRegistry.emplace_back( name, _graph, nh, fh );
		TargetRegistration& target = _targetRegistry.back();

		BOOST_FOREACH( const CameraRegistration::Ptr & cam, target.GetCameras() )
		{
			ROS_INFO_STREAM( "Registering camera " << cam->_name );
			_cameraRegistry[cam->_name] = cam;
		}
		BOOST_FOREACH( const FiducialRegistration::Ptr& fid, target.GetFiducials() )
		{
			ROS_INFO_STREAM( "Registering fiducial " << fid->_name );			
			_fiducialRegistry[fid->_name] = fid;
		}
	}

	std::vector<std::string> topics;
	unsigned int buffLen;
	GetParamRequired( ph, "detection_topics", topics );
	GetParam( ph, "detection_buffer_len", buffLen, (unsigned int) 10 );

	BOOST_FOREACH( const std::string & topic, topics )
	{
		_detSubs.push_back( nh.subscribe( topic,
		                                  buffLen,
		                                  &ArrayCalibrator::DetectionCallback,
		                                  this ) );
		ROS_INFO_STREAM( "Subscribing to detections on " << topic );
	}

	_spinTimer = nh.createTimer( ros::Duration( 1.0 ),
	                             &ArrayCalibrator::TimerCallback,
	                             this );

	// slam->write( std::cout );
}

void ArrayCalibrator::TimerCallback( const ros::TimerEvent& event )
{
	ProcessUntil( event.current_real );
}

void ArrayCalibrator::DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
{
	WriteLock lock( _mutex );
	ImageFiducialDetections dets( *msg );
	if( dets.detections.empty() ) { return; }
	_detBuffer[dets.timestamp.toSec()] = dets;
	ROS_INFO_STREAM( _detBuffer.size() << " detections buffered" );
}

void ArrayCalibrator::ProcessUntil( const ros::Time& until )
{
	if( until < _buffTime )
	{
		ROS_WARN_STREAM( "Cannot process to " << until <<
		                 " as it precedes current head " << _buffTime );
		return;
	}

	// TODO Copy detections out with mutex, then process?
	WriteLock lock( _mutex );	
	while( !_detBuffer.empty() )
	{
		DetectionsBuffer::const_iterator oldest = _detBuffer.begin();
		ImageFiducialDetections dets = oldest->second;

		if( dets.timestamp > until ) { break; }

		BOOST_FOREACH( const FiducialDetection &det, dets.detections )
		{
			ProcessDetection( dets.sourceName, dets.timestamp, det );
		}

		_detBuffer.erase( oldest );
	}
	_buffTime = until;
}

void ArrayCalibrator::ProcessDetection( const std::string& sourceName,
                                        const ros::Time& time,
                                        const FiducialDetection& det  )
{
	if( _cameraRegistry.count( sourceName ) == 0 )
	{
		ROS_WARN_STREAM( "No optimization camera target " << sourceName );
		return;
	}
	if( _fiducialRegistry.count( det.name ) == 0 )
	{
		ROS_WARN_STREAM( "No optimization fiducial target " << det.name );
		return;
	}

	CameraRegistration::Ptr& camera = _cameraRegistry.at( sourceName );
	FiducialRegistration::Ptr& fiducial = _fiducialRegistry.at( det.name );

	if( !camera->IsIntrinsicsInitialized() )
	{
		ROS_ERROR_STREAM( "Uninitialized camera " << camera->_name );
		return;
	}
	if( !fiducial->IsIntrinsicsInitialized() )
	{
		ROS_ERROR_STREAM( "Uninitialized fiducial " << fiducial->_name );
		return;
	}

	isam::PoseSE3_Node* camPoseNode = camera->parent.CreatePoseNode( time );
	isam::PoseSE3_Node* fidPoseNode = fiducial->parent.CreatePoseNode( time );

	bool camPoseGrounded = camera->parent.IsPoseInitialized( time );
	bool fidPoseGrounded = fiducial->parent.IsPoseInitialized( time );
	bool camExtInit = camera->IsExtrinsicsInitialized();
	bool fidExtInit = fiducial->IsExtrinsicsInitialized();

	// If something is uninitialized
	if( !camPoseGrounded ^ !fidPoseGrounded ^ !camExtInit ^ !fidExtInit )
	{
		PoseSE3 relPose = EstimateArrayPose( det,
		                                     IsamToFiducial( fiducial->GetIntrinsicsNode()->value() ) );

		PoseSE3 camExt = IsamToPose( camera->GetExtrinsicsNode()->value() );
		PoseSE3 fidExt = IsamToPose( fiducial->GetExtrinsicsNode()->value() );
		PoseSE3 camPose = IsamToPose( camPoseNode->value() );
		PoseSE3 fidPose = IsamToPose( fidPoseNode->value() );

		if( !camPoseGrounded )
		{
			camPose = fidPose * fidExt * relPose.Inverse() * camExt.Inverse();
			ROS_INFO_STREAM( "Initializing pose of " << camera->parent._name <<
			                 " at " << time << " to " << camPose );
			camera->parent.InitializePose( time, camPose );
		}
		else if( !fidPoseGrounded )
		{
			fidPose = camPose * camExt * relPose * fidExt.Inverse();
			ROS_INFO_STREAM( "Initializing pose of " << fiducial->parent._name <<
			                 " at " << time << " to " << fidPose );
			fiducial->parent.InitializePose( time, fidPose );
		}
		else if( !camExtInit )
		{
			camExt = camPose.Inverse() * fidPose * fidExt * relPose.Inverse();
			ROS_INFO_STREAM( "Initializing extrinsics of " << camera->_name <<
			                 " to " << camExt );
			camera->InitializeExtrinsics( camExt, _extInitCov );
		}
		else // !fidExtInit
		{
			fidExt = fidPose.Inverse() * camPose * camExt * relPose;
			ROS_INFO_STREAM( "Initializing extrinsics of " << fiducial->_name <<
			                 " to " << fidExt );
			fiducial->InitializeExtrinsics( fidExt, _extInitCov );
		}
	}
}

// void ArrayCalibrator::DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
// {
//  ROS_INFO_STREAM( "Detection received." );

//  ros::Time timestamp = msg->header.stamp;
//  std::string camName = msg->header.frame_id;
//  if( cameraRegistry.count( camName ) == 0 ) { return; }
//  CameraRegistration& camReg = cameraRegistry[camName];

//  //const ExtrinsicsInfo& camInfo = extrinsicsManager.GetInfo( camName );
//  PoseSE3 camPose = extrinsicsInterface.GetExtrinsics( camName, ? ? ? );

//  // ALready verified target registered during registration
//  TargetRegistration& camFrameReg = targetRegistry[camInfo.referenceFrame];

//  BOOST_FOREACH( const FiducialDetection &detection, msg->detections )
//  {
//      std::string fidName = detection.name;
//      if( fiducialRegistry.count( fidName ) == 0 ) { return; }
//      FiducialRegistration& fidReg = fiducialRegistry[fidName];

//      // const ExtrinsicsInfo& fidInfo = extrinsicsManager.GetInfo( fidName );
//      PoseSE3 fidPose = extrinsicsInterface.GetExtrinsics( fidName, ? ? ? );
//      // Already verified target registered during registration
//      TargetRegistration& fidFrameReg = targetRegistry[fidInfo.referenceFrame];

//      bool camGrounded = camFrameReg.poses->IsGrounded( timestamp );
//      bool fidGrounded = fidFrameReg.poses->IsGrounded( timestamp );

//      // If one of the two frames is uninitialized, we initialize it using PNP
//      if( !camGrounded || !fidGrounded )
//      {
//          if( !camGrounded && !fidGrounded )
//          {
//              ROS_WARN_STREAM( "Cannot create observation between frames "
//                               << camInfo.referenceFrame << " and "
//                               << fidInfo.referenceFrame << " because both are ungrounded." );
//              return;
//          }

//          PoseSE3 relPose = EstimateArrayPose( detection,
//                                               IsamToFiducial( fidReg.intrinsics->value() ) );

//          ROS_INFO_STREAM( "Fiducial detected at: " << relPose );

//          if( !camGrounded )
//          {
//              // TODO Buffer observation if can't retrieve node, probably due to odometry lagging
//              isam::PoseSE3_Node::Ptr fidFrameNode = fidFrameReg.poses->RetrieveNode( timestamp );
//              if( !fidFrameNode )
//              {
//                  ROS_WARN_STREAM( "Could not retrieve fiducial frame node." );
//                  continue;
//              }
//              PoseSE3 fidFramePose = fidFrameNode->value().pose;
//              PoseSE3 camFramePose = fidFramePose * fidInfo.extrinsics
//                                     * relPose.Inverse() * camInfo.extrinsics.Inverse();
//              ROS_INFO_STREAM( "Creating camera node with prior at pose " << camFramePose );
//              camFrameReg.poses->CreateNode( timestamp, camFramePose );
//              camFrameReg.poses->CreatePrior( timestamp,
//                                              camFramePose,
//                                              isam::Covariance( 1E6 * isam::eye( 6 ) ) );
//              camFrameReg.initialized = true;
//          }
//          else if( !fidGrounded )
//          {
//              // TODO Buffer observation if can't retrieve node, probably due to odometry lagging
//              isam::PoseSE3_Node::Ptr camFrameNode = camFrameReg.poses->RetrieveNode( timestamp );
//              if( !camFrameNode )
//              {
//                  ROS_WARN_STREAM( "Could not retrieve camera frame node." );
//                  continue;
//              }
//              PoseSE3 camFramePose = camFrameNode->value().pose;
//              PoseSE3 fidFramePose = camFramePose * camInfo.extrinsics
//                                     * relPose * fidInfo.extrinsics.Inverse();
//              ROS_INFO_STREAM( "Creating fiducial node with prior at pose " << fidFramePose );
//              fidFrameReg.poses->CreateNode( timestamp, fidFramePose );
//              fidFrameReg.poses->CreatePrior( timestamp,
//                                              fidFramePose,
//                                              isam::Covariance( 1E6 * isam::eye( 6 ) ) );
//              fidFrameReg.initialized = true;
//          }
//      }

//      CreateObservationFactor( camReg, camFrameReg,
//                               fidReg, fidFrameReg,
//                               detection, msg->header.stamp );
//  }

//  ROS_INFO_STREAM( "Running optimization..." );
//  slam->batch_optimization();
// //  std::cout << "===== Iteration: " << observations.size() << std::endl;
// //  slam->write( std::cout );

//  // char buff[50];
//  // sprintf( buff, "%3.2f", observations.size() * 100.0 / targetNumObservations );
//  // std::string percentage( buff );

//  // ROS_INFO_STREAM( "Observations: [" << observations.size() << "/" << targetNumObservations
//  //     << "] (" << percentage << "%");
//  // if( observations.size() >= targetNumObservations ) {
//  //  WriteResults();
//  //  exit( 0 );
//  // }
// }

// void ArrayCalibrator::RegisterFiducial( const std::string& fidName, bool calibrate )
// {
//  // Cache the fiducial lookup information
//  // TODO Support no-prior startup
//  if( !fiducialManager.CheckMemberInfo( fidName, true, ros::Duration( 5.0 ) )
//      || !extrinsicsManager.CheckMemberInfo( fidName, true, ros::Duration( 5.0 ) ) )
//  {
//      ROS_ERROR_STREAM( "Could not retrieve info for " << fidName );
//      return;
//  }

//  // Retrieve the fiducial extrinsics, intrinsics, and reference frame
//  const Fiducial& fiducial = fiducialManager.GetInfo( fidName );
//  const ExtrinsicsInfo& extrinsicsInfo = extrinsicsManager.GetInfo( fidName );

//  if( targetRegistry.count( extrinsicsInfo.referenceFrame ) == 0 )
//  {
//      ROS_ERROR_STREAM( "Parent target " << extrinsicsInfo.referenceFrame
//                                         << " for fiducial " << fidName << " is not registered target." );
//      return;
//  }

//  ROS_INFO_STREAM( "Registering fiducial " << fidName );

//  FiducialRegistration& registration = fiducialRegistry[fidName];
//  registration.name = fidName;
//  registration.optimizeExtrinsics = calibrate;
//  registration.optimizeIntrinsics = false;

//  registration.extrinsics = std::make_shared<isam::PoseSE3_Node>();
//  registration.extrinsics->init( isam::PoseSE3( extrinsicsInfo.extrinsics ) );
//  if( registration.optimizeExtrinsics )
//  {
//      slam->add_node( registration.extrinsics.get() );
//      registration.extrinsicsPrior = std::make_shared<isam::PoseSE3_Prior>
//                                         ( registration.extrinsics.get(),
//                                         isam::PoseSE3( extrinsicsInfo.extrinsics ),
//                                         isam::Covariance( 1E-2 * isam::eye( 6 ) ) );
//      slam->add_factor( registration.extrinsicsPrior.get() );
//  }

//  std::vector<isam::Point3d> pts;
//  pts.reserve( fiducial.points.size() );
//  for( unsigned int i = 0; i < fiducial.points.size(); i++ )
//  {
//      pts.push_back( PointToIsam( fiducial.points[i] ) );
//  }
//  isam::FiducialIntrinsics intrinsics( pts );
//  registration.intrinsics = std::make_shared<isam::FiducialIntrinsics_Node>( intrinsics.name(), intrinsics.dim() );
//  registration.intrinsics->init( intrinsics );
//  if( registration.optimizeIntrinsics )
//  {
//      slam->add_node( registration.intrinsics.get() );
//      // TODO priors
//  }
// }
}
