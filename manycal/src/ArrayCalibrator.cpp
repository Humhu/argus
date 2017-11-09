#include "manycal/ArrayCalibrator.h"
#include "manycal/ManycalCommon.h"
#include "manycal/Factors.h"

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
	ros::NodeHandle gh( ph.resolveName( "graph" ) );
	_graph = GraphOptimizer( gh );

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
		BOOST_FOREACH( const FiducialRegistration::Ptr & fid, target.GetFiducials() )
		{
			ROS_INFO_STREAM( "Registering fiducial " << fid->_name );
			_fiducialRegistry[fid->_name] = fid;
		}
	}

	std::vector<std::string> topics;
	unsigned int buffLen;
	GetParamRequired( ph, "detection_topics", topics );
	GetParam( ph, "detection_buffer_len", buffLen, (unsigned int) 10 );

	double maxLag;
	GetParam( ph, "max_detection_buffer_lag", maxLag, 5.0 );
	_maxLag = ros::Duration( maxLag );

	BOOST_FOREACH( const std::string & topic, topics )
	{
		_detSubs.push_back( nh.subscribe( topic,
		                                  buffLen,
		                                  &ArrayCalibrator::DetectionCallback,
		                                  this ) );
		ROS_INFO_STREAM( "Subscribing to detections on " << topic );
	}

	double spinLag;
	GetParamRequired( ph, "spin_lag", spinLag );
	_spinLag = ros::Duration( spinLag );

	double imgStd;
	GetParam( ph, "detection_img_std", imgStd, 0.01 );
	_detectionImgVariance = imgStd * imgStd;
	ROS_INFO_STREAM( "Using normalized image coordinate std dev " << imgStd );

	double spinRate;
	GetParam( ph, "spin_rate", spinRate, 1.0 );
	_spinTimer = nh.createTimer( ros::Duration( 1.0 / spinRate ),
	                             &ArrayCalibrator::TimerCallback,
	                             this );
}

ArrayCalibrator::~ArrayCalibrator()
{
	BOOST_FOREACH( const TargetRegistration &target, _targetRegistry )
	{
		target.SaveExtrinsics();
	}
}

bool ArrayCalibrator::WriteHandler( manycal::WriteCalibration::Request& req,
                                    manycal::WriteCalibration::Response& res )
{
	WriteLock lock( _optMutex );	
	_graph.GetOptimizer().batch_optimization();
	BOOST_FOREACH( const TargetRegistration &target, _targetRegistry )
	{
		target.SaveExtrinsics();
	}
	return true;
}

void ArrayCalibrator::TimerCallback( const ros::TimerEvent& event )
{
	ProcessUntil( event.current_real - _spinLag );

	// _graph.GetOptimizer().write( std::cout );
	Print();
	// TODO Switch back to .update()?
	WriteLock lock( _optMutex );
	_graph.GetOptimizer().batch_optimization();
}

void ArrayCalibrator::Print()
{
	typedef CameraRegistry::value_type CameraItem;
	BOOST_FOREACH( const CameraItem &item, _cameraRegistry )
	{
		CameraRegistration::Ptr cam = item.second;
		ROS_INFO_STREAM( "Camera " << cam->_name << " relative to " <<
		                 cam->parent._name << " has extrinsics " << cam->GetExtrinsicsPose() );
	}

	typedef FiducialRegistry::value_type FiducialItem;
	BOOST_FOREACH( const FiducialItem &item, _fiducialRegistry )
	{
		FiducialRegistration::Ptr fid = item.second;
		ROS_INFO_STREAM( "Fiducial " << fid->_name << " relative to " <<
		                 fid->parent._name << " has extrinsics " << fid->GetExtrinsicsPose() );
	}
}

void ArrayCalibrator::DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
{
	WriteLock lock( _buffMutex );
	ImageFiducialDetections dets( *msg );
	if( dets.detections.empty() ) { return; }

	BOOST_FOREACH( const FiducialDetection &det, dets.detections )
	{
		DetectionData data;
		data.sourceName = dets.sourceName;
		data.timestamp = dets.timestamp;
		data.detection = det;
		_detBuffer.push_back( data );
	}

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
	WriteLock lock( _buffMutex );
	unsigned int numProcessed = 1;
	DetectionsBuffer unprocessed( _detBuffer.size() );
	while( numProcessed != 0 )
	{
		numProcessed = 0;
		unprocessed.clear();
		BOOST_FOREACH( DetectionData & data, _detBuffer )
		{
			// If in future, save it for later
			if( data.timestamp > until )
			{
				unprocessed.push_back( data );
				continue;
			}

			// If too lagged, drop it
			if( data.timestamp < (until - _maxLag) ) { continue; }

			bool succ = ProcessDetection( data.sourceName,
			                              data.timestamp,
			                              data.detection );
			if( succ )
			{
				++numProcessed;
			}
			else
			{
				unprocessed.push_back( data );
			}
		}
		_detBuffer = unprocessed;
	}

	_buffTime = until;
}

// TODO Re-buffer detections that we dropped due to initializations to try again afterwards
bool ArrayCalibrator::ProcessDetection( const std::string& sourceName,
                                        const ros::Time& time,
                                        const FiducialDetection& det  )
{
	ROS_INFO_STREAM( "Processing observation of " << det.name << " from " << sourceName );
	if( _cameraRegistry.count( sourceName ) == 0 )
	{
		ROS_WARN_STREAM( "No optimization camera target " << sourceName );
		return false;
	}
	if( _fiducialRegistry.count( det.name ) == 0 )
	{
		ROS_WARN_STREAM( "No optimization fiducial target " << det.name );
		return false;
	}

	CameraRegistration::Ptr& camera = _cameraRegistry.at( sourceName );
	FiducialRegistration::Ptr& fiducial = _fiducialRegistry.at( det.name );

	if( !camera->IsIntrinsicsInitialized() )
	{
		ROS_ERROR_STREAM( "Uninitialized camera intrinsics " << camera->_name );
		return false;
	}
	if( !fiducial->IsIntrinsicsInitialized() )
	{
		ROS_ERROR_STREAM( "Uninitialized fiducial intrinsics " << fiducial->_name );
		return false;
	}

	bool camPoseGrounded = camera->parent.IsPoseInitialized( time );
	bool fidPoseGrounded = fiducial->parent.IsPoseInitialized( time );
	bool camExtInit = camera->IsExtrinsicsInitialized();
	bool fidExtInit = fiducial->IsExtrinsicsInitialized();

	unsigned int numInit = 0;
	if( camPoseGrounded ) { numInit++; }
	if( fidPoseGrounded ) { numInit++; }
	if( camExtInit ) { numInit++; }
	if( fidExtInit ) { numInit++; }

	// If one part of the pose loop is uninitialized
	if( numInit == 3 )
	{
		PoseSE3 relPose = EstimateArrayPose( det,
		                                     IsamToFiducial( fiducial->GetIntrinsicsNode()->value() ) );

		PoseSE3 camExt = IsamToPose( camera->GetExtrinsicsNode()->value() );
		PoseSE3 fidExt = IsamToPose( fiducial->GetExtrinsicsNode()->value() );

		// NOTE In the case of OdometryGraph we may get nullptr for these pose nodes
		// Case 1. If the graph is grounded, this means we don't have odometry available yet,
		//         so come back later
		// Case 2. If the graph is not grounded, this means the graph is uninitialized
		isam::PoseSE3_Node* camPoseNode = camera->parent.GetPoseNode( time );
		isam::PoseSE3_Node* fidPoseNode = fiducial->parent.GetPoseNode( time );
		if( camPoseGrounded && !camPoseNode ) { return false; }
		if( fidPoseGrounded && !fidPoseNode ) { return false; }
		PoseSE3 camPose = (camPoseNode) ? IsamToPose( camPoseNode->value() ) : PoseSE3();
		PoseSE3 fidPose = (fidPoseNode) ? IsamToPose( fidPoseNode->value() ) : PoseSE3();


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
			camera->InitializeExtrinsics( camExt );
		}
		else // !fidExtInit
		{
			fidExt = fidPose.Inverse() * camPose * camExt * relPose;
			ROS_INFO_STREAM( "Initializing extrinsics of " << fiducial->_name <<
			                 " to " << fidExt );
			fiducial->InitializeExtrinsics( fidExt );
		}
	}
	// If we fail above then there must be more than one uninitialized
	else if( numInit < 4 )
	{
		ROS_WARN_STREAM( "More than one part of pose chain missing: camPose: " <<
		                 camPoseGrounded << " fidPose: " << fidPoseGrounded << " camExt: " <<
		                 camExtInit << " fidExt: " << fidExtInit );
		return false;
	}

	// Otherwise everything is initialized
	isam::FiducialFactor::Ptr factor;
	unsigned int N = 2 * det.points.size();
	MatrixType cov = _detectionImgVariance * MatrixType::Identity( N, N );
	factor = create_fiducial_factor( time,
	                                 det,
	                                 cov,
	                                 *camera,
	                                 *fiducial );

	if( !factor )
	{
		ROS_WARN_STREAM( "Could not create factor!" );
		return false;
	}
	_observations.push_back( factor );
	_graph.AddFactor( factor );
	return true;
}
}
