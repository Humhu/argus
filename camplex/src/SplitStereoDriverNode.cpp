#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm/remove_if.hpp>

#include <assert.h>

#include "camplex/SplitStereoDriverNode.h"
#include "camplex/CameraCalibration.h"

namespace argus
{
SplitStereoDriverNode::SplitStereoDriverNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _leftNh( "~left" ),
	_rightNh( "~right" ),
	_leftIt( _leftNh ),
	_rightIt( _rightNh ),
	_leftInfoManager( _leftNh ),
	_rightInfoManager( _rightNh ),
	_mode( STREAM_OFF )
{
	// Initialize the _driver
	std::string devPath;
	GetParam<std::string>( ph, "device_path", devPath, "/dev/video0" );
	_driver.Open( devPath, CameraDriver::BLOCKING );
	if( !_driver.IsOpen() )
	{
		throw std::runtime_error( "Could not open device at: " + devPath );
	}

	// Name uniquely IDs camera and validates calibration file
	GetParamRequired( ph, "camera_name", _cameraName );
	std::string leftName = _cameraName + "_left";
	std::string rightName = _cameraName + "_right";
	_leftInfoManager.setCameraName( leftName );
	_leftInfoManager.setCameraName( rightName );

	// Read frame info to set scale
	unsigned int frameWidth, frameHeight;
	GetParam<unsigned int>( ph, "frame_width", frameWidth, 640 );
	GetParam<unsigned int>( ph, "frame_height", frameHeight, 480 );
	cv::Size frameSize( frameWidth/2, frameHeight );

	// Read camera calibrations
	std::string calibFile;
	if( GetParam( ph, "left_camera_info_url", calibFile ) )
	{
		_leftInfoManager.loadCameraInfo( calibFile );
	}
	if( GetParam( ph, "right_camera_info_url", calibFile ) )
	{
		_rightInfoManager.loadCameraInfo( calibFile );
	}

	// Cache camera info messages
	// NOTE Can't call getCameraInfo more than once on _cameraInfoManager, or else segfault for some reason!
	CameraCalibration calib( leftName, _leftInfoManager.getCameraInfo() );
	calib.SetScale( frameSize );
	_leftInfo = boost::make_shared<sensor_msgs::CameraInfo>( calib.GetInfo() );

	calib = CameraCalibration( rightName, _rightInfoManager.getCameraInfo() );
	calib.SetScale( frameSize );
	_rightInfo = boost::make_shared<sensor_msgs::CameraInfo>( calib.GetInfo() );

	// Read framerate and output spec
	unsigned int frameRate;
	GetParam<unsigned int>( ph, "frame_rate", frameRate, 30 );
	std::string fourCC;
	GetParam<std::string>( ph, "four_cc", fourCC, "YUYV" );

	OutputSpecification spec;
	spec.frameSize.first = frameWidth;
	spec.frameSize.second = frameHeight;
	// TODO Support fractional framerates
	spec.framePeriod.denominator = frameRate;
	spec.framePeriod.numerator = 1;
	spec.pixelFormat = FourCC( fourCC[0], fourCC[1], fourCC[2], fourCC[3] );
	_driver.SetOutputSpecification( spec );

	// Verify that we got the correct output spec
	OutputSpecification actualSpec = _driver.ReadCurrentOutputSpecification();
	if( spec.frameSize.first != frameWidth ||
	    spec.frameSize.second != frameHeight ||
	    spec.framePeriod.denominator != frameRate )
	{
		ROS_WARN_STREAM( "Requested spec of: " << std::endl << spec << std::endl
		                                       << " but received spec of: " << std::endl << actualSpec );
	}

	unsigned int numBuffers;
	GetParam<unsigned int>( ph, "num_buffers", numBuffers, 10 );
	_driver.AllocateBuffers( numBuffers );

	// Parse and set controls
	std::vector<ControlSpecification> controlSpecs;
	controlSpecs = _driver.ReadControlSpecifications();
	BOOST_FOREACH( const ControlSpecification &spec, controlSpecs )
	{
		if( spec.disabled || spec.readOnly ) { continue; }

		std::string origName = spec.name;
		std::string convName = origName;
		boost::to_lower( convName );
		boost::replace_all( convName, " ", "_" );
		convName.erase( boost::remove_if( convName, boost::is_any_of( ",()[]" ) ), convName.end() );

		std::string type = ControlSpecification::TypeToString( spec.type );
		if( type == "integer" || type == "menu" )
		{
			_numericParams.emplace_back();
			_numericParams.back().Initialize( ph,
			                                  (double) spec.defaultVal,
			                                  convName,
			                                  origName );
			_numericParams.back().AddCheck<IntegerValued>( ROUND_CLOSEST );
			_numericParams.back().AddCheck<GreaterThanOrEqual>( spec.minVal );
			_numericParams.back().AddCheck<LessThanOrEqual>( spec.maxVal );
			NumericParam::Callback cb = boost::bind( &SplitStereoDriverNode::IntControlCallback,
			                                         this,
			                                         spec.id,
			                                         _1 );
			_numericParams.back().AddCallback( cb );
		}
		else if( type == "boolean" )
		{
			_booleanParams.emplace_back();
			_booleanParams.back().Initialize( ph,
			                                  (bool) spec.defaultVal,
			                                  convName,
			                                  origName );
			BooleanParam::Callback cb = boost::bind( &SplitStereoDriverNode::BoolControlCallback,
			                                         this,
			                                         spec.id,
			                                         _1 );
		}
	}

	// TODO Search controls by name instead of ID?
	// Set any control registers
	YAML::Node controls;
	if( GetParam( ph, "controls", controls ) )
	{
		YAML::Node::const_iterator iter;
		for( iter = controls.begin(); iter != controls.end(); ++iter )
		{
			const std::string& name = iter->first.as<std::string>();
			const YAML::Node& info = iter->second;
			int id, value;
			GetParamRequired( info, "id", id );
			GetParamRequired( info, "value", value );
			_driver.SetControl( id, value );
		}
	}

	unsigned int pubBuffSize;
	GetParam<unsigned int>( ph, "buff_size", pubBuffSize, 1 );
	_leftPub = _leftIt.advertiseCamera( "image_raw", pubBuffSize );
	_rightPub = _rightIt.advertiseCamera( "image_raw", pubBuffSize );

	bool streamOnStart;
	GetParam( ph, "stream_on_start", streamOnStart, true );
	if( streamOnStart )
	{
		WriteLock lock( _mutex );
		StartStreaming( lock );
	}

	unsigned int numThreads;
	GetParam<unsigned int>( ph, "num_threads", numThreads, 1 );
	_workers.SetNumWorkers( numThreads );
	WorkerPool::Job job = boost::bind( &SplitStereoDriverNode::Spin, this );
	for( unsigned int i = 0; i < numThreads; ++i )
	{
		_workers.EnqueueJob( job );
	}
	_workers.StartWorkers();

	_leftInfoServer = _leftNh.advertiseService( "get_camera_info",
	                                            &SplitStereoDriverNode::GetLeftInfoService,
	                                            this );
	_rightInfoServer = _rightNh.advertiseService( "get_camera_info",
	                                              &SplitStereoDriverNode::GetRightInfoService,
	                                              this );
	_capabilitiesServer = ph.advertiseService( "print_capabilities",
	                                           &SplitStereoDriverNode::PrintCapabilitiesService,
	                                           this );
	_setStreamingServer = ph.advertiseService( "set_streaming",
	                                           &SplitStereoDriverNode::SetStreamingService,
	                                           this );
}

SplitStereoDriverNode::~SplitStereoDriverNode()
{
	_blocked.notify_all();
}

void SplitStereoDriverNode::IntControlCallback( int id, double value )
{
	int valInt = (int) std::round( value );
	WriteLock lock( _mutex );
	_driver.SetControl( id, valInt );
}

void SplitStereoDriverNode::BoolControlCallback( int id, bool value )
{
	int valInt = value ? 1 : 0;
	WriteLock lock( _mutex );
	_driver.SetControl( id, valInt );
}

void SplitStereoDriverNode::StartStreaming( WriteLock& lock )
{
	assert( lock.owns_lock( &_mutex ) );

	if( _mode == STREAM_CONTINUOUS ) { return; }
	// NOTE Ordering - start worker before _driver starts buffering
	_driver.SetStreaming( true );
	_mode = STREAM_CONTINUOUS;
	_blocked.notify_all(); // Notify worker to start
}

void SplitStereoDriverNode::StopStreaming( WriteLock& lock )
{
	assert( lock.owns_lock( &_mutex ) );

	if( _mode == STREAM_OFF ) { return; }
	// NOTE Ordering - stop _driver before worker stops
	_driver.SetStreaming( false );
	_mode = STREAM_OFF;
}

bool SplitStereoDriverNode::GetLeftInfoService( camplex::GetCameraInfo::Request& req,
                                                camplex::GetCameraInfo::Response& res )
{
	ReadLock lock( _mutex );
	res.info = *_leftInfo;
	return true;
}

bool SplitStereoDriverNode::GetRightInfoService( camplex::GetCameraInfo::Request& req,
                                                 camplex::GetCameraInfo::Response& res )
{
	ReadLock lock( _mutex );
	res.info = *_rightInfo;
	return true;
}

bool SplitStereoDriverNode::SetStreamingService( camplex::SetStreaming::Request& req,
                                                 camplex::SetStreaming::Response& res )
{
	WriteLock lock( _mutex );

	if( req.enableStreaming ) { StartStreaming( lock ); }
	else { StopStreaming( lock ); }
	return true;
}

// TODO Return capabilities in a string instead?
bool SplitStereoDriverNode::PrintCapabilitiesService( camplex::PrintCapabilities::Request& req,
                                                      camplex::PrintCapabilities::Response& res )
{
	std::stringstream ss;
	try
	{
		if( req.printCapabilities )
		{
			ss << _driver.ReadCapabilities() << std::endl;
		}
		if( req.printOutputs )
		{
			std::vector<OutputSpecification> outputs =
			    _driver.ReadOutputSpecifications();
			BOOST_FOREACH( OutputSpecification & spec, outputs )
			{
				ss << spec << std::endl;
			}
		}
		if( req.printControls )
		{
			std::vector<ControlSpecification> controls = _driver.ReadControlSpecifications();
			BOOST_FOREACH( ControlSpecification & spec, controls )
			{
				ss << spec << std::endl;
			}
		}
	}
	catch( std::runtime_error& e )
	{
		ROS_ERROR_STREAM( "Error while reading camera capabilities." );
		return false;
	}
	res.capabilities = ss.str();
	return true;
}

void SplitStereoDriverNode::Spin()
{
	cv::Mat frame, frameLeft, frameRight;
	std_msgs::Header leftHeader, rightHeader;

	while( !ros::isShuttingDown() )
	{
		WriteLock lock( _mutex );
		while( _mode == STREAM_OFF && !ros::isShuttingDown() )
		{
			_blocked.wait( lock );
		}
		if( ros::isShuttingDown() ) { return; }

		frame = _driver.GetFrame();
		if( frame.empty() )
		{
			ROS_WARN( "Received empty frame from device." );
		}
		lock.unlock();

		// Populate message headers
		ros::Time now = ros::Time::now(); // TODO Configure ROS time or wall time
		_leftInfo->header.stamp = now;
		_rightInfo->header.stamp = now;

		unsigned int w = frame.size().width;
		unsigned int h = frame.size().height;

        cv::Rect leftRoi( 0, 0, w / 2, h );
        cv::Rect rightRoi( w / 2, 0, w / 2, h );
		frameLeft = frame( leftRoi );
		frameRight = frame( rightRoi );

		cv_bridge::CvImage imgLeft( _leftInfo->header, "bgr8", frameLeft );
		cv_bridge::CvImage imgRight( _rightInfo->header, "bgr8", frameRight );
		_leftPub.publish( imgLeft.toImageMsg(), _leftInfo );
		_rightPub.publish( imgRight.toImageMsg(), _rightInfo );
	}
}
}
