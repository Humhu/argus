#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/range/algorithm/remove_if.hpp>

#include <assert.h>

#include "camplex/DriverNode.h"
#include "camplex/CameraCalibration.h"

namespace argus
{

// TODO Dump the crap CameraInfoManager?
DriverNode::DriverNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _it( ph ),
	_cameraInfoManager( std::make_shared<InfoManager>( ph ) ),
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
	_cameraInfoManager->setCameraName( _cameraName );
        GetParam( ph, "camera_frame", _cameraFrame, _cameraName );

	std::string calibFile;
	if( GetParam( ph, "camera_info_url", calibFile ) )
	{
		_cameraInfoManager->loadCameraInfo( calibFile );
	}

	// NOTE Can't call getCameraInfo more than once on _cameraInfoManager, or else segfault!
	CameraCalibration calib( _cameraName, _cameraInfoManager->getCameraInfo() );


	// Set the _driver parameters
	unsigned int frameWidth, frameHeight;
	GetParam<unsigned int>( ph, "frame_width", frameWidth, 640 );
	GetParam<unsigned int>( ph, "frame_height", frameHeight, 480 );

	cv::Size scale( frameWidth, frameHeight );
	calib.SetScale( scale );
	_cameraInfo = boost::make_shared<sensor_msgs::CameraInfo>( calib.GetInfo() );

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

	std::vector<ControlSpecification> controlSpecs;
	controlSpecs = _driver.ReadControlSpecifications();
	BOOST_FOREACH( const ControlSpecification &spec, controlSpecs )
	{
		if( spec.disabled || spec.readOnly ) { continue; }

		std::string origName = spec.name;
		std::string convName = origName;
		boost::to_lower( convName );
		boost::replace_all( convName, " ", "_" );
		convName.erase( boost::remove_if( convName, boost::is_any_of(",()[]") ), convName.end() );

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
			NumericParam::Callback cb = boost::bind( &DriverNode::IntControlCallback,
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
			BooleanParam::Callback cb = boost::bind( &DriverNode::BoolControlCallback,
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
	_itPub = _it.advertiseCamera( "image_raw", pubBuffSize );

	bool streamOnStart;
	GetParam( ph, "stream_on_start", streamOnStart, true );
	if( streamOnStart )
	{
		WriteLock lock( _mutex );
		StartStreaming( lock );
	}

	unsigned int numThreads;
	GetParam<unsigned int>(ph, "num_threads", numThreads, 1);
	_workers.SetNumWorkers( numThreads );
	WorkerPool::Job job = boost::bind( &DriverNode::Spin, this );
	for( unsigned int i = 0; i < numThreads; ++i )
	{
		_workers.EnqueueJob( job );
	}
	_workers.StartWorkers();

	_getInfoServer = ph.advertiseService( "get_camera_info",
	                                      &DriverNode::GetCameraInfoService,
	                                      this );
	_capabilitiesServer = ph.advertiseService( "print_capabilities",
	                                           &DriverNode::PrintCapabilitiesService,
	                                           this );
	_setStreamingServer = ph.advertiseService( "set_streaming",
	                                           &DriverNode::SetStreamingService,
	                                           this );
}

DriverNode::~DriverNode()
{
	_blocked.notify_all();
}

void DriverNode::IntControlCallback( int id, double value )
{
	int valInt = (int) std::round( value );
	WriteLock lock( _mutex );
	_driver.SetControl( id, valInt );
}

void DriverNode::BoolControlCallback( int id, bool value )
{
	int valInt = value ? 1 : 0;
	WriteLock lock( _mutex );
	_driver.SetControl( id, valInt );
}

void DriverNode::StartStreaming( WriteLock& lock )
{
	assert( lock.owns_lock( &_mutex ) );

	if( _mode == STREAM_CONTINUOUS ) { return; }
	// NOTE Ordering - start worker before _driver starts buffering
	_driver.SetStreaming( true );
	_mode = STREAM_CONTINUOUS;
	_blocked.notify_all(); // Notify worker to start
}

void DriverNode::StopStreaming( WriteLock& lock )
{
	assert( lock.owns_lock( &_mutex ) );

	if( _mode == STREAM_OFF ) { return; }
	// NOTE Ordering - stop _driver before worker stops
	_driver.SetStreaming( false );
	_mode = STREAM_OFF;
}

bool DriverNode::GetCameraInfoService( camplex::GetCameraInfo::Request& req,
                                       camplex::GetCameraInfo::Response& res )
{
	ReadLock lock( _mutex );
	res.info = *_cameraInfo;
	return true;
}

bool DriverNode::SetStreamingService( camplex::SetStreaming::Request& req,
                                      camplex::SetStreaming::Response& res )
{
	WriteLock lock( _mutex );

	if( req.enableStreaming ) { StartStreaming( lock ); }
	else { StopStreaming( lock ); }
	return true;
}

// TODO Return capabilities in a string instead?
bool DriverNode::PrintCapabilitiesService( camplex::PrintCapabilities::Request& req,
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

void DriverNode::Spin()
{
	cv::Mat frame;
	std_msgs::Header header;
	header.frame_id = _cameraFrame;

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
		header.stamp = ros::Time::now(); // TODO Configure ROS time or wall time
		_cameraInfo->header = header; // Want timestamps to match

		cv_bridge::CvImage img( header, "bgr8", frame );
		_itPub.publish( img.toImageMsg(), _cameraInfo );
	}
}

}
