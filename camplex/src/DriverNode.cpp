#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <boost/foreach.hpp>

#include <assert.h>

#include "camplex/DriverNode.h"
#include "camplex/CameraCalibration.h"

namespace camplex
{

	// TODO Dump the crap CameraInfoManager?
	DriverNode::DriverNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: nodeHandle( nh ), 
		privHandle ( ph ),
		it( privHandle ), 
		cameraInfoManager( std::make_shared<InfoManager>( privHandle ) ),
		frameCounter( 0 ),
		mode( STREAM_OFF )
	{
		
		captureFramesServer =
			privHandle.advertiseService( "capture_frames",
							&DriverNode::CaptureFramesService, this );
		getInfoServer =
			privHandle.advertiseService( "get_camera_info",
							&DriverNode::GetCameraInfoService, this );
		capabilitiesServer = 
			privHandle.advertiseService( "print_capabilities", 
							&DriverNode::PrintCapabilitiesService, this );
		setStreamingServer =
			privHandle.advertiseService( "set_streaming",
							&DriverNode::SetStreamingService, this );
		
		// Name uniquely IDs camera and validates calibration file
		privHandle.getParam( "camera_name", cameraName );
 		cameraInfoManager->setCameraName( cameraName );
		if( privHandle.hasParam( "camera_info_url" ) )
		{
			std::string calibFile;
			privHandle.getParam( "camera_info_url", calibFile );
			cameraInfoManager->loadCameraInfo( calibFile );
		}
		
		// NOTE Can't call getCameraInfo more than once on cameraInfoManager, or else segfault!!
		CameraCalibration calib( cameraName, cameraInfoManager->getCameraInfo() );
		
		// Set the driver parameters
		unsigned int frameWidth = 640;
		unsigned int frameHeight = 480;
		int frameRate;
		
		if( privHandle.hasParam( "frame_resolution" ) )
		{
			std::vector<int> frameResolution;
			privHandle.getParam( "frame_resolution", frameResolution );
			if( frameResolution.size() < 2 ) {
				ROS_ERROR_STREAM( "Frame resolution must be 2 integers." );
				exit( -1 );
			}
			if( frameResolution[0] < 0 || frameResolution[1] < 0 )
			{
				ROS_ERROR_STREAM( "Frame resolution must be positive." );
				exit( -1 );
			}
			frameWidth = (unsigned int) frameResolution[0];
			frameHeight = (unsigned int) frameResolution[1];
			cv::Size scale( frameWidth, frameHeight );
			calib.SetScale( scale );
		}
		cameraInfo = boost::make_shared<sensor_msgs::CameraInfo>( calib.GetInfo() );
		
		// Initialize the driver
		std::string devPath;
		if( !privHandle.getParam( "device_path", devPath ) )
		{
			ROS_WARN( "No device path specified. Defaulting to /dev/video0" );
			devPath = "/dev/video0";
		}
		driver.Open( devPath, CameraDriver::BLOCKING );
		if( !driver.IsOpen() )
		{
			ROS_ERROR( "Could not open device at %s", devPath.c_str() );
			return;
		}
		
		privHandle.param( "frame_rate", frameRate, 30 );
		
		OutputSpecification spec;
		spec.frameSize.first = frameWidth;
		spec.frameSize.second = frameHeight;
		// TODO Support fractional framerates
		spec.framePeriod.denominator = frameRate;
		spec.framePeriod.numerator = 1;
		// TODO Support other image formats
		spec.pixelFormat = FourCC( 'B', 'G', 'R', '3' );
		driver.SetOutputSpecification( spec );
		
		// Verify that we got the correct output spec
		OutputSpecification actualSpec = driver.ReadCurrentOutputSpecification();
		if( spec.frameSize.first != frameWidth ||
			spec.frameSize.second != frameHeight ||
			spec.framePeriod.denominator != frameRate )
		{
			ROS_WARN_STREAM( "Requested spec of: " << std::endl << spec << std::endl 
				<< " but received spec of: " << std::endl << actualSpec );
		}
		
		int numBuffers;
		privHandle.param( "num_buffers", numBuffers, 10 );
		driver.AllocateBuffers( numBuffers );
		
		// TODO Update to use YAML
		// TODO Search controls by name instead of ID?
		// Set any control registers
		if( privHandle.hasParam( "controls" ) )
		{
			XmlRpc::XmlRpcValue controls;
			privHandle.getParam( "controls", controls );
			
			XmlRpc::XmlRpcValue::iterator iter = controls.begin();
			while( iter != controls.end() )
			{
				XmlRpc::XmlRpcValue::ValueStruct::value_type item = *iter;
				iter++;
				int id = item.second["id"];
				int value = item.second["value"];
				driver.SetControl( id, value );
			}
		}
		
		int rosBufferSize;
		privHandle.param( "pub_buff_size", rosBufferSize, 1 );
		it_pub = it.advertiseCamera( "image_raw", rosBufferSize );
				
		// Set up publisher
		processWorker = boost::thread( boost::bind( &DriverNode::Process, this ) );

		bool streamOnStart;
		privHandle.param( "stream_on_start", streamOnStart, true );
		if( streamOnStart )
		{
			Lock lock( mutex );
			StartStreaming( lock );
		}
		
	}
	
	DriverNode::~DriverNode()
	{
		processWorker.interrupt();
		processWorker.join();
		Lock lock( mutex );
		StopStreaming( lock );
	}
	
	void DriverNode::StartStreaming( Lock& lock )
	{
		assert( lock.owns_lock( &mutex ) );
		
		if( mode == STREAM_CONTINUOUS ) { return; }
		// NOTE Ordering - start worker before driver starts buffering
		driver.SetStreaming( true );
		mode = STREAM_CONTINUOUS;
		blocked.notify_all(); // Notify worker to start
	}
	
	void DriverNode::StartCapture( Lock& lock, unsigned int num )
	{
		assert( lock.owns_lock( &mutex ) );
		
		// NOTE Ordering - start worker before driver starts buffering
		remainingToCapture = num;
		driver.SetStreaming( true );
		mode = STREAM_CAPTURE;
		blocked.notify_all(); // Notify worker to start
	}
	
	void DriverNode::StopStreaming( Lock& lock )
	{
		assert( lock.owns_lock( &mutex ) );

		if( mode == STREAM_OFF ) { return; }
		// NOTE Ordering - stop driver before worker stops
		driver.SetStreaming( false );
		mode = STREAM_OFF;
	}
	
	bool DriverNode::GetCameraInfoService( GetCameraInfo::Request& req,
										   GetCameraInfo::Response& res )
	{
		Lock lock( mutex );
		res.info = *cameraInfo;
		return true;
	}
	
	bool DriverNode::CaptureFramesService( CaptureFrames::Request& req,
										   CaptureFrames::Response& res )
	{
		Lock lock( mutex );
		
		StartCapture( lock, req.numToCapture );
		return true;
	}
	
	bool DriverNode::SetStreamingService( SetStreaming::Request& req,
										  SetStreaming::Response& res )
	{
		Lock lock( mutex );

		if( req.enableStreaming ) { StartStreaming( lock ); }
		else { StopStreaming( lock ); }
		return true;
	}
	
	// TODO Return capabilities in a string instead?
	bool DriverNode::PrintCapabilitiesService( PrintCapabilities::Request& req,
											   PrintCapabilities::Response& res )
	{
	  std::stringstream ss;
		try 
		{
			if( req.printCapabilities )
			{
				ss << driver.ReadCapabilities() << std::endl;
			}
			if( req.printOutputs )
			{
				std::vector<OutputSpecification> outputs = 
					driver.ReadOutputSpecifications();
				BOOST_FOREACH( OutputSpecification& spec, outputs )
				{
					ss << spec << std::endl;
				}
			}
			if( req.printControls )
			{
				std::vector<ControlSpecification> controls = driver.ReadControlSpecifications();
				BOOST_FOREACH( ControlSpecification& spec, controls )
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
	
	void DriverNode::Process()
	{
		cv::Mat frame;
		std_msgs::Header header;
		header.frame_id = cameraName;
		
		try
		{
			while( true )
			{
				boost::this_thread::interruption_point();
				
				Lock lock( mutex );
				while( mode == STREAM_OFF )
				{
					blocked.wait( lock );
				}
				lock.unlock();
				
				// Unlock here in case the call to GetFrame blocks
				frame = driver.GetFrame();
				if( frame.empty() )
				{
					ROS_WARN( "Received empty frame from device." );
				}

				lock.lock();
				
				// Populate message headers
				header.stamp = ros::Time::now(); // TODO Configure ROS time or wall time
				cameraInfo->header = header; // Want timestamps to match
				
				cv_bridge::CvImage img( header, "bgr8", frame );
				it_pub.publish( img.toImageMsg(), cameraInfo );
				
				// TODO Topic diagnostics
				//topic_diagnostics_.tick( header.stamp );
				
				if( mode == STREAM_CAPTURE )
				{
					remainingToCapture--;
					if( remainingToCapture == 0 )
					{
						StopStreaming( lock );
					}
				}
			}
		}
		catch( boost::thread_interrupted e ) {
			ROS_INFO_STREAM( "Camera worker thread interrupted. Stopping..." );
		}
		
	}
	
}
