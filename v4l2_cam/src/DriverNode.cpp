#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

#include <boost/foreach.hpp>

#include "v4l2_cam/DriverNode.h"

namespace v4l2_cam
{

	DriverNode::DriverNode( ros::NodeHandle& nh, ros::NodeHandle& ph,
		const std::string& pubName )
		: ResourceUser( nh, ph ),
		nodeHandle( nh ), 
		privHandle ( ph ), 
		capabilitiesServer( privHandle.advertiseService( 
							"print_capabilities", 
							&DriverNode::PrintCapabilitiesService, 
							this ) ),
		setStreamingServer( privHandle.advertiseService(
							"set_streaming",
							&DriverNode::SetStreamingService,
							this ) ),
		it( privHandle ), 
		cameraInfoManager( std::make_shared<InfoManager>( privHandle ) ),
		frameCounter( 0 ),
		streaming( false ),
		publishName( pubName )
	{

		// Name uniquely IDs camera and validates calibration file
		privHandle.getParam( "camera_name", cameraName );
 		cameraInfoManager->setCameraName( cameraName );
		if( privHandle.hasParam( "camera_info_url" ) )
		{
			std::string calibFile;
			privHandle.getParam( "camera_info_url", calibFile );
			cameraInfoManager->loadCameraInfo( calibFile );
		}
		cameraInfo = boost::make_shared<sensor_msgs::CameraInfo>( cameraInfoManager->getCameraInfo() );
		
		// Initialize the driver
		std::string devPath;
		if( !privHandle.getParam( "device_path", devPath ) )
		{
			ROS_WARN( "No device path specified. Defaulting to /dev/video0" );
			devPath = "/dev/video0";
		}
		driver.Open( devPath, v4l2_cam::CameraDriver::BLOCKING );
		if( !driver.IsOpen() )
		{
			ROS_ERROR( "Could not open device at %s", devPath.c_str() );
			return;
		}
		
		// Set the driver parameters (limited for now)
		int frameWidth = 640;
		int frameHeight = 480;
		int frameRate;
		
		if( privHandle.hasParam( "frame_resolution" ) )
		{
			std::vector<int> frameResolution;
			privHandle.getParam( "frame_resolution", frameResolution );
			if( frameResolution.size() < 2 ) {
				throw std::runtime_error( "Frame resolution must be 2 integers." );
			}
			frameWidth = frameResolution[0];
			frameHeight = frameResolution[1];
		}
		privHandle.param( "frame_rate", frameRate, 30 );
		
		v4l2_cam::OutputSpecification spec;
		spec.frameSize.first = frameWidth;
		spec.frameSize.second = frameHeight;
		// TODO Support fractional framerates
		spec.framePeriod.denominator = frameRate;
		spec.framePeriod.numerator = 1;
		// TODO Support other image formats
		spec.pixelFormat = v4l2_cam::FourCC( 'B', 'G', 'R', '3' );
		driver.SetOutputSpecification( spec );
		
		// Verify that we got the correct output spec
		v4l2_cam::OutputSpecification actualSpec = driver.ReadCurrentOutputSpecification();
		if( spec.frameSize.first != frameWidth ||
			spec.frameSize.second != frameHeight ||
			spec.framePeriod.denominator != frameRate )
		{
			ROS_WARN_STREAM( "Requested spec of: " << std::endl << spec << std::endl 
				<< " but received spec of: " << std::endl << actualSpec );
		}
		
		// Open the driver and begin running
		int numBuffers;
		privHandle.param( "num_buffers", numBuffers, 10 );
		driver.AllocateBuffers( numBuffers );
		
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
		
		statusPublisher = privHandle.advertise<CameraStatus>( "camera_status", 10 );
		
		// Set up publisher
		processWorker = boost::thread( boost::bind( &DriverNode::Process, this ) );

		bool streamOnStart;
		privHandle.param( "stream_on_start", streamOnStart, false );
		if( streamOnStart )
		{
			StartStreaming();
		}
		
	}
	
	DriverNode::~DriverNode()
	{
		StopStreaming();
		processWorker.interrupt();
	}
	
	void DriverNode::StartStreaming()
	{
		if( streaming.load( boost::memory_order_relaxed ) ) { return; }
		
		if( !AcquireResources() ) { 
			ROS_WARN_STREAM( "Could not acquire resources required for camera stream." );
			return;
		}
		
		streaming.store( true, boost::memory_order_relaxed );
		unsigned int numFrames = frameCounter; // TODO Unsychronized read!
		driver.SetStreaming( true );
		blocked.notify_all();
		
		CameraStatus msg;
		msg.header.frame_id = cameraName;
		msg.header.stamp = ros::Time::now();
		msg.streaming = true;
		msg.framesStreamed = numFrames;
		statusPublisher.publish( msg );
	}
	
	void DriverNode::StopStreaming()
	{
		streaming.store( false, boost::memory_order_relaxed );
		driver.SetStreaming( false );
		RelinquishResources();
		
		CameraStatus msg;
		msg.header.frame_id = cameraName;
		msg.header.stamp = ros::Time::now();
		msg.streaming = false;
		msg.framesStreamed = frameCounter; // TODO Unsynchronized read!
		statusPublisher.publish( msg );
	}
	
	bool DriverNode::SetStreamingService( v4l2_cam::SetStreaming::Request& req,
										  v4l2_cam::SetStreaming::Response& res )
	{
		Lock lock( mutex );

		remainingToStream = req.numFramesToStream;

		if( req.enableStreaming )
		{
			StartStreaming();
		}
		else
		{
			StopStreaming();
		}
		
		
		return true;
	}
	
	bool DriverNode::PrintCapabilitiesService( v4l2_cam::PrintCapabilities::Request& req,
											   v4l2_cam::PrintCapabilities::Response& res )
	{
		try 
		{
			if( req.printCapabilities )
			{
				std::cout << driver.ReadCapabilities() << std::endl;
			}
			if( req.printOutputs )
			{
				std::vector<v4l2_cam::OutputSpecification> outputs = 
					driver.ReadOutputSpecifications();
				BOOST_FOREACH( v4l2_cam::OutputSpecification& spec, outputs )
				{
					std::cout << spec << std::endl;
				}
			}
			if( req.printControls )
			{
				std::vector<v4l2_cam::ControlSpecification> controls = driver.ReadControlSpecifications();
				BOOST_FOREACH( v4l2_cam::ControlSpecification& spec, controls )
				{
					std::cout << spec << std::endl;
				}
			}
		}
		catch( std::runtime_error& e )
		{
			return false;
		}
		return true;
	}
	
	void DriverNode::Process()
	{
		cv::Mat frame;
		std_msgs::Header header;
		header.frame_id = publishName;
		
		while( true )
		{
			Lock lock( mutex );
			while( !streaming.load( boost::memory_order_relaxed ) )
			{
				blocked.wait( lock );
			}
			
			frame = driver.GetFrame();
			if( frame.empty() )
			{
				ROS_WARN( "Received empty frame from device." );
			}

			// Populate message headers
			header.stamp = ros::Time::now(); // TODO Configure ROS time or wall time
			header.seq = frameCounter++;
			
			// TODO Verify cameraInfoManager matches current device settings
			cameraInfo->header = header;
				
			cv_bridge::CvImage img( header, "bgr8", frame );
			
			it_pub.publish( img.toImageMsg(), cameraInfo );
			
			// TODO Topic diagnostics
			//topic_diagnostics_.tick( header.stamp );
			
			// remainingToStream equaling 0 here means stream continuously
			if( remainingToStream > 0 )
			{
				remainingToStream--;
				if( remainingToStream == 0 )
				{
					StopStreaming();
				}
			}
		}
	}
	
}
