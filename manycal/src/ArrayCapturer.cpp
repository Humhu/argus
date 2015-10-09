#include "manycal/ArrayCapturer.h"
#include "camplex/CaptureFrames.h"

#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace manycal
{
	
ArrayCapturer::ArrayCapturer( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), imagePort( nodeHandle ), 
imageCounter( 0 ), cycleCounter( 0 )
{
	std::vector< std::string > cameraNames;
	if( !privHandle.getParam( "camera_names", cameraNames ) )
	{
		ROS_ERROR_STREAM( "Please specify cameras to capture from." );
		exit( -1 );
	}
	
	BOOST_FOREACH( const std::string& cameraName, cameraNames )
	{
		image_transport::Subscriber sub = imagePort.subscribe( cameraName + "/image_raw", 1, 
			boost::bind( &ArrayCapturer::ImageCallback, this, _1, cameraName ) );
		imageSubs.push_back( sub );
		
		std::string captureServiceName = cameraName + "/capture_frames";
		ros::service::waitForService( captureServiceName );
		ros::ServiceClient client = nodeHandle.serviceClient<camplex::CaptureFrames>( captureServiceName );
		captureClients.push_back( client );
	}
	
	// TODO Macro or util'ize this
	if( !privHandle.getParam( "output_directory", outputDirectory ) )
	{
		ROS_ERROR( "Please specify output directory." );
		exit( -1 );
	}
	if( outputDirectory.back() != '/' )
	{
		outputDirectory += '/';
	}
	
	std::string outputLogPath = outputDirectory + "image_log.txt";
	outputLog.open( outputLogPath );
	if( !outputLog.is_open() )
	{
		ROS_ERROR_STREAM( "Could not open output log at " << outputLogPath );
		exit( -1 );
	}
	outputLog << "[camera_name] [cycle_number] [image_path]" << std::endl;
	
	captureServer = privHandle.advertiseService( "capture_array", &ArrayCapturer::CaptureArrayCallback, this );
	
	int numSimultaneous;
	privHandle.param( "num_simultaneous_captures", numSimultaneous, 1 );
	cameraTokens.Increment( numSimultaneous - 1 );
	pool.SetNumWorkers( numSimultaneous );
	pool.StartWorkers();
}
	
bool ArrayCapturer::CaptureArrayCallback( CaptureArray::Request& req,
										  CaptureArray::Response& res )
{
	for( unsigned int i = 0; i < captureClients.size(); i++ )
	{
		argus_utils::WorkerPool::Job job = boost::bind( &ArrayCapturer::CaptureJob, this, i );
		pool.EnqueueJob( job );
		cameraTokens.Decrement();
	}
	boost::unique_lock< boost::mutex > lock( mutex );
	cycleCounter++;
	return true;
}

void ArrayCapturer::CaptureJob( unsigned int index )
{
	camplex::CaptureFrames srv;
	srv.request.numToCapture = 1;
	if( !captureClients[index].call( srv ) )
	{
		ROS_WARN_STREAM( "Could not capture from camera " << index );
	}
}

void ArrayCapturer::ImageCallback( const sensor_msgs::ImageConstPtr& msg, 
								   const std::string& cameraName )
{
	boost::unique_lock< boost::mutex > lock( mutex );
	
	std::stringstream ss;
	ss << outputDirectory << "image_" << imageCounter << ".png";
	cv::imwrite( ss.str(), cv_bridge::toCvShare(msg, "bgr8")->image );
	
	outputLog << cameraName << " " << cycleCounter << " " << ss.str() << std::endl;
	
	imageCounter++;
	
	cameraTokens.Increment();
}
	
} // end namespace manycal
