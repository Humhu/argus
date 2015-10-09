#include "manycal/ArraySynchronizer.h"
#include "camplex/CaptureFrames.h"

#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace manycal
{
	
ArraySynchronizer::ArraySynchronizer( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), publicPort( nodeHandle ), privatePort( privHandle )
{
	std::vector< std::string > cameraNames;
	if( !privHandle.getParam( "camera_names", cameraNames ) )
	{
		ROS_ERROR_STREAM( "Please specify cameras to capture from." );
		exit( -1 );
	}
	
	BOOST_FOREACH( const std::string& cameraName, cameraNames )
	{
		image_transport::CameraSubscriber sub = publicPort.subscribeCamera( cameraName + "/image_raw", 1, 
			&ArraySynchronizer::ImageCallback, this );
		imageSubs.push_back( sub );
		
		std::string captureServiceName = cameraName + "/capture_frames";
		ros::service::waitForService( captureServiceName );
		ros::ServiceClient client = nodeHandle.serviceClient<camplex::CaptureFrames>( captureServiceName );
		captureClients.push_back( client );
	}
	
	
	captureServer = privHandle.advertiseService( "capture_array", &ArraySynchronizer::CaptureArrayCallback, this );
	imagePub = privatePort.advertiseCamera( "image_synchronized", 1 );
	
	int numSimultaneous;
	privHandle.param( "num_simultaneous_captures", numSimultaneous, 1 );
	cameraTokens.Increment( numSimultaneous - 1 );
	pool.SetNumWorkers( numSimultaneous );
	pool.StartWorkers();
}
	
bool ArraySynchronizer::CaptureArrayCallback( CaptureArray::Request& req,
										  CaptureArray::Response& res )
{
	
	// TODO Make sure buffer is cleared?
	for( unsigned int i = 0; i < captureClients.size(); i++ )
	{
		argus_utils::WorkerPool::Job job = boost::bind( &ArraySynchronizer::CaptureJob, this, i );
		pool.EnqueueJob( job );
		cameraTokens.Decrement();
	}

	ros::Time now = ros::Time::now();
	
	boost::unique_lock< boost::mutex > lock( mutex );
	BOOST_FOREACH( BufferedData& data, dataBuffer )
	{
		std_msgs::Header header;
		header.stamp = now;
		data.image->header = header;
		data.info->header = header;
		imagePub.publish( data.image, data.info );
	}
	
	return true;
}

void ArraySynchronizer::CaptureJob( unsigned int index )
{
	camplex::CaptureFrames srv;
	srv.request.numToCapture = 1;
	if( !captureClients[index].call( srv ) )
	{
		ROS_WARN_STREAM( "Could not capture from camera " << index );
	}
}

void ArraySynchronizer::ImageCallback( const sensor_msgs::Image::ConstPtr& image, 
									   const sensor_msgs::CameraInfo::ConstPtr& info )
{
	boost::unique_lock< boost::mutex > lock( mutex );
	
	BufferedData data;
	data.image = boost::make_shared<sensor_msgs::Image>( *image );
	data.info = boost::make_shared<sensor_msgs::CameraInfo>( *info );
	dataBuffer.push_back( data );
	
	cameraTokens.Increment();
}
	
} // end namespace manycal
