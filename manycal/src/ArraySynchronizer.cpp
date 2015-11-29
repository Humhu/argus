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
	
	cameraRegistry.reserve( cameraNames.size() );
	BOOST_FOREACH( const std::string& cameraName, cameraNames )
	{
		if( cameraRegistry.count( cameraName ) > 0 )
		{
			ROS_ERROR_STREAM( "Duplicate camera: " << cameraName );
			exit( -1 );
		}
		
		CameraRegistration registration;
		registration.name = cameraName;
		
		std::string captureServiceName = cameraName + "/capture_frames";
		ros::service::waitForService( captureServiceName );
		registration.captureClient = nodeHandle.serviceClient<camplex::CaptureFrames>( captureServiceName );
		
		cameraRegistry[ cameraName ] = ( registration );
	}
	
	imagePub = privatePort.advertiseCamera( "image_output", 2*cameraNames.size() );
	imageSub = publicPort.subscribeCamera( "image_input", 2*cameraNames.size(), &ArraySynchronizer::ImageCallback, this );
		
	captureServer = privHandle.advertiseService( "capture_array", &ArraySynchronizer::CaptureArrayCallback, this );
	
	int numSimultaneous;
	privHandle.param( "num_simultaneous_captures", numSimultaneous, 1 );
	cameraTokens.Increment( numSimultaneous );
	pool.SetNumWorkers( numSimultaneous );
	pool.StartWorkers();
}

bool ArraySynchronizer::CaptureArrayCallback( CaptureArray::Request& req, 
                                              CaptureArray::Response& res )
{
	
	// TODO Make sure buffer is cleared?
	BOOST_FOREACH( CameraRegistry::value_type& item, cameraRegistry )
	{
		argus_utils::WorkerPool::Job job = boost::bind( &ArraySynchronizer::CaptureJob, 
		                                                this, boost::ref( item.second ) );
		pool.EnqueueJob( job );
	}

	// TODO Handle timeouts
	ROS_INFO( "Dispatched all jobs. Waiting on semaphore." );
	completedJobs.Decrement( cameraRegistry.size() ); // Wait for all jobs to complete
	
	ros::Time now = ros::Time::now();
	
	BOOST_FOREACH( CameraRegistry::value_type& item, cameraRegistry )
	{
		std_msgs::Header header;
		header.stamp = now;
		
		CameraRegistration& registration = item.second;
		if( !registration.data.image )
		{
			ROS_ERROR_STREAM( "Null buffered data!" );
		}
		
		header.frame_id = registration.data.image->header.frame_id;
		header.seq = registration.data.image->header.seq;
		registration.data.image->header = header;
		registration.data.info->header = header;
		imagePub.publish( registration.data.image, registration.data.info );
	}
	
	return true;
}

void ArraySynchronizer::CaptureJob( CameraRegistration& registration )
{
	cameraTokens.Decrement(); // Manage number of active cameras
	camplex::CaptureFrames srv;
	srv.request.numToCapture = 1;
	if( !registration.captureClient.call( srv ) )
	{
		ROS_WARN_STREAM( "Could not capture from camera " << registration.name );
	}
}

void ArraySynchronizer::ImageCallback( const sensor_msgs::Image::ConstPtr& image, 
                                       const sensor_msgs::CameraInfo::ConstPtr& info )
{
	// Concurrent modification through ref is safe
	if( cameraRegistry.count( image->header.frame_id ) == 0 )
	{
		ROS_WARN_STREAM( "Received image from unregistered camera " << image->header.frame_id );
		return;
		
	}
 	CameraRegistration& registration = cameraRegistry[ image->header.frame_id ];
	ROS_INFO_STREAM( "Image received from " << image->header.frame_id );
	registration.data.image = boost::make_shared<sensor_msgs::Image>( *image );
	registration.data.info = boost::make_shared<sensor_msgs::CameraInfo>( *info );
	
	cameraTokens.Increment();
	completedJobs.Increment();
}
	
} // end namespace manycal
