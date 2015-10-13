#include "manycal/ArraySynchronizer.h"
#include "camplex/CaptureFrames.h"

#include <boost/foreach.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace manycal
{
	
ArraySynchronizer::ArraySynchronizer( ros::NodeHandle& nh, ros::NodeHandle& ph )
: nodeHandle( nh ), privHandle( ph ), publicPort( nodeHandle )
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
		CameraRegistration registration;
		registration.name = cameraName;
		
		std::string captureServiceName = cameraName + "/capture_frames";
		ros::service::waitForService( captureServiceName );
		registration.captureClient = nodeHandle.serviceClient<camplex::CaptureFrames>( captureServiceName );
		registration.imagePub = publicPort.advertiseCamera( cameraName + "/image_synchronized", 1 );
		
		cameraRegistry.push_back( registration );
		
		// Have to push registration into registry before we can get the reference to it
		cameraRegistry.back().imageSub = publicPort.subscribeCamera( cameraName + "/image_raw", 1, 
		    boost::bind( &ArraySynchronizer::ImageCallback, this, _1, _2, boost::ref(cameraRegistry.back()) ) );
	}
	
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
	BOOST_FOREACH( CameraRegistration& registration, cameraRegistry )
	{
		argus_utils::WorkerPool::Job job = boost::bind( &ArraySynchronizer::CaptureJob, 
		                                                this, boost::ref(registration) );
		pool.EnqueueJob( job );
	}

	// TODO Handle timeouts
	completedJobs.Decrement( cameraRegistry.size() ); // Wait for all jobs to complete
	
	ros::Time now = ros::Time::now();
	
	std_msgs::Header header;
	header.stamp = now;
// 	static unsigned int sequence = 0;
	BOOST_FOREACH( CameraRegistration& registration, cameraRegistry )
	{
		if( !registration.data.image )
		{
			ROS_ERROR_STREAM( "Null buffered data!" );
		}
		header.frame_id = registration.data.image->header.frame_id;
		header.seq = registration.data.image->header.seq;
		registration.data.image->header = header;
		registration.data.info->header = header;
		registration.imagePub.publish( registration.data.image, registration.data.info );
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
                                       const sensor_msgs::CameraInfo::ConstPtr& info,
                                       CameraRegistration& registration )
{
	// Concurrent modification through ref is safe
// 	CameraRegistration& registration = cameraRegistry.at( index );
	registration.data.image = boost::make_shared<sensor_msgs::Image>( *image );
	registration.data.info = boost::make_shared<sensor_msgs::CameraInfo>( *info );
	
	cameraTokens.Increment();
	completedJobs.Increment();
}
	
} // end namespace manycal
