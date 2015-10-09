#pragma once

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "argus_utils/Semaphore.hpp"
#include "argus_utils/WorkerPool.hpp"

#include "manycal/CaptureArray.h"

#include <boost/thread/thread.hpp>

namespace manycal
{

/*! \brief Cycles and gathers outputs from the array into a synchronized output. */
class ArraySynchronizer
{
public:	
	
	ArraySynchronizer( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	void Cycle();
	
private:
	
	struct BufferedData
	{
		sensor_msgs::Image::Ptr image;
		sensor_msgs::CameraInfo::Ptr info;
	};
	
	ros::NodeHandle nodeHandle, privHandle;
	ros::ServiceServer captureServer;
	
	image_transport::ImageTransport publicPort, privatePort;
	std::vector< image_transport::CameraSubscriber > imageSubs;
	image_transport::CameraPublisher imagePub;
	
	std::vector< ros::ServiceClient > captureClients;
	
	argus_utils::Semaphore cameraTokens;
	argus_utils::WorkerPool pool;
	
	boost::mutex mutex;
	std::vector< BufferedData > dataBuffer;
	
	void ImageCallback( const sensor_msgs::Image::ConstPtr& image,
						const sensor_msgs::CameraInfo::ConstPtr& info );
	
	void CaptureJob( unsigned int index );
	
	bool CaptureArrayCallback( CaptureArray::Request& req,
							   CaptureArray::Response& res );
};
	
}
