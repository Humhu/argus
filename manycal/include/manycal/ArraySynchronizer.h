#pragma once

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "argus_utils/Semaphore.hpp"
#include "argus_utils/WorkerPool.hpp"

#include "manycal/CaptureArray.h"

#include <boost/thread/thread.hpp>

namespace manycal
{

/*! \brief Cycles and gathers outputs from the array into synchronized outputs.
 * Outputs show up as [camera_name]/image_synchronized. */
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
	
	struct CameraRegistration
	{
		std::string name;
		ros::ServiceClient captureClient;
		image_transport::CameraSubscriber imageSub;
		image_transport::CameraPublisher imagePub;
		BufferedData data;
	};
	
	ros::NodeHandle nodeHandle, privHandle;
	ros::ServiceServer captureServer;
	
	std::vector< CameraRegistration > cameraRegistry;
	
	image_transport::ImageTransport publicPort;
		
	argus_utils::Semaphore cameraTokens;
	argus_utils::Semaphore completedJobs;
	argus_utils::WorkerPool pool;
	
	void ImageCallback( const sensor_msgs::Image::ConstPtr& image,
						const sensor_msgs::CameraInfo::ConstPtr& info,
						CameraRegistration& registration );
	
	void CaptureJob( CameraRegistration& registration );
	
	bool CaptureArrayCallback( CaptureArray::Request& req,
							   CaptureArray::Response& res );
};
	
}
