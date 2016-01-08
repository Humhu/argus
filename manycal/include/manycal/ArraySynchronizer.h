#pragma once

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "argus_utils/Semaphore.h"
#include "argus_utils/WorkerPool.h"
#include "lookup/LookupInterface.h"

#include "manycal/CaptureArray.h"

#include <boost/thread/thread.hpp>
#include <unordered_map>

namespace manycal
{

/*! \brief Cycles and gathers outputs from the array into synchronized outputs.
 * Outputs show up as image_synchronized. */
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
		BufferedData data;
	};
	
	image_transport::CameraPublisher imagePub;
	image_transport::CameraSubscriber imageSub;
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	ros::ServiceServer captureServer;
	
	lookup::LookupInterface lookupInterface;
	
	typedef std::unordered_map< std::string, CameraRegistration > CameraRegistry;
	CameraRegistry cameraRegistry;
	
	image_transport::ImageTransport publicPort;
	image_transport::ImageTransport privatePort;
		
	argus_utils::Semaphore cameraTokens;
	argus_utils::Semaphore completedJobs;
	argus_utils::WorkerPool pool;
	
	void ImageCallback( const sensor_msgs::Image::ConstPtr& image,
						const sensor_msgs::CameraInfo::ConstPtr& info );
	
	void CaptureJob( CameraRegistration& registration );
	
	bool CaptureArrayCallback( CaptureArray::Request& req,
							   CaptureArray::Response& res );
};
	
}
