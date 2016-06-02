#pragma once

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"

#include "argus_utils/synchronization/Semaphore.h"
#include "argus_utils/synchronization/WorkerPool.h"
#include "lookup/LookupInterface.h"

#include "manycal/CaptureArray.h"

#include <boost/thread/thread.hpp>
#include <unordered_map>

namespace argus
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
	
	LookupInterface lookupInterface;
	
	typedef std::unordered_map< std::string, CameraRegistration > CameraRegistry;
	CameraRegistry cameraRegistry;
	
	image_transport::ImageTransport publicPort;
	image_transport::ImageTransport privatePort;
		
	Semaphore cameraTokens;
	//Semaphore completedJobs;
	unsigned int receivedImages;
	ros::Time clampTime;
	WorkerPool pool;
	int numSimultaneous;
	
	void ImageCallback( const sensor_msgs::Image::ConstPtr& image,
	                    const sensor_msgs::CameraInfo::ConstPtr& info );
	
	void CaptureJob( CameraRegistration& registration );
	
	bool CaptureArrayCallback( manycal::CaptureArray::Request& req,
	                           manycal::CaptureArray::Response& res );
};
	
}
