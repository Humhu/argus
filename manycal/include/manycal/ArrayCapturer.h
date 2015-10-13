#pragma once

#include "manycal/CaptureArray.h"
#include "argus_utils/Semaphore.hpp"
#include "argus_utils/WorkerPool.hpp"

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <boost/thread/thread.hpp>

#include <fstream>
#include <iostream>

namespace manycal
{
	
/*! \brief Subscribes to an image_synchronized topic and saves images. */
class ArrayCapturer
{
public:	
	
	ArrayCapturer( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
private:
	
	ros::NodeHandle nodeHandle, privHandle;
	ros::ServiceServer captureServer;
	
	image_transport::ImageTransport imagePort;
	std::vector< image_transport::Subscriber > imageSubs;
	std::vector< ros::ServiceClient > captureClients;
	
	std::string outputDirectory;
	std::ofstream outputLog;
	
	boost::mutex mutex;
	argus_utils::Semaphore cameraTokens;
	argus_utils::WorkerPool pool;
	unsigned int imageCounter;
	unsigned int cycleCounter;
	
	void ImageCallback( const sensor_msgs::Image::ConstPtr& msg,
						const std::string& cameraName );
	
	void CaptureJob( unsigned int index );
	
	bool CaptureArrayCallback( CaptureArray::Request& req,
							   CaptureArray::Response& res );
	
};
	
} // end namespace manycal
