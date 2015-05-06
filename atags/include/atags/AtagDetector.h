#ifndef _ATAGS_ATAG_DETECTOR_H_
#define _ATAGS_ATAG_DETECTOR_H_

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>

#include "apriltags/TagDetector.h"

#include <memory>

namespace atags 
{
	
	class AtagDetector
	{
	public:
		
		typedef std::shared_ptr<AtagDetector> Ptr;
		
		AtagDetector( const ros::NodeHandle& nh, const ros::NodeHandle& ph );
		
		void ImageCallback( const sensor_msgs::ImageConstPtr& msg,
							const sensor_msgs::CameraInfoConstPtr& info_msg );
		
	private:
		
		ros::NodeHandle nodeHandle;
		ros::NodeHandle privHandle;
				
		ros::Publisher detectionPublisher;

		image_transport::ImageTransport imagePort;
		image_transport::CameraSubscriber cameraSub;
		image_geometry::PinholeCameraModel cameraModel;
		
		AprilTags::TagDetector::Ptr detector;
		
	};
	
}

#endif
