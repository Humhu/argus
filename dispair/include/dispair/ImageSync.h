#pragma once

// Class for synchronizing image inputs
// Currently we only support image/disparity pairs

#include <sensor_msgs/Image.h>
#include <stereo_msgs/DisparityImage.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/subscriber_filter.h>

#include <memory>

namespace argus
{

// TODO StereoImageSync for image pairs

// NOTE This is based off of stereo_processor.h from viso2_ros
// This class should be derived and the callback implemented
class DisparityImageSync
{
public:

	// Construct the synchronizer
	// nh specifies the subscription namespace
	// ph provides parameters for queueing, sync types
	DisparityImageSync( ros::NodeHandle& nh, ros::NodeHandle& ph );

private:

	image_transport::ImageTransport _imageTransport;
	image_transport::SubscriberFilter _imageSub;
	message_filters::Subscriber<stereo_msgs::DisparityImage> _disparitySub;
	message_filters::Subscriber<sensor_msgs::CameraInfo> _infoSub;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
	                                                  stereo_msgs::DisparityImage,
	                                                  sensor_msgs::CameraInfo>
	ExactPolicy;
	typedef message_filters::Synchronizer<ExactPolicy> ExactSync;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
	                                                        stereo_msgs::DisparityImage,
	                                                        sensor_msgs::CameraInfo>
	ApproximatePolicy;
	typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;

	std::shared_ptr<ExactSync> _exactSync;
	std::shared_ptr<ApproximateSync> _approxSync;

	// Method to be implemented by derived class
	virtual void DataCallback( const sensor_msgs::ImageConstPtr& image,
	                           const stereo_msgs::DisparityImageConstPtr& disparity,
	                           const sensor_msgs::CameraInfoConstPtr& info ) = 0;
};
}