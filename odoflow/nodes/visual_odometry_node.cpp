#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/TwistStamped.h>
#include <image_geometry/pinhole_camera_model.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "odoflow/VisualOdometryPipeline.h"

using namespace argus_utils;
using namespace odoflow;

// TODO Add image pyramids
class VisualOdometryNode
{
	// ROS Node members
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	
	ros::Publisher posePub;
	
	image_transport::ImageTransport rImagePort;
	image_transport::CameraSubscriber rImageSub;
	
	image_geometry::PinholeCameraModel cameraModel;
	
	ros::Time lastTime;
	VisualOdometryPipeline pipeline;
	
	PoseSE3 pose;
	
public:
	
	VisualOdometryNode()
	: privHandle( "~" ),
	rImagePort( nodeHandle ),
	pipeline( nodeHandle, privHandle )
	{
		
		// Subscribe to image stream from some input source
		std::string topicName = nodeHandle.resolveName( "image_raw" );
		rImageSub = rImagePort.subscribeCamera( topicName, 1,
			&VisualOdometryNode::ImageCallback, this );
		
		posePub = privHandle.advertise<geometry_msgs::TwistStamped>( "odometry", 10 );
		
		pose = PoseSE3(); // Initialization (technically unnecessary)
		
	}
	
	~VisualOdometryNode()
	{}
	
	void ImageCallback( const sensor_msgs::ImageConstPtr& msg,
						const sensor_msgs::CameraInfoConstPtr& info_msg )
	{
		
		cameraModel.fromCameraInfo( info_msg );
		pipeline.SetRectificationParameters( cameraModel.intrinsicMatrix(),
											 cameraModel.distortionCoeffs() );
		
		ros::Time timestamp = msg->header.stamp;
		
		// Decode the received image into an OpenCV object
		cv_bridge::CvImageConstPtr frame;
		try
		{
			frame = cv_bridge::toCvShare( msg, "mono8" );
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR( "VisualOdometryNode cv_bridge exception: %s", e.what() );
			return;
		}
		
		cv::Mat image = frame->image;
		
		PoseSE3 displacement;
		bool success = pipeline.ProcessImage( image, displacement );
		
		if( success )
		{
			double dt = timestamp.toSec() - lastTime.toSec();
			PoseSE3::TangentVector tangent = se3log( displacement );
			tangent = tangent/dt;
			
			geometry_msgs::TwistStamped msg;
			msg.header.stamp = timestamp;
			msg.twist.linear.x = tangent(0);
			msg.twist.linear.y = tangent(1);
			msg.twist.linear.z = 0;
			msg.twist.angular.x = 0;
			msg.twist.angular.y = 0;
			msg.twist.angular.z = tangent(5);
			
			posePub.publish( msg );
			
		}
		
		lastTime = timestamp;
		
	}
	
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "optical_flow" );
	VisualOdometryNode flow;
	ros::spin();
	return 0;
}
