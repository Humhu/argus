#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "atags/AtagDetector.h"
#include "argus_msgs/ImageFiducialDetections.h"
#include "camplex/CameraCalibration.h"

#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class AtagNode
{
public:

	AtagNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: _imagePort( nh )
	{
		ros::NodeHandle dh( ph.resolveName("detector") );
		_detector.ReadParams( dh );

		_detPub = ph.advertise<argus_msgs::ImageFiducialDetections>( "detections", 20 );

		unsigned int buffLen;
		GetParam<unsigned int>( ph, "buffer_size", buffLen, 10 );
		_cameraSub = _imagePort.subscribeCamera( "image", buffLen, &AtagNode::ImageCallback, this );
	}
	
private:

	AtagDetector _detector;

	ros::Publisher _detPub;

	image_transport::ImageTransport _imagePort;
	image_transport::CameraSubscriber _cameraSub;

	void ImageCallback( const sensor_msgs::Image::ConstPtr& img,
	                    const sensor_msgs::CameraInfo::ConstPtr& info )
	{
		CameraCalibration cameraModel( img->header.frame_id, *info );
		
		// Detection occurs in grayscale
		cv::Mat msgFrame = cv_bridge::toCvShare( img )->image;
		cv::Mat frame;
		if( msgFrame.channels() > 1 )
		{
			cv::cvtColor( msgFrame, frame, CV_BGR2GRAY );
		}
		else
		{
			frame = msgFrame;
		}

		ImageFiducialDetections detections;
		detections.sourceName = img->header.frame_id;
		detections.timestamp = img->header.stamp;
		detections.detections = _detector.ProcessImage( frame, cameraModel );
		
		if( detections.detections.size() == 0 ) { return; }
		
		_detPub.publish( detections.ToMsg() );
	}
	
};

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "atag_detector" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	AtagNode node( nh, ph );
	
	int numSpinners;
	ph.param( "num_threads", numSpinners, 1 );
	
	ros::AsyncSpinner spinner( numSpinners );
	spinner.start();
	ros::waitForShutdown();
	
	return 0;
	
}
