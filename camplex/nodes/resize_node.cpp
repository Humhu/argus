#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "argus_utils/utils/ParamUtils.h"
#include "paraset/ParameterManager.hpp"

using namespace argus;

/*! \brief Downsizes images.
 */
class SubsamplerNode
{
public:

	SubsamplerNode( ros::NodeHandle& nh,
	                ros::NodeHandle& ph )
		: _imagePort( nh )
	{
		unsigned int inBuffSize, outBuffSize;
		GetParam<unsigned int>( ph, "input_buffer_size", inBuffSize, 5 );
		GetParam<unsigned int>( ph, "output_buffer_size", outBuffSize, 5 );

		std::string interpMode;
		GetParam<std::string>( ph, "interpolation_mode", interpMode, "nearest" );
		if( interpMode == "nearest" )
		{
			_interpMode = cv::InterpolationFlags::INTER_NEAREST;
		}
		else if( interpMode == "linear" )
		{
			_interpMode = cv::InterpolationFlags::INTER_LINEAR;
		}
		else if( interpMode == "area" )
		{
			_interpMode = cv::InterpolationFlags::INTER_AREA;
		}
		else
		{
			throw std::invalid_argument( "Unsupported interpolation mode: " + interpMode );
		}

		_outputScale.InitializeAndRead( ph, 1.0, "output_scale",
		                                "Resize scale factor" );

		_imageSub = _imagePort.subscribeCamera( "image_raw",
		                                        inBuffSize,
		                                        &SubsamplerNode::ImageCallback,
		                                        this );
		_imagePub = _imagePort.advertiseCamera( "image_resized", outBuffSize );
	}

	void ImageCallback( const sensor_msgs::ImageConstPtr& msg,
	                    const sensor_msgs::CameraInfoConstPtr& info )
	{
		cv_bridge::CvImageConstPtr frame;
		try
		{
			frame = cv_bridge::toCvShare( msg, msg->encoding );
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return;
		}

		cv::Mat resized;
		cv::resize( frame->image, resized, cv::Size( 0, 0 ), _outputScale, _outputScale, _interpMode );
		sensor_msgs::ImagePtr outImage = cv_bridge::CvImage( msg->header,
		                                                     msg->encoding,
		                                                     resized ).toImageMsg();
		_imagePub.publish( outImage, info );
	}

private:

	image_transport::ImageTransport _imagePort;
	image_transport::CameraSubscriber _imageSub;
	image_transport::CameraPublisher _imagePub;
	cv::InterpolationFlags _interpMode;

	NumericParam _outputScale;
};

int main( int argc, char**argv )
{
	ros::init( argc, argv, "subsampler_node" );

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle( "~" );
	SubsamplerNode ss( nodeHandle, privHandle );

	ros::spin();

	return 0;
}
