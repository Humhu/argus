#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include "camplex/CameraCalibration.h"
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

		bool imageOnly;
		GetParam( ph, "image_only", imageOnly, false );
		if( imageOnly )
		{
			ROS_INFO_STREAM( "Operating in image mode - resizing image only" );
			_imageSub = _imagePort.subscribe( "image_raw",
			                                  inBuffSize,
			                                  &SubsamplerNode::ImageCallback,
			                                  this );
			_imagePub = _imagePort.advertise( "image_resized", outBuffSize );
		}
		else
		{
			ROS_INFO_STREAM( "Operating in camera mode - resizing image and info" );
			_cameraSub = _imagePort.subscribeCamera( "image_raw",
			                                         inBuffSize,
			                                         &SubsamplerNode::CameraCallback,
			                                         this );
			_cameraPub = _imagePort.advertiseCamera( "image_resized", outBuffSize );
		}
	}

	sensor_msgs::ImagePtr DecodeImage( const sensor_msgs::ImageConstPtr& msg )
	{
		cv_bridge::CvImageConstPtr frame;
		try
		{
			frame = cv_bridge::toCvShare( msg, msg->encoding );
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR( "cv_bridge exception: %s", e.what() );
			return nullptr;
		}

		cv::Mat resized;
		cv::resize( frame->image, resized, cv::Size( 0, 0 ), _outputScale, _outputScale, _interpMode );
		return cv_bridge::CvImage( msg->header, msg->encoding, resized ).toImageMsg();
	}

	void ImageCallback( const sensor_msgs::ImageConstPtr& msg )
	{
		sensor_msgs::ImagePtr outImage = DecodeImage( msg );
		if( !outImage ) { return; }
		_imagePub.publish( outImage );
	}

	void CameraCallback( const sensor_msgs::ImageConstPtr& msg,
	                     const sensor_msgs::CameraInfoConstPtr& info )
	{
		sensor_msgs::ImagePtr outImage = DecodeImage( msg );
		if( !outImage ) { return; }

		CameraCalibration calib( "", *info );
		cv::Size dims = calib.GetScale();
		dims.width *= _outputScale;
		dims.height *= _outputScale;
		calib.SetScale( dims );

		sensor_msgs::CameraInfoPtr infoResized = 
			boost::make_shared<sensor_msgs::CameraInfo>( calib.GetInfo() );

		_cameraPub.publish( outImage, infoResized );
	}

private:

	image_transport::ImageTransport _imagePort;

	image_transport::Subscriber _imageSub;
	image_transport::Publisher _imagePub;

	image_transport::CameraSubscriber _cameraSub;
	image_transport::CameraPublisher _cameraPub;

	cv::InterpolationFlags _interpMode;

	NumericParam _outputScale;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "subsampler_node" );

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle( "~" );
	SubsamplerNode ss( nodeHandle, privHandle );

	ros::spin();

	return 0;
}
