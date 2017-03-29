#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "camplex/CameraCalibration.h"
#include <opencv2/imgproc/imgproc.hpp>

class UndistortionNode 
{
public:
	
	UndistortionNode( const ros::NodeHandle& nh, 
	                  const ros::NodeHandle& ph )
	: _imagePort( nh ), _mapsInited( false )
	{
		int inBuffSize, outBuffSize;
		ph.param( "input_buffer_size", inBuffSize, 5 );
		ph.param( "output_buffer_size", outBuffSize, 5 );
		ph.param( "cache_undistortion", _useMaps, false );

		_imageSub = _imagePort.subscribeCamera( "image_raw", 
		                                        inBuffSize, 
		                                        &UndistortionNode::ImageCallback, 
		                                        this );
		_imagePub = _imagePort.advertiseCamera( "image_undistorted", outBuffSize );
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
		
		camplex::CameraCalibration calib( "", *info );
		
		if( _useMaps && !_mapsInited )
		{
			_distMap1 = cv::Mat( frame->image.size(), CV_16SC2 );
			_distMap2 = cv::Mat( frame->image.size(), CV_16UC1 );
			cv::initUndistortRectifyMap( calib.GetIntrinsicMatrix(),
			                             calib.GetDistortionCoeffs(),
			                             cv::noArray(),
			                             calib.GetIntrinsicMatrix(),
			                             frame->image.size(),
			                             CV_16SC2,
			                             _distMap1,
			                             _distMap2 );
			_mapsInited = true;
		}

		cv::Mat undistorted( frame->image.size(), frame->image.type() );

		if( !_useMaps )
		{
			cv::undistort( frame->image, 
			               undistorted, 
			               calib.GetIntrinsicMatrix(), 
			               calib.GetDistortionCoeffs() );
		}
		else
		{
			cv::remap( frame->image, 
			           undistorted,
			           _distMap1, 
			           _distMap2, 
			           cv::INTER_LINEAR );
		}

		sensor_msgs::ImagePtr outImage = cv_bridge::CvImage( msg->header, 
		                                                     msg->encoding, 
		                                                     undistorted ).toImageMsg();
		sensor_msgs::CameraInfoPtr outInfo = boost::make_shared<sensor_msgs::CameraInfo>( *info );
		outInfo->D = std::vector<double>( 5, 0.0 );
		_imagePub.publish( outImage, outInfo );
	}
	
private:

	image_transport::ImageTransport _imagePort;
	image_transport::CameraSubscriber _imageSub;
	image_transport::CameraPublisher _imagePub;

	bool _useMaps;
	bool _mapsInited;
	cv::Mat _distMap1;
	cv::Mat _distMap2;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "undistortion_node" );
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle("~");
	UndistortionNode undisto( nodeHandle, privHandle );
	
	ros::spin();
	
	return 0;
}
