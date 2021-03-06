#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include "camplex/CameraCalibration.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <unordered_map>

#include <argus_utils/utils/ParamUtils.h>
#include <argus_utils/synchronization/SynchronizationTypes.h>

using namespace argus;

class UndistortionNode
{
public:

	struct UndistortMaps
	{
		cv::Mat distMap1;
		cv::Mat distMap2;
	};

	UndistortionNode( const ros::NodeHandle& nh,
	                  const ros::NodeHandle& ph )
		: _imagePort( nh ), _mapsInited( false )
	{
		unsigned int inBuffSize, outBuffSize;
		GetParam( ph, "input_buffer_size", inBuffSize, (unsigned int) 5 );
		GetParam( ph, "output_buffer_size", outBuffSize, (unsigned int) 5 );
		GetParam( ph, "cache_undistortion", _useMaps, false );

		_imageSub = _imagePort.subscribeCamera( "image_raw",
		                                        inBuffSize,
		                                        &UndistortionNode::ImageCallback,
		                                        this );
		_imagePub = _imagePort.advertiseCamera( "image_undistorted", outBuffSize );
	}

	const UndistortMaps& GetMaps( const std::string& cameraName,
	                              const cv::Mat& frame,
	                              const CameraCalibration& calib )
	{
		WriteLock lock( _mutex );
		if( _mapRegistry.count( cameraName ) > 0 )
		{
			return _mapRegistry[cameraName];
		}

		UndistortMaps& maps = _mapRegistry[cameraName];
		maps.distMap1 = cv::Mat( frame.size(), CV_16SC2 );
		maps.distMap2 = cv::Mat( frame.size(), CV_16UC1 );
		cv::initUndistortRectifyMap( calib.GetIntrinsicMatrix(),
		                             calib.GetDistortionCoeffs(),
		                             cv::noArray(),
		                             calib.GetIntrinsicMatrix(),
		                             frame.size(),
		                             CV_16SC2,
		                             maps.distMap1,
		                             maps.distMap2 );
		return maps;
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

		CameraCalibration calib( "", *info );

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
			const UndistortMaps& maps = GetMaps( msg->header.frame_id,
			                                     frame->image,
			                                     calib );
			cv::remap( frame->image,
			           undistorted,
			           maps.distMap1,
			           maps.distMap2,
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

	Mutex _mutex;

	bool _useMaps;
	bool _mapsInited;

	std::unordered_map<std::string, UndistortMaps> _mapRegistry;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "undistortion_node" );

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle( "~" );
	UndistortionNode undisto( nodeHandle, privHandle );

	unsigned int numThreads;
	GetParam( privHandle, "num_threads", numThreads, (unsigned int) 1 );
	ros::AsyncSpinner spinner( numThreads );
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
