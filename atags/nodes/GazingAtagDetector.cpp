#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <cmath>

#include "atags/AtagDetector.h"
#include "argus_msgs/ImageFiducialDetections.h"
#include "camplex/CameraCalibration.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/synchronization/MessageThrottler.hpp"

using namespace argus;

class GazingAtagNode
{
public:

	GazingAtagNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _imagePort( nh )
	{
		double s;
		GetParamRequired( ph, "poll_rate", s );
		_sleepTime = ros::Duration( 1.0 / s );

		GetParamRequired( ph, "min_framerate", _minFramerate );
        GetParamRequired( ph, "decay_rate", _decayRate );

		ros::NodeHandle th( ph.resolveName( "throttle" ) );
		unsigned int buffLen;
		GetParamRequired( th, "buffer_length", buffLen );
		double r;
		GetParamRequired( th, "max_rate", r );
		_throttle.SetBufferLength( buffLen );
		_throttle.SetTargetRate( r );

		ros::NodeHandle dh( ph.resolveName( "detector" ) );
		_detector.ReadParams( dh );

		_detPub = ph.advertise<argus_msgs::ImageFiducialDetections>( "detections", 20 );

		GetParam<unsigned int>( ph, "buffer_size", buffLen, 10 );
		_cameraSub = _imagePort.subscribeCamera( "image", buffLen, &GazingAtagNode::ImageCallback, this );
	}

	void Spin()
	{
		DataThrottler::KeyedData data;
		while( ros::ok() )
		{
            ros::Time now = ros::Time::now();
            UpdateThrottles( now );
			if( !_throttle.GetOutput( now.toSec(), data ) )
			{
				_sleepTime.sleep();
				continue;
			}

			const std::string& sourceName = data.first;
			const sensor_msgs::Image::ConstPtr& img = data.second.first;
			const sensor_msgs::CameraInfo::ConstPtr& info = data.second.second;

			camplex::CameraCalibration cameraModel( img->header.frame_id, *info );

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

			if( detections.detections.size() == 0 ) { continue; }

            _lastDetectTimes[ detections.sourceName ] = now;

			_detPub.publish( detections.ToMsg() );
		}
	}

private:

	AtagDetector _detector;

	ros::Publisher _detPub;

	image_transport::ImageTransport _imagePort;
	image_transport::CameraSubscriber _cameraSub;

	typedef std::pair<sensor_msgs::Image::ConstPtr, sensor_msgs::CameraInfo::ConstPtr> CameraData;
	typedef MessageThrottler<CameraData> DataThrottler;
	DataThrottler _throttle;
	ros::Duration _sleepTime;

    double _decayRate;
	double _minFramerate;
	std::map<std::string, ros::Time> _lastDetectTimes;

    void UpdateThrottles( const ros::Time& now )
    {
        typedef std::map<std::string, ros::Time>::value_type Item;
        BOOST_FOREACH( const Item& item, _lastDetectTimes )
        {
            const std::string& sourceName = item.first;
            const ros::Time& lastDetectTime = item.second;
            double dt = (now - lastDetectTime).toSec();
            // TODO Incorporate min rate into throttler logic
            double weight = std::exp( -_decayRate * dt ) + _minFramerate;
            _throttle.SetSourceWeight( sourceName, weight );
        }
    }

	void ImageCallback( const sensor_msgs::Image::ConstPtr& img,
	                    const sensor_msgs::CameraInfo::ConstPtr& info )
	{
		const std::string& sourceName = img->header.frame_id;
		try
		{
			_throttle.BufferData( sourceName, CameraData( img, info ) );
		}
		catch( const std::invalid_argument& e )
		{
			_throttle.RegisterSource( sourceName );
			_throttle.SetSourceWeight( sourceName, 1.0 );
			_lastDetectTimes[sourceName] = img->header.stamp;
			ROS_INFO_STREAM( "Registering source: " << sourceName );
			_throttle.BufferData( sourceName, CameraData( img, info ) );
		}
	}

};

int main( int argc, char**argv )
{

	ros::init( argc, argv, "gazing_atag_detector" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );

	GazingAtagNode node( nh, ph );

	ros::AsyncSpinner spinner( 1 );
	spinner.start();
	node.Spin();

	return 0;

}
