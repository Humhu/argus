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
#include "argus_utils/synchronization/WorkerPool.h"

using namespace argus;

class GazingAtagNode
{
public:

	GazingAtagNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _imagePort( nh )
	{
		// double s;
		// GetParamRequired( ph, "poll_rate", s );
		// _pollTimer = nh.createTimer( ros::Duration( 1.0 / s ),
		//                              &GazingAtagNode::TimerCallback, this );

		GetParamRequired( ph, "decay_rate", _decayRate );

		ros::NodeHandle th( ph.resolveName( "throttle" ) );
		double maxRate, minRate;
		unsigned int buffLen;
		GetParamRequired( th, "buffer_length", buffLen );
		GetParamRequired( th, "max_rate", maxRate );
		GetParamRequired( th, "min_rate", minRate );
		_throttle.SetBufferLength( buffLen );
		_throttle.SetTargetRate( maxRate );
		_throttle.SetMinRate( minRate );

		ros::NodeHandle dh( ph.resolveName( "detector" ) );
		_detector.ReadParams( dh );

		_detPub = ph.advertise<argus_msgs::ImageFiducialDetections>( "detections", 20 );

		GetParam<unsigned int>( ph, "buffer_size", buffLen, 10 );
		_cameraSub = _imagePort.subscribeCamera( "image", buffLen, &GazingAtagNode::ImageCallback, this );

		double detRate;
		GetParam( ph, "detector_poll_rate", detRate, 1.0 );
		_detectorRest = ros::Duration( 1.0 / detRate );

		unsigned int numDetectorThreads;
		GetParam<unsigned int>( ph, "num_detector_threads", numDetectorThreads, 1 );
		_detectorWorkers.SetNumWorkers( numDetectorThreads );
		for( unsigned int i = 0; i < numDetectorThreads; ++i )
		{
			WorkerPool::Job job = boost::bind( &GazingAtagNode::DetectionSpin, this );
			_detectorWorkers.EnqueueJob( job );
		}
		_detectorWorkers.StartWorkers();
	}

private:

	AtagDetector _detector;
	WorkerPool _detectorWorkers;
	ros::Duration _detectorRest;

	ros::Publisher _detPub;
	ros::Timer _pollTimer;

	image_transport::ImageTransport _imagePort;
	image_transport::CameraSubscriber _cameraSub;

	typedef std::pair<sensor_msgs::Image::ConstPtr, sensor_msgs::CameraInfo::ConstPtr> CameraData;
	typedef MessageThrottler<CameraData> DataThrottler;
	DataThrottler _throttle;

	Mutex _decayMutex;
	double _decayRate;
	std::map<std::string, ros::Time> _lastDetectTimes;

	// void TimerCallback( const ros::TimerEvent& event )
	void DetectionSpin()
	{
		DataThrottler::KeyedData data;
		while( !ros::isShuttingDown() )
		{
			ros::Time now = ros::Time::now();
			UpdateThrottles( now );
			if( !_throttle.GetOutput( now.toSec(), data ) )
			{
				_detectorRest.sleep();
				continue;
			}

			const sensor_msgs::Image::ConstPtr& img = data.second.first;
			const sensor_msgs::CameraInfo::ConstPtr& info = data.second.second;


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

			CameraCalibration cameraModel( img->header.frame_id, *info );
			cameraModel.SetScale( frame.size() );

			ImageFiducialDetections detections;
			detections.sourceName = img->header.frame_id;
			detections.timestamp = img->header.stamp;
			detections.detections = _detector.ProcessImage( frame, cameraModel );

			if( detections.detections.size() == 0 ) { continue; }

			WriteLock lock( _decayMutex );
			_lastDetectTimes[detections.sourceName] = now;
			lock.unlock();

			_detPub.publish( detections.ToMsg() );
		}
	}

	void ProcessData( const CameraData& data )
	{
		
	}

	void UpdateThrottles( const ros::Time& now )
	{
		ReadLock lock( _decayMutex );
		typedef std::map<std::string, ros::Time>::value_type Item;
		BOOST_FOREACH( const Item &item, _lastDetectTimes )
		{
			const std::string& sourceName = item.first;
			const ros::Time& lastDetectTime = item.second;
			double dt = ( now - lastDetectTime ).toSec();
			if( dt < 0 )
			{
				ROS_WARN_STREAM( "Throttle update with negative dt: " << dt );
				return;
			}
			// TODO Incorporate min rate into throttler logic
			double weight = std::exp( -_decayRate * dt );
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

	unsigned int numThreads;
	GetParam<unsigned int>( ph, "num_rx_threads", numThreads, 1 );
	ros::AsyncSpinner spinner( numThreads );
	spinner.start();
	ros::waitForShutdown();

	return 0;

}
