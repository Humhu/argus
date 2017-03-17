#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include "atags/AtagDetector.h"
#include "argus_msgs/ImageFiducialDetections.h"
#include "camplex/CameraCalibration.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/synchronization/MessageSynchronizer.hpp"

using namespace argus;
namespace smsgs = sensor_msgs;

class SyncingAtagNode
{
public:

	SyncingAtagNode( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: _imagePort( nh )
	{
		double maxDt;
		unsigned int buffLen, minSync;
		GetParamRequired( ph, "sync/buffer_length", buffLen );
		GetParamRequired( ph, "sync/max_dt", maxDt );
		GetParam<unsigned int>( ph, "sync/min_sync_num", minSync, 0 );
		_sync.SetBufferLength( buffLen );
		_sync.SetMaxDt( maxDt );
		_sync.SetMinSyncNum( minSync );

		ros::NodeHandle dh( ph.resolveName( "detector" ) );
		_detector.ReadParams( dh );

		_detPub = ph.advertise<argus_msgs::ImageFiducialDetections>( "detections", 20 );

		GetParam<unsigned int>( ph, "buffer_size", buffLen, 10 );
		_cameraSub = _imagePort.subscribeCamera( "image", buffLen, &SyncingAtagNode::ImageCallback, this );

		double s;
		GetParamRequired( ph, "poll_rate", s );
		_pollTimer = nh.createTimer( ros::Duration( 1.0 / s ),
		                             &SyncingAtagNode::TimerCallback, this );
	}

private:

	AtagDetector _detector;

	ros::Publisher _detPub;
	ros::Timer _pollTimer;

	image_transport::ImageTransport _imagePort;
	image_transport::CameraSubscriber _cameraSub;

	typedef std::pair<smsgs::Image::ConstPtr, smsgs::CameraInfo::ConstPtr> CameraData;
	typedef MessageSynchronizer<CameraData> DataSynchronizer;
	DataSynchronizer _sync;

	void TimerCallback( const ros::TimerEvent& event )
	{
		std::vector<DataSynchronizer::KeyedStampedData> out;
		if( !_sync.GetOutput( out ) ) { return; }

		ros::Time stamp = std::get<2>( out[0] ).first->header.stamp;
		BOOST_FOREACH( const DataSynchronizer::KeyedStampedData& item, out )
		{
            const CameraData& data = std::get<2>( item );
            ProcessData( data );
		}
	}

	void ProcessData( const CameraData& data )
	{
		const smsgs::Image::ConstPtr& img = data.first;
		const smsgs::CameraInfo::ConstPtr& info = data.second;

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

		if( detections.detections.size() == 0 ) { return; }

		_detPub.publish( detections.ToMsg() );
	}

	void ImageCallback( const smsgs::Image::ConstPtr& img,
	                    const smsgs::CameraInfo::ConstPtr& info )
	{
		const std::string& sourceName = img->header.frame_id;
        double timestamp = img->header.stamp.toSec();
		try
		{
			_sync.BufferData( sourceName, timestamp, CameraData( img, info ) );
		}
		catch( const std::invalid_argument& e )
		{
			_sync.RegisterSource( sourceName );
			_sync.BufferData( sourceName, timestamp, CameraData( img, info ) );
		}
	}
};

int main( int argc, char**argv )
{

	ros::init( argc, argv, "syncing_atag_detector" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );

	SyncingAtagNode node( nh, ph );

	unsigned int numThreads;
	GetParam<unsigned int>( ph, "num_threads", numThreads, 1 );
	ros::AsyncSpinner spinner( numThreads );
	spinner.start();
	ros::waitForShutdown();

	return 0;

}
