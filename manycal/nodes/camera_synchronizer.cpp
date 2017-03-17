#include <ros/ros.h>
#include <deque>
#include <boost/foreach.hpp>

#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "argus_utils/synchronization/MessageSynchronizer.hpp"

#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class CameraSynchronizer
{
public:

	CameraSynchronizer( ros::NodeHandle& nh, ros::NodeHandle& ph )
		: publicPort( nh )
	{
		unsigned int buffLen, minSync;
		double dt;
		GetParamRequired( ph, "sync/buffer_length", buffLen );
		GetParamRequired( ph, "sync/max_dt", dt );
		GetParam<unsigned int>( ph, "sync/min_sync_num", minSync, 0 );

		_sync.SetBufferLength( buffLen );
		_sync.SetMaxDt( dt );
		_sync.SetMinSyncNum( minSync );

		YAML::Node sources;
		GetParamRequired( ph, "sources", sources );
		YAML::Node::const_iterator iter;
		for( iter = sources.begin(); iter != sources.end(); ++iter )
		{
			const std::string& name = iter->first.as<std::string>();
			const YAML::Node& info = iter->second;

			std::string input_topic, output_topic;
			GetParamRequired( info, "input", input_topic );
			_sync.RegisterSource( name );
			_subscribers.emplace_back(
			    publicPort.subscribeCamera( input_topic, 10,
			                                boost::bind( &CameraSynchronizer::CameraCallback, this,
			                                             name, _1, _2 ) ) );

			GetParamRequired( info, "output", output_topic );
			_publishers[name] = publicPort.advertiseCamera( output_topic, 10 );
		}

		double pollRate;
		GetParamRequired( ph, "poll_rate", pollRate );
		_pollTimer = nh.createTimer( ros::Duration( 1.0 / pollRate ), &CameraSynchronizer::TimerCallback,
		                             this );
	}

private:

	ros::Timer _pollTimer;
	image_transport::ImageTransport publicPort;
	std::deque<image_transport::CameraSubscriber> _subscribers;
	std::map<std::string, image_transport::CameraPublisher> _publishers;

	typedef std::pair<sensor_msgs::Image::ConstPtr, sensor_msgs::CameraInfo::ConstPtr> CameraData;
	typedef MessageSynchronizer<CameraData> DataSynchronizer;
	DataSynchronizer _sync;

	void TimerCallback( const ros::TimerEvent& event )
	{
		std::vector<DataSynchronizer::KeyedStampedData> data;
		if( !_sync.GetOutput( data ) ) { return; }

		BOOST_FOREACH( const DataSynchronizer::KeyedStampedData & datum, data )
		{
			const std::string& name = std::get<0>( datum );
			const CameraData& data = std::get<2>( datum );
			_publishers[name].publish( data.first, data.second );
		}
	}

	void CameraCallback( const std::string& name,
	                     const sensor_msgs::Image::ConstPtr& image,
	                     const sensor_msgs::CameraInfo::ConstPtr& info )
	{
		CameraData data( image, info );
		_sync.BufferData( name, image->header.stamp.toSec(), data );
	}

};

int main( int argc, char**argv )
{
	ros::init( argc, argv, "camera_synchronizer" );

	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );

	unsigned int numThreads;
	GetParam<unsigned int>( ph, "num_threads", numThreads, 1 );

	CameraSynchronizer calibrator( nh, ph );

	//ros::spin();
	ros::AsyncSpinner spinner( numThreads );
	spinner.start();
	ros::waitForShutdown();

	return 0;
}
