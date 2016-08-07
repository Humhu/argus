#include <ros/ros.h>
#include "broadcast/BroadcastMultiReceiver.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class BroadcastListener
{
public:

	BroadcastListener()
	{
		ros::NodeHandle ph( "~" );
		double printRate;
		GetParam( ph, "print_rate", printRate, 1.0 );
		_printTimer = ph.createTimer( ros::Duration( 1.0/printRate ),
		                              &BroadcastListener::TimerCallback,
		                              this );

		if( ph.hasParam( "topics" ) )
		{
			ros::NodeHandle topich( "~/topics" );
			_mrx.Initialize( topich );
			_multi = true;
		}
		else
		{
			YAML::Node props;
			GetParamRequired( ph, "", props );
			std::string streamName;
			GetParamRequired( props, "stream_name", streamName );
			_rx.Initialize( streamName, props );
			_multi = false;
		}
	}

private:

	bool _multi;
	BroadcastReceiver _rx;
	BroadcastMultiReceiver _mrx;
	ros::Timer _printTimer;

	void TimerCallback( const ros::TimerEvent& event )
	{
		StampedFeatures f;
		ros::Time start = ros::Time::now();
		if( (_multi && !_mrx.ReadStream( event.current_real, f )) ||
		    (!_multi && !_rx.ReadStream( event.current_real, f )) )
		{
			ROS_WARN_STREAM( "Could not read stream at time: " << event.current_real );
			return;
		}
		ros::Time finish = ros::Time::now();
		ros::Duration latency = finish - start;
		ROS_INFO_STREAM( "Query time: " << event.current_real << 
		                 " Query latency: " << latency <<
		                 " Feature time: " << f.time << 
		                 " features: " << f.features.transpose() << std::endl );
	}

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "broadcast_listener" );
	BroadcastListener listener;
	ros::spin();
	return 0;
}