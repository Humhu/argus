#include "paraset/RewardInterpolater.h"
#include "paraset/MonteCarloValue.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class MonteCarloValuePublisher
{
public:

	MonteCarloValuePublisher( ros::NodeHandle& nh,
	                          ros::NodeHandle& ph )
	{

		double offset;
		GetParamRequired( ph, "timer_offset", offset );
		_timerOffset = ros::Duration( offset );

		ros::NodeHandle vh( ph.resolveName( "value_estimation" ) );
		_value.Initialize( nh, vh );

		_valuePub = ph.advertise<paraset::RewardStamped>( "value", 0 );

		double timerRate;
		GetParamRequired( ph, "timer_rate", timerRate );
		_timer = nh.createTimer( ros::Duration( 1.0/timerRate ),
		                         &MonteCarloValuePublisher::TimerCallback,
		                         this );
	}

private:

	MonteCarloValue _value;
	
	ros::Duration _timerOffset;
	ros::Timer _timer;
	ros::Publisher _valuePub;

	void TimerCallback( const ros::TimerEvent& event )
	{
		ros::Time query = event.current_real - _timerOffset;

		paraset::RewardStamped msg;
		msg.header.stamp = query;
		try
		{
			msg.reward = _value.Evaluate( ParamAction( query, VectorType() ) );
		}
		catch( std::out_of_range& e )
		{
			ROS_WARN_STREAM( "Could not query value function: " << e.what() );
			return;
		}

		_valuePub.publish( msg );
	}

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "monte_carlo_value_node" );

	ros::NodeHandle nh, ph( "~" );
	MonteCarloValuePublisher mcvpub( nh, ph );
	ros::spin();
}