#include <ros/ros.h>

#include "paraset/TDErrorCritic.h"
#include "paraset/RewardStamped.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class TDErrorNode
{
public:

	TDErrorNode( ros::NodeHandle& nh,
	             ros::NodeHandle& ph )
	{
		_critic.Initialize( nh, ph );

		_valuePub = ph.advertise<paraset::RewardStamped>( "output", 0 );

		double offset;
		GetParamRequired( ph, "sample_offset", offset );
		_offset = ros::Duration( offset );

		double updateRate;
		GetParamRequired( ph, "update_rate", updateRate );
		_publishTimer = nh.createTimer( ros::Duration( 1.0/updateRate ),
		                                &TDErrorNode::UpdateCallback,
		                                this );
	}

private:

	TDErrorCritic _critic;

	ros::Duration _offset;
	ros::Publisher _valuePub;
	ros::Timer _publishTimer;

	void UpdateCallback( const ros::TimerEvent& event )
	{
		ros::Time queryTime = event.current_real - _offset;
		try
		{
			paraset::RewardStamped msg;
			msg.header.stamp = queryTime;
			msg.reward = _critic.Evaluate( ParamAction( queryTime, VectorType() ) );
		}
		catch( std::out_of_range e )
		{
			ROS_WARN_STREAM( "Could not publish critique at time: " << queryTime );
		}
	}

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "td_error_node" );

	ros::NodeHandle nh, ph( "~" );
	TDErrorNode avn( nh, ph );
	ros::spin();

	return 0;
}