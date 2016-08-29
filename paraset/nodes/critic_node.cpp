#include <ros/ros.h>

#include "paraset/ContinuousActionCritique.h"
#include "paraset/TDErrorCritic.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/MapUtils.hpp"

using namespace argus;

class CriticNode
{
public:

	CriticNode( ros::NodeHandle& nh,
	                      ros::NodeHandle& ph )
	{
		_critic.Initialize( nh, ph );

		double offset;
		GetParamRequired( ph, "critique_offset", offset );
		_critiqueOffset = ros::Duration( offset );

		_critiquePub = ph.advertise<paraset::ContinuousActionCritique>( "critique", 0 );
		_actionSub = nh.subscribe( "actions", 0, &CriticNode::ActionCallback, this );

		double updateRate;
		GetParamRequired( ph, "update_rate", updateRate );
		_updateTimer = nh.createTimer( ros::Duration( 1.0/updateRate ),
		                               &CriticNode::TimerCallback,
		                               this );
	}

private:

	TDErrorCritic _critic;

	ros::Subscriber _actionSub;
	ros::Publisher _critiquePub;
	
	ros::Timer _updateTimer;
	ros::Duration _critiqueOffset;

	typedef std::map<ros::Time, paraset::ContinuousParamAction> ActionBuffer;
	ActionBuffer _actionBuffer;

	void ActionCallback( const paraset::ContinuousParamAction::ConstPtr& msg )
	{
		_actionBuffer[msg->header.stamp] = *msg;
	}

	void TimerCallback( const ros::TimerEvent& event )
	{
		while( _actionBuffer.size() > 0 )
		{
			ContinuousParamAction action( _actionBuffer.begin()->second );
			if( action.time + _critiqueOffset > event.current_real )
			{
				return;
			}

			remove_lowest( _actionBuffer );

			paraset::ContinuousActionCritique msg;
			msg.action = action.ToMsg();
			msg.reward = _critic.GetReward( action.time );
			msg.advantage = _critic.Evaluate( ParamAction( action.time, action.output ) );
			_critiquePub.publish( msg );
		}
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "critic_node" );

	ros::NodeHandle nh, ph( "~" );
	CriticNode avn( nh, ph );
	ros::spin();

	return 0;
}