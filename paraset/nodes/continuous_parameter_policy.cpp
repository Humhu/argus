#include <ros/ros.h>
#include "paraset/ContinuousParamPolicy.h"
#include "poli/ContinuousPolicyManager.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;

class ContinuousParameterPolicyNode
{
public:

	ContinuousParameterPolicyNode( ros::NodeHandle& nh,
	                               ros::NodeHandle& ph )
	{
		_interface.Initialize( nh, ph );
		_manager.Initialize( &_interface, nh, ph );

		double updateRate;
		GetParamRequired( ph, "update_rate", updateRate );
		_updateTimer = nh.createTimer( ros::Duration( 1.0/updateRate ),
		                               &ContinuousParameterPolicyNode::TimerCallback,
		                               this );
	}

private:

	ContinuousParamPolicy _interface;
	percepto::ContinuousPolicyManager _manager;

	ros::Timer _updateTimer;

	void TimerCallback( const ros::TimerEvent& event )
	{
		_manager.Execute( event.current_real );
	}

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "continuous_parameter_policy" );

	ros::NodeHandle nh, ph( "~" );
	ContinuousParameterPolicyNode cppn( nh, ph );
	ros::spin();
	return 0;
}