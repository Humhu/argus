#include <ros/ros.h>
#include "paraset/ContinuousParamPolicy.h"
#include "poli/ContinuousPolicyManager.h"
#include "argus_utils/utils/ParamUtils.h"

using namespace argus;
using namespace percepto;

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
		try
		{
			ContinuousAction action = _manager.Execute( event.current_real );
			
			ContinuousPolicy::DistributionParameters dist = _manager.GetNormalizedDistribution( event.current_real );
			std::vector<std::string> paramNames = _interface.GetParameterNames();
			std::stringstream ss;
			for( unsigned int i = 0; i < action.output.size(); i++ )
			{
				ss << paramNames[i] << ": " << action.output(i) << std::endl;
			}

			ROS_INFO_STREAM( "Distribution: " << std::endl << dist << std::endl <<
			                 "Action: " << std::endl << ss.str() );
		}
		catch( std::out_of_range )
		{
			ROS_WARN_STREAM( "Could not execute policy at time: " << event.current_real );
		}
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