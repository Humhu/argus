#include "paraset/ContinuousPolicyLearner.h"

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "continuous_policy_learner_node" );

	ros::NodeHandle nh, ph( "~" );
	ContinuousPolicyLearner cpl;
	cpl.Initialize( nh, ph );
	ros::spin();
	
	return 0;
}