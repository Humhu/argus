#include "paraset/ApproximateValueLearner.h"

using namespace argus;

int main( int argc, char** argv )
{
	ros::init( argc, argv, "approximate_value_learner_node" );

	ros::NodeHandle nh, ph( "~" );
	ApproximateValueLearner avl;
	avl.Initialize( nh, ph );
	ros::spin();
	
	return 0;
}