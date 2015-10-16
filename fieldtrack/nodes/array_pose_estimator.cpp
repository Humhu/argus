#include <ros/ros.h>

#include "fieldtrack/ArrayPoseEstimator.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "array_pose_estimator" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	fieldtrack::ArrayPoseEstimator estimator( nh, ph );
	ros::spin();
	
	exit( 0 );
}
