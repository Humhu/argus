#include <ros/ros.h>

#include "camplex/FiducialPoseEstimator.h"

int main( int argc, char** argv )
{
	ros::init( argc, argv, "fiducial_pose_estimator" );
	
	ros::NodeHandle nh;
	ros::NodeHandle ph( "~" );
	
	argus::FiducialPoseEstimator estimator( nh, ph );
	ros::spin();
	
	exit( 0 );
}
