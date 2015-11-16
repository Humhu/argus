#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <argus_utils/PoseSE3.h>
#include <argus_utils/GeometryUtils.h>
#include <argus_utils/MatrixUtils.h>

using namespace argus_utils;

void TimerCallback( const ros::TimerEvent& event,
                    ros::Publisher& pub,
                    const std::string& frame_id )
{
	
	PoseSE3 displacement; // Zero
	PoseSE3::CovarianceMatrix cov = PoseSE3::CovarianceMatrix::Identity();
	
	geometry_msgs::PoseWithCovarianceStamped msg;
	msg.header.stamp = event.current_real;
	msg.header.frame_id = frame_id;
	
	msg.pose.pose = PoseToMsg( displacement );
	SerializeMatrix( cov, msg.pose.covariance );
	
	pub.publish( msg );
}

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "dummy_odometry" );
	
	ros::NodeHandle nh, ph( "~" );
	
	double rate;
	ph.param( "rate", rate, 10.0 );
	
	std::string source;
	if( !ph.getParam( "frame_id", source ) )
	{
		ROS_ERROR_STREAM( "Please specify frame_id" );
	}
	
	ros::Publisher pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>
	    ( "displacements", rate );
	
	ros::Timer timer = nh.createTimer( ros::Duration( 1.0/rate ), 
	                                   boost::bind( &TimerCallback, 
	                                                _1, 
	                                                boost::ref( pub ),
	                                                boost::cref( source ) ) );
	
	ros::spin();
	
}
