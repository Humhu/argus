#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <argus_utils/geometry/PoseSE3.h>
#include <argus_utils/geometry/GeometryUtils.h>
#include <argus_utils/utils/MatrixUtils.h>

using namespace argus;

class DummyOdometer
{
public:
	
	DummyOdometer( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph )
	{
		double rate;
		privHandle.param( "rate", rate, 10.0 );
		
		if( !privHandle.getParam( "frame_id", frameID ) )
		{
			ROS_ERROR_STREAM( "Please specify frame_id" );
		}
		
		std::vector<double> cov;
		if( !privHandle.getParam( "covariance_rate", cov ) )
		{
			covRate = PoseSE3::CovarianceMatrix::Identity();
		}
		else
		{
			if( !ParseMatrix( cov, covRate ) )
			{
				ROS_ERROR_STREAM( "Could not parse covariance rate matrix." );
				exit( -1 );
			}
		}
		
		dispPub = nodeHandle.advertise<geometry_msgs::PoseWithCovarianceStamped>
		    ( "displacements", rate );
	
		timer = std::make_shared<ros::Timer>( 
		    nh.createTimer( ros::Duration( 1.0/rate ), 
	                        boost::bind( &DummyOdometer::TimerCallback, this, _1 ) ) );
	}
	
	void TimerCallback( const ros::TimerEvent& event )
	{
		PoseSE3 displacement; // Zero
		
		geometry_msgs::PoseWithCovarianceStamped msg;
		msg.header.stamp = event.current_real;
		msg.header.frame_id = frameID;
		
		msg.pose.pose = PoseToMsg( displacement );
		
		double dt = (event.current_real - event.last_real).toSec();
		SerializeMatrix( dt * covRate, msg.pose.covariance );
		
		dispPub.publish( msg );
	}
	
private:
	
	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;
	std::shared_ptr<ros::Timer> timer;
	ros::Publisher dispPub;
	
	std::string frameID;
	PoseSE3::CovarianceMatrix covRate;
	
};

int main( int argc, char** argv )
{
	
	ros::init( argc, argv, "dummy_odometry" );
	
	ros::NodeHandle nh, ph( "~" );
	DummyOdometer odometer( nh, ph );
	ros::spin();
	return 0;
}
