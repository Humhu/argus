#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <argus_utils/geometry/PoseSE3.h>
#include <argus_utils/utils/ParamUtils.h>
#include <argus_utils/geometry/GeometryUtils.h>
#include <argus_utils/utils/MatrixUtils.h>

using namespace argus;

class DummyOdometer
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	DummyOdometer( ros::NodeHandle& nh, ros::NodeHandle& ph )
	{
		
		GetParamRequired( ph, "frame_id", _frameID );		
		GetParamRequired( ph, "covariance_rate", _covRate );
		
		_odomPub = nh.advertise<nav_msgs::Odometry>( "odom", 10 );
		
		double rate;
		GetParam( ph, "rate", rate, 10.0 );
		_timer = nh.createTimer( ros::Duration( 1.0 / rate ),
		                        boost::bind( &DummyOdometer::TimerCallback, this, _1 ) );
	}

	void TimerCallback( const ros::TimerEvent& event )
	{
		nav_msgs::Odometry odom;
		odom.header.frame_id = _frameID;
		odom.header.stamp = event.current_real;

		PoseSE3::CovarianceMatrix poseCov = PoseSE3::CovarianceMatrix::Zero();
		odom.pose.pose = PoseToMsg( PoseSE3() );
		SerializeMatrix( poseCov, odom.pose.covariance );

		double dt = (event.current_real - event.last_real).toSec();
		PoseSE3::TangentVector vel = PoseSE3::TangentVector::Zero();
		PoseSE3::CovarianceMatrix velCov = _covRate * dt;
		odom.twist.twist = TangentToMsg( vel );
		SerializeMatrix( velCov, odom.twist.covariance );

		_odomPub.publish( odom );
	}

private:

	ros::Timer _timer;
	ros::Publisher _odomPub;

	std::string _frameID;
	PoseSE3::CovarianceMatrix _covRate;
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "dummy_odometry" );

	ros::NodeHandle nh, ph( "~" );
	DummyOdometer odometer( nh, ph );
	ros::spin();
	return 0;
}
