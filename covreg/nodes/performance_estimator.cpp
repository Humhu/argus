#include <ros/ros.h>
#include <argus_msgs/EstimatePerformance.h>
#include <nav_msgs/Odometry.h>
#include <argus_utils/random/MultivariateGaussian.hpp>
#include <argus_utils/utils/MatrixUtils.h>
#include <argus_utils/utils/ParamUtils.h>

using namespace argus;

class PerformanceEstimator
{
public:

	// TODO For now just x vel and yaw vel
	PerformanceEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
	: mvn( 3 )
	{
		GetParamRequired<unsigned int>( ph, "num_samples", numSamples );
		odomSub = nh.subscribe( "odom", 10, &PerformanceEstimator::OdomCallback, this );
		perfPub = nh.advertise<argus_msgs::EstimatePerformance>( "performance", 10 );
	}

private:

	ros::Subscriber odomSub;
	ros::Publisher perfPub;

	unsigned int numSamples;

	MultivariateGaussian<> mvn;

	void OdomCallback( const nav_msgs::Odometry::ConstPtr& msg )
	{
		MatrixType twistCov(6,6);
		if( !ParseMatrix( msg->twist.covariance, twistCov ) )
		{
			ROS_ERROR_STREAM( "Error in parsing covariance." );
			return;
		}
		MatrixType subCov(3,3);
		std::array<unsigned int,3> inds = {0,1,5};
		GetSubmatrix( twistCov, subCov, inds, inds );
		// mvn.SetCovariance( subCov );

		// double rmsLinAcc = 0;
		// double rmsAngAcc = 0;
		// for( unsigned int i = 0; i < numSamples; ++i )
		// {
		// 	VectorType vel = mvn.Sample();
		// 	rmsLinAcc += LinearRMS( vel );
		// 	rmsAngAcc += AngularRMS( vel );
		// }

		// argus_msgs::EstimatePerformance perf;
		// perf.rms_linear_vel = rmsLinAcc/numSamples;
		// perf.rms_angular_vel = rmsAngAcc/numSamples;
		// perfPub.publish( perf );

		// RMS is approximately mean of the diagonals rooted
		argus_msgs::EstimatePerformance perf;
		perf.rms_linear_vel = std::sqrt( (subCov(0,0) + subCov(1,1)) * 0.5 );
		perf.rms_angular_vel = std::sqrt( subCov(2,2) );
		perfPub.publish( perf );
	}

	double LinearRMS( const VectorType& vel )
	{
		return vel.head(1).norm();
	}

	double AngularRMS( const VectorType& vel )
	{
		return vel.tail(1).norm();
	}

};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "performance_estimator" );

	ros::NodeHandle nh, ph( "~" );
	PerformanceEstimator pe( nh, ph );
	ros::spin();
	return 0;
}