#include "camera_array/SystemDistributions.h"
#include <boost/foreach.hpp>

#define POSE_DIM (PoseSE3::TangentDimension)

namespace argus
{

RobotTargetDistribution::Properties::Properties( bool targetPose, bool targetVel,
                                                 bool robotPose, bool robotVel )
: sampleTargetPoses( targetPose ), sampleTargetVelocities( targetVel ),
  sampleRobotPose( robotPose ), sampleRobotVelocity( robotVel ) {}

RobotTargetDistribution::RobotTargetDistribution( Properties props )
: properties( props ), gaussian( 6 )
{}
	
void RobotTargetDistribution::SetMean( const RobotTargetState& b )
{
	base = b;
}
	
RobotTargetState RobotTargetDistribution::Sample()
{
	RobotTargetState sample( base );
	
	if( properties.sampleRobotPose )
	{
		PoseSE3::CovarianceMatrix poseCov = 
		    base.robot.covariance.topLeftCorner( POSE_DIM, POSE_DIM );
		sample.robot.pose = SamplePose( base.robot.pose, poseCov );
	}
	if( properties.sampleRobotVelocity )
	{
		PoseSE3::CovarianceMatrix velCov = 
		    base.robot.covariance.block( POSE_DIM, POSE_DIM, POSE_DIM, POSE_DIM );
		sample.robot.velocity = SampleVelocity( base.robot.velocity, velCov );
	}
	if( properties.sampleTargetPoses )
	{
		BOOST_FOREACH( const RobotTargetState::TargetMap::value_type& item, base.targets )
		{
			const std::string& name = item.first;
			const TargetState& state = item.second;
			PoseSE3::CovarianceMatrix poseCov = 
			    state.covariance.topLeftCorner( POSE_DIM, POSE_DIM );
			sample.targets[ name ].pose = SamplePose( state.pose, poseCov );
		}
	}
	if( properties.sampleTargetVelocities )
	{
		BOOST_FOREACH( const RobotTargetState::TargetMap::value_type& item, base.targets )
		{
			const std::string& name = item.first;
			const TargetState& state = item.second;
			PoseSE3::CovarianceMatrix velCov = 
			    state.covariance.block( POSE_DIM, POSE_DIM, POSE_DIM, POSE_DIM );
			sample.targets[ name ].velocity = SampleVelocity( state.velocity, velCov );
		}
	}
	return sample;
}

PoseSE3 
RobotTargetDistribution::SamplePose( const PoseSE3& mean, 
                                     const PoseSE3::CovarianceMatrix& cov )
{
	gaussian.SetCovariance( cov );
	PoseSE3::TangentVector d = gaussian.Sample(2.0); // Truncates to 3 standard deviations
	return mean * PoseSE3::Exp( d );
}

PoseSE3::TangentVector
RobotTargetDistribution::SampleVelocity( const PoseSE3::TangentVector& mean,
                                         const PoseSE3::CovarianceMatrix& cov )
{
	gaussian.SetCovariance( cov );
	return mean + gaussian.Sample();
}

}
