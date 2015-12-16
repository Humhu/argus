#include "camera_array/SystemDistributions.h"
#include <boost/foreach.hpp>

using namespace argus_utils;

namespace camera_array
{

RobotTargetDistribution::Properties::Properties( bool targetPose, bool targetVel,
                                                 bool robotPose, bool robotVel )
: sampleTargetPoses( targetPose ), sampleTargetVelocities( targetVel ),
  sampleRobotPose( robotPose ), sampleRobotVelocity( robotVel ) {}

RobotTargetDistribution::RobotTargetDistribution( Properties props )
: properties( props )
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
		sample.robot.pose = SamplePose( base.robot.pose, 
		                                base.robot.poseCovariance );
	}
	if( properties.sampleRobotVelocity )
	{
		sample.robot.velocity = SampleVelocity( base.robot.velocity,
		                                        base.robot.velocityCovariance );
	}
	if( properties.sampleTargetPoses )
	{
		BOOST_FOREACH( const RobotTargetState::TargetMap::value_type& item, base.targets )
		{
			const std::string& name = item.first;
			const TargetState& state = item.second;
			sample.targets[ name ].pose = SamplePose( state.pose, 
			                                          state.poseCovariance );
		}
	}
	if( properties.sampleTargetVelocities )
	{
		BOOST_FOREACH( const RobotTargetState::TargetMap::value_type& item, base.targets )
		{
			const std::string& name = item.first;
			const TargetState& state = item.second;
			sample.targets[ name ].velocity = SampleVelocity( state.velocity, 
			                                                  state.velocityCovariance );
		}
	}
	return sample;
}

PoseSE3 
RobotTargetDistribution::SamplePose( const PoseSE3& mean, 
                                     const PoseSE3::CovarianceMatrix& cov )
{
	gaussian.SetCovariance( cov );
	PoseSE3::TangentVector d = gaussian.Sample(); // Truncates to 3 standard deviations
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
