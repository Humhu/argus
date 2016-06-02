#pragma once

#include "camera_array/DistributionInterfaces.h"
#include "camera_array/SystemStates.h"

#include "argus_utils/random/MultivariateGaussian.hpp"

namespace argus
{

class RobotTargetDistribution
: public Distribution<RobotTargetState>
{
public:

	struct Properties
	{
		bool sampleTargetPoses;
		bool sampleTargetVelocities;
		bool sampleRobotPose;
		bool sampleRobotVelocity;
		
		Properties( bool targetPose = false, bool targetVel = false,
		            bool robotPose = false, bool robotVel = false );
	};
	
	RobotTargetDistribution( Properties props = Properties() );
	
	void SetMean( const RobotTargetState& b );
	
	virtual RobotTargetState Sample();
	
private:
	
	RobotTargetState base;
	Properties properties;
	MultivariateGaussian<> gaussian; // Default mean zero, identity cov
	
	PoseSE3 SamplePose( const PoseSE3& mean, 
	                    const PoseSE3::CovarianceMatrix& cov );
	
	PoseSE3::TangentVector 
	SampleVelocity( const PoseSE3::TangentVector& mean, 
	                const PoseSE3::CovarianceMatrix& cov );
	
};

}
