#pragma once

#include "camera_array/SystemModels.h"
#include "camera_array/SystemStates.h"
#include "camera_array/SystemDistributions.h"
#include "argus_utils/RandomUtils.hpp"
#include <boost/random/mersenne_twister.hpp>

namespace camera_array
{

class ExpectationPolicy
: public RobotArrayPolicy
{
public:
	
	ExpectationPolicy( const RobotArrayReward::Ptr& r,
					   unsigned int N );
	
	virtual CameraArrayAction ChooseAction( const RobotTargetState& state,
											const ActionList& actions );
									
private:
	
	
	RobotArrayReward::Ptr reward;
	unsigned int numSamples;
	RobotTargetDistribution::Ptr distribution;
	boost::random::mt19937 generator;

	
};

}
