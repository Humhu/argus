#pragma once

#include "camera_array/ModelInterfaces.h"
#include "camera_array/DistributionInterfaces.h"

namespace camera_array
{

// TODO Other sampling techniques
	
/*! \brief Naive sampling for integral estimation. */
template <typename State, typename Action>
class ExpectedRewardFunction
: public RewardFunction <State, Action>
{
public:

	ExpectedRewardFunction( const typename Distribution<State>::Ptr& dis,
	                        const typename RewardFunction<State,Action>::Ptr& base,
	                        unsigned int N = 10 )
	: baseReward( base ), numSamples( N )
	{}
	
	virtual double CalculateReward( const State& state, const Action& action )
	{
		distribution->SetMean( state );
		double sum = 0;
		for( unsigned int i = 0; i < numSamples; i++ )
		{
			State sample = distribution->Sample();
			sum += baseReward->CalculateReward( sample, action );
		}
		return sum / numSamples;
	}
	
private:
	
	typename Distribution<State>::Ptr distribution;
	typename RewardFunction<State,Action>::Ptr baseReward;
	
	unsigned int numSamples;
	
};
	
} // end namespace camera_array
