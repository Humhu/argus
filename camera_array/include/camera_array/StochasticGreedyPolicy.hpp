#pragma once

#include <boost/random/mersenne_twister.hpp>
#include <boost/random/random_device.hpp>

#include "camera_array/ModelInterfaces.h"
#include "argus_utils/RandomUtils.hpp"

namespace camera_array
{

template <typename State, typename Action>
class StochasticGreedyPolicy
: public DecisionPolicy<State, Action>
{
public:

	typedef typename DecisionPolicy<State,Action>::ActionList ActionList;
	
	StochasticGreedyPolicy( const typename RewardFunction<State,Action>::Ptr& r,
							double scale )
	: rewardFunc( r ), scaling( scale )
	{
		boost::random::random_device rng;
		generator.seed( rng );
	}
	
	virtual Action ChooseAction( const State& state, const ActionList& actions )
	{
		std::vector<double> weights( actions.size() );
		for( unsigned int i = 0; i < actions.size(); i++ )
		{
			weights[i] = std::exp( scaling * rewardFunc->CalculateReward( state, actions[i] ) ) - 1.0;
		}
		std::vector<unsigned int> inds;
		argus_utils::NaiveWeightedSample( weights, 1, inds, generator );
		
		return actions[ inds[0] ];
	}

private:
	
	typename RewardFunction<State,Action>::Ptr rewardFunc;
	
	double scaling;
	boost::random::mt19937 generator;
	
};

}
