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
		double maxReward = -std::numeric_limits<double>::infinity();
		double minReward = std::numeric_limits<double>::infinity();
		for( unsigned int i = 0; i < actions.size(); i++ )
		{
			weights[i] = rewardFunc->CalculateReward( state, actions[i] );
			if( weights[i] < minReward ) { minReward = weights[i]; }
			if( weights[i] > maxReward ) { maxReward = weights[i]; }
		}

		double rewardRange = maxReward - minReward;
		if( rewardRange < 1E-6 ) { rewardRange = 1.0; }
		double k = scaling / ( rewardRange );
		// Normalize the exponents by making sure the min raw weight is 0 and the max weight is 1
		for( unsigned int i = 0; i < actions.size(); i++ )
		  {
		    weights[i] = std::exp( k * ( weights[i] - minReward ) );
			std::cout << "Action: " << actions[i] << " weight: " << weights[i] << std::endl;
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
