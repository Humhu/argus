#include "camera_array/ExpectationPolicy.h"

#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <boost/random/random_device.hpp>

namespace argus
{

ExpectationPolicy::ExpectationPolicy( const RobotArrayReward::Ptr& r,
								      unsigned int N )
: reward( r ), numSamples( N )
{
	RobotTargetDistribution::Properties props;
	props.sampleTargetPoses = true;
	distribution = std::make_shared<RobotTargetDistribution>( props );
	
	boost::random::random_device rng;
	generator.seed( rng );
}	

CameraArrayAction ExpectationPolicy::ChooseAction( const RobotTargetState& state,
												   const ActionList& actions )
{
	distribution->SetMean( state );
	std::vector<double> rewards( actions.size(), 0.0 );
	
// 	BOOST_FOREACH( const RobotTargetState::TargetMap::value_type& item, state.targets )
// 	{
// 		ROS_INFO_STREAM( "Mean Target: " << item.first << " pose: " << item.second.pose );
// 		ROS_INFO_STREAM( "cov: " << item.second.poseCovariance );
// 	}
	
	for( unsigned int i = 0; i < numSamples; i++ )
	{
		RobotTargetState sample = distribution->Sample();
		
// 		ROS_INFO_STREAM( "Sampling " << i );
// 		BOOST_FOREACH( const RobotTargetState::TargetMap::value_type& item, sample.targets )
// 		{
// 			ROS_INFO_STREAM( "Target: " << item.first << " pose: " << item.second.pose );
// 		}
		for( unsigned int j = 0; j < actions.size(); j++ )
		{
			rewards[j] += reward->CalculateReward( sample, actions[j] );
		}
	}
	
	double maxSeen = -std::numeric_limits<double>::infinity();
	std::vector<CameraArrayAction> ret;
	for( unsigned int j = 0; j < actions.size(); j++ )
	{
		ROS_INFO_STREAM( "Reward for " << actions[j] << " is " << rewards[j] );
		if( rewards[j] < maxSeen ) { continue; }
		ret.push_back( actions[j] );
		if( rewards[j] == maxSeen ) { continue; }
		maxSeen = rewards[j];
	}
	
	boost::random::uniform_int_distribution<> dist( 0, ret.size() - 1 );
	
	return ret[ dist( generator ) ];
}

}
