#include "paraset/TDErrorCritic.h"
#include "argus_utils/utils/ParamUtils.h"

#include "paraset/MonteCarloValue.h"
#include "paraset/ApproximateValue.h"

#include "paraset/RewardStamped.h"

namespace argus
{

TDErrorCritic::TDErrorCritic() {}

void TDErrorCritic::Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph )
{
	ros::NodeHandle rh( ph.resolveName( "reward_function" ) );
	_rewardFunction.Initialize( nh, rh );

	ros::NodeHandle vh( ph.resolveName( "value_function" ) );
	std::string valueType;
	GetParamRequired( vh, "type", valueType );
	if( valueType == "monte_carlo" )
	{
		_valueFunction = std::make_shared<MonteCarloValue>();
		_valueFunction->Initialize( nh, vh );
	}
	else if( valueType == "approximate" )
	{
		_valueFunction = std::make_shared<ApproximateValue>();
		_valueFunction->Initialize( nh, vh );
	}
	else
	{
		throw std::invalid_argument( "TDErrorCritic: Unknown value function type: " + valueType );
	}

	double dt, discountRate;
	GetParamRequired( ph, "timestep", dt );
	_timestep = ros::Duration( dt );

	if( GetParam( ph, "discount_rate", discountRate ) )
	{
		_discountFactor = std::exp( dt * std::log( discountRate ) );
		ROS_INFO_STREAM( "Computed discount factor of: " << _discountFactor << 
		                 " from desired rate: " << discountRate );
	}
	else
	{
		GetParamRequired( ph, "discount_factor", _discountFactor );
	}

	_estPub = ph.advertise<paraset::RewardStamped>( "output", 0 );
}

double TDErrorCritic::GetReward( const ros::Time& time ) const
{
	return _rewardFunction.IntegratedReward( time, time + _timestep );
}

double TDErrorCritic::Evaluate( const ParamAction& act ) const
{
	
	ros::Time currTime = act.time;
	ros::Time nextTime = currTime + _timestep;

	double reward = _rewardFunction.IntegratedReward( currTime, nextTime );
	double currValue = _valueFunction->Evaluate( ParamAction( currTime, VectorType() ) );
	double nextValue = _valueFunction->Evaluate( ParamAction( nextTime, VectorType() ) );

	double tdError = reward + _discountFactor * nextValue - currValue;
	return tdError;
}

ros::Duration TDErrorCritic::GetTimestep() const
{
	return _timestep;
}

}