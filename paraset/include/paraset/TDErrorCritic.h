#pragma once

#include "paraset/ParasetInterfaces.h"
#include "paraset/RewardInterpolater.h"

namespace argus
{

class TDErrorCritic
: public PolicyCritic
{
public:

	typedef std::shared_ptr<TDErrorCritic> Ptr;

	TDErrorCritic();

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	double GetReward( const ros::Time& time ) const;
	virtual void Publish( const ParamAction& act ) const;
	virtual double Evaluate( const ParamAction& act ) const;

private:

	ros::Publisher _estPub;

	RewardInterpolater _rewardFunction;
	PolicyCritic::Ptr _valueFunction;
	
	double _discountFactor;
	ros::Duration _timestep;
};

}