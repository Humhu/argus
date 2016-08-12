#pragma once

#include "paraset/ParasetInterfaces.h"
#include "paraset/RewardStamped.h"
#include <map>

namespace argus
{

enum InterpolationMode
{
	INTERP_ZERO_ORDER_HOLD,
	INTERP_PIECEWISE_LINEAR
};

InterpolationMode StringToInterpMode( const std::string& str );

class RewardInterpolater
{
public:

	typedef std::shared_ptr<RewardInterpolater> Ptr;

	RewardInterpolater();

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	double Evaluate( const ros::Time& t ) const;

private:

	typedef std::map<ros::Time, double> RewardSeries;

	RewardSeries _rewards;

	double _cacheTime;
	InterpolationMode _interpMode;
	ros::Subscriber _rewardSub;

	void RewardCallback( const paraset::RewardStamped::ConstPtr& msg );
	double ComputeSpan() const;

	double EvaluateZOH( const ros::Time& t ) const;
	double EvaluatePWL( const ros::Time& t ) const;
};

}