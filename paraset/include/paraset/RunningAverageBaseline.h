#pragma once

#include "paraset/ParasetInterfaces.h"
#include "paraset/RewardStamped.h"

#include <map>

namespace argus
{

class RunningAverageBaseline
: public PolicyCritic
{
public:

	RunningAverageBaseline();

	// TODO
	RunningAverageBaseline( PolicyCritic::Ptr source );

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	virtual void Publish( const ParamAction& act ) const;
	virtual double Evaluate( const ParamAction& act ) const;

private:

	double _acc;
	double _gamma;
	double _cacheTime;

	PolicyCritic::Ptr _source;
	ros::Duration _pollOffset;
	ros::Timer _pollTimer;

	bool _accInitialized;
	bool _delayStarted;
	ros::Time _startDelayTime;
	ros::Duration _initDelay;

	bool _burnInStarted;
	ros::Time _startBurnTime;
	ros::Time _lastBurnTime;
	ros::Duration _burnInDuration;

	ros::Publisher _outputPub;
	typedef std::map<ros::Time, double> CacheType;
	CacheType _cache;
	ros::Subscriber _valueSub;

	void Update( const ros::Time& time, double reward );
	void RewardCallback( const paraset::RewardStamped::ConstPtr& msg );
	double GetSpan() const;
	void TimerCallback( const ros::TimerEvent& event );

	bool IsBurnedIn() const;
};

}