#pragma once

namespace argus
{

// Based on work by Schulman et. al. (2015)
class GeneralizedAdvantageCritic
{
public:

	typedef std::shared_ptr<GeneralizedAdvantageCritic> Ptr;

	GeneralizedAdvantageCritic();

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	double GetReward( const ros::Time& time ) const;
	virtual void Publish( const ParamAction& act ) const;
	virtual double Evaluate( const ParamAction& act ) const;

private:

	TDErrorCritic _tdErrorCritic;
	double lambda;
	ros::Duration 

};

}