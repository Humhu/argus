#pragma once

#include <ros/ros.h>
#include <memory>

#include "paraset/ParasetCommon.h"

namespace argus
{

class PolicyCritic
{
public:

	typedef std::shared_ptr<PolicyCritic> Ptr;

	PolicyCritic() {}
	virtual ~PolicyCritic() {}

	virtual double Evaluate( const ParamAction& act ) const = 0;
	virtual void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph ) = 0;

};

}