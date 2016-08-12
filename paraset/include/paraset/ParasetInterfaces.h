#pragma once

#include <ros/ros.h>
#include <memory>

#include "paraset/ParasetCommon.h"

namespace argus
{

// Base interface for all functions that map from time to scalars
template <typename In>
class MappingFunction
{
public:

	typedef In InputType;
	typedef std::shared_ptr<MappingFunction> Ptr;

	MappingFunction() {}
	virtual ~MappingFunction() {}

	virtual double Evaluate( const InputType& input ) const = 0;

};

class PolicyCritic
: public MappingFunction<ParamAction>
{
public:

	typedef std::shared_ptr<PolicyCritic> Ptr;

	PolicyCritic() {}
	virtual ~PolicyCritic() {}

	// Publish messages for the input
	virtual void Publish( const ParamAction& act ) const = 0;

	virtual void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph ) = 0;

};

typedef MappingFunction<ros::Time> TimeFunction;
typedef MappingFunction<VectorType> VectorFunction;

}