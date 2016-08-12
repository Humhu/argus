#pragma once

#include "paraset/ContinuousParamAction.h"
#include "paraset/DiscreteParamAction.h"

#include "argus_utils/utils/LinalgTypes.h"

namespace argus
{

struct ParamAction
{
	ros::Time time;
	VectorType input;

	ParamAction();
	ParamAction( const ros::Time& t,
	             const VectorType& in );
};

struct ContinuousParamAction
: public ParamAction
{
	typedef paraset::ContinuousParamAction MsgType;
	VectorType output;

	ContinuousParamAction();
	ContinuousParamAction( const ros::Time& t, 
	                       const VectorType& in,
	                       const VectorType& out );
	ContinuousParamAction( const MsgType& msg );
	MsgType ToMsg() const;
};

struct DiscreteParamAction
: public ParamAction
{
	typedef paraset::DiscreteParamAction MsgType;

	unsigned int index;

	DiscreteParamAction();
	DiscreteParamAction( const ros::Time& t,
	                     const VectorType& in,
	                     unsigned int ind );
	DiscreteParamAction( const MsgType& msg );
	MsgType ToMsg() const;
};

}