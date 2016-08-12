#pragma once

#include "paraset/ParasetInterfaces.h"

#include <unordered_map>

namespace argus
{

class DifferenceCritic
: public PolicyCritic
{
public:

	typedef std::shared_ptr<DifferenceCritic> Ptr;

	DifferenceCritic();

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	virtual void Publish( const ParamAction& act ) const;

	virtual double Evaluate( const ParamAction& act ) const;

private:

	PolicyCritic::Ptr _valueFunction;
	PolicyCritic::Ptr _baselineFunction;
};

}