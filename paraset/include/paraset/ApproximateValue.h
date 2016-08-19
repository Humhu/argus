#pragma once

#include "paraset/ParasetInterfaces.h"
#include "paraset/ValueFunctionModules.h"

#include "broadcast/BroadcastMultiReceiver.h"

namespace argus
{

class ApproximateValue
: public PolicyCritic
{
public:

	typedef std::shared_ptr<ApproximateValue> Ptr;

	ApproximateValue();

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	virtual void Publish( const ParamAction& x ) const;
	virtual double Evaluate( const ParamAction& x ) const;

	ScalarFieldApproximator::Ptr GetApproximatorModule() const;

private:

	ros::Publisher _outputPub;

	std::string _moduleType;
	mutable percepto::TerminalSource<VectorType> _approximatorInput;
	mutable ScalarFieldApproximator::Ptr _approximator;

	BroadcastMultiReceiver _receiver;

};

}