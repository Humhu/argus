#pragma once

#include "paraset/ParasetInterfaces.h"
#include "paraset/ValueFunctionModules.h"

#include "broadcast/BroadcastMultiReceiver.h"

#include "argus_msgs/FloatVectorStamped.h"

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

	ScalarFieldApproximator::Ptr CreateApproximatorModule() const;

	VectorType GetInput( const ros::Time& time ) const;
	percepto::Parameters::Ptr GetParameters() const;

private:

	ros::Publisher _outputPub;
	ros::Subscriber _paramsSub;

	std::string _moduleType;
	mutable percepto::TerminalSource<VectorType> _approximatorInput;
	mutable ScalarFieldApproximator::Ptr _approximator;
	percepto::Parameters::Ptr _approximatorParams;

	BroadcastMultiReceiver _receiver;

	void ParamsCallback( const argus_msgs::FloatVectorStamped::ConstPtr& msg );

};

}