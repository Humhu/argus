#pragma once

#include "paraset/ContinuousPolicy.h"
#include "paraset/PolicyModules.h"
#include "argus_msgs/FloatVectorStamped.h"
#include "broadcast/BroadcastMultiReceiver.h"

#include "argus_utils/random/MultivariateGaussian.hpp"

namespace argus
{

// TODO Subscribe to parameter updates
class ContinuousPolicyManager
{
public:

	struct DistributionParameters
	{
		VectorType mean;
		MatrixType info;
	};

	typedef VarReLUGaussian NetworkType;

	ContinuousPolicyManager();
	
	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	const NetworkType& GetPolicyModule() const;
	percepto::Parameters::Ptr GetParameters();

	StampedFeatures GetInput( const ros::Time& time );
	DistributionParameters GetDistributionParams( const ros::Time& time );
	void SetOutput( const VectorType& out );
	unsigned int GetNumOutputs() const;
	ContinuousPolicy& GetPolicyInterface();

private:

	ContinuousPolicy _policyInterface;
	ros::Subscriber _paramSub;

	BroadcastMultiReceiver _inputStreams;

	NetworkType::Ptr _network;
	percepto::TerminalSource<VectorType> _networkInput;
	percepto::Parameters::Ptr _networkParameters;

	void ParamCallback( const argus_msgs::FloatVectorStamped::ConstPtr& msg );
};

}