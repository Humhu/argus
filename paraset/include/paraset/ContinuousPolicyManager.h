#pragma once

#include "paraset/ContinuousPolicy.h"
#include "paraset/ContinuousPolicyModules.h"
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

	ContinuousPolicyManager();
	
	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	ContinuousPolicyModule::Ptr GetPolicyModule() const;

	percepto::Parameters::Ptr GetParameters();
	percepto::Parameters::Ptr GetMeanParameters();
	percepto::Parameters::Ptr GetCovParameters();

	const Eigen::ArrayXd& GetScales() const;
	const VectorType& GetOffsets() const;

	StampedFeatures GetInput( const ros::Time& time );

	// NOTE Returns normalized distribution parameters
	DistributionParameters GetDistributionParams( const ros::Time& time );

	void SetOutput( const VectorType& out );
	unsigned int GetNumOutputs() const;
	ContinuousPolicy& GetPolicyInterface();

private:

	ContinuousPolicy _policyInterface;
	ros::Subscriber _paramSub;

	Eigen::ArrayXd _policyScales;
	VectorType _policyOffsets;

	BroadcastMultiReceiver _inputStreams;

	std::string _moduleType;
	ContinuousPolicyModule::Ptr _network;

	percepto::TerminalSource<VectorType> _networkInput;
	percepto::Parameters::Ptr _networkParameters;

	void ParamCallback( const argus_msgs::FloatVectorStamped::ConstPtr& msg );
};

}