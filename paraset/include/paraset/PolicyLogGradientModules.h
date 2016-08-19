#pragma once

#include "paraset/DiscretePolicyModules.h"
#include "paraset/ContinuousPolicyModules.h"

#include "percepto/compo/ProductWrapper.hpp"
#include "percepto/compo/ScaleWrapper.hpp"
#include "percepto/relearn/DiscreteLogProbability.hpp"
#include "percepto/relearn/GaussianLogProbability.hpp"

namespace argus
{

struct DiscreteLogGradientModule
{
	typedef percepto::Source<double> SourceType;
	typedef percepto::Sink<double> SinkType;

	DiscreteLogGradientModule( DiscretePolicyModule::Ptr net, 
	                           const VectorType& input,
	                           unsigned int actionIndex, 
	                           double actionAdvantage );

	percepto::TerminalSource<VectorType> networkInput;
	DiscretePolicyModule::Ptr network;
	percepto::DiscreteLogProbability logProb;
	percepto::ScaleWrapper<double> logExpectedAdvantage;

	void Foreprop();
	void Invalidate();
	SourceType* GetOutputSource();
};

struct ContinuousLogGradientModule
{
	typedef percepto::Source<double> SourceType;
	typedef percepto::Sink<double> SinkType;

	ContinuousLogGradientModule( ContinuousPolicyModule::Ptr net, 
	                             const VectorType& input,
	                             const VectorType& action, 
	                             double actionAdvantage );

	percepto::TerminalSource<VectorType> networkInput;
	ContinuousPolicyModule::Ptr network;
	percepto::GaussianLogProbability logProb;
	percepto::ScaleWrapper<double> logExpectedAdvantage;

	void Foreprop();
	void Invalidate();

	SourceType* GetOutputSource();

};

}