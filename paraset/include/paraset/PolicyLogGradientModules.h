#pragma once

#include "paraset/PolicyModules.h"
#include "paraset/DiscretePolicyManager.h" // TODO Move network types out of managers?
#include "paraset/ContinuousPolicyManager.h"

#include "percepto/compo/ScaleWrapper.hpp"
#include "percepto/relearn/DiscreteLogProbability.hpp"
#include "percepto/relearn/GaussianLogProbability.hpp"

namespace argus
{

struct DiscreteLogGradientModule
{
	typedef percepto::Source<double> SourceType;
	typedef percepto::Sink<double> SinkType;
	typedef DiscretePolicyManager::NetworkType NetworkType;

	DiscreteLogGradientModule( const NetworkType& net, 
	                           const VectorType& input,
	                           unsigned int actionIndex, 
	                           double actionAdvantage );

	percepto::TerminalSource<VectorType> networkInput;
	NetworkType network;
	percepto::DiscreteLogProbability logProb;
	percepto::ScaleWrapper<double> logExpectation;

	void Foreprop();
	void Invalidate();
	SourceType* GetOutputSource();
};

struct ContinuousLogGradientModule
{
	typedef percepto::Source<double> SourceType;
	typedef percepto::Sink<double> SinkType;
	typedef ContinuousPolicyManager::NetworkType NetworkType;

	ContinuousLogGradientModule( const NetworkType& net, 
	                             const VectorType& input,
	                             const VectorType& action, 
	                             double actionAdvantage );

	percepto::TerminalSource<VectorType> networkInput;
	NetworkType network;
	percepto::GaussianLogProbability logProb;
	percepto::ScaleWrapper<double> logExpectation;

	void Foreprop();
	void Invalidate();
	SourceType* GetOutputSource();
};

}