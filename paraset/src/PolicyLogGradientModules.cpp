#include "paraset/PolicyLogGradientModules.h"

namespace argus
{

DiscreteLogGradientModule::DiscreteLogGradientModule( DiscretePolicyModule::Ptr net,
                                                      const VectorType& input,
                                                      unsigned int actionIndex,
                                                      double actionAdvantage )
: network( net )
{
	networkInput.SetOutput( input );

	network->SetInputSource( &networkInput );

	logProb.SetSource( &network->GetProbabilitySource() );
	logProb.SetIndex( actionIndex );

	logExpectedAdvantage.SetSource( &logProb );
	logExpectedAdvantage.SetScale( actionAdvantage );
}

void DiscreteLogGradientModule::Foreprop()
{
	networkInput.Foreprop();
}

void DiscreteLogGradientModule::Invalidate()
{
	networkInput.Invalidate();
}

DiscreteLogGradientModule::SourceType*
DiscreteLogGradientModule::GetOutputSource()
{
	return &logExpectedAdvantage;
}

ContinuousLogGradientModule::ContinuousLogGradientModule( ContinuousPolicyModule::Ptr net,
                                                          const VectorType& input,
                                                          const VectorType& action,
                                                          double actionAdvantage )
: network( net )
{
	networkInput.SetOutput( input );
	network->SetInputSource( &networkInput );

	logProb.SetMeanSource( &network->GetMeanSource() );
	logProb.SetInfoSource( &network->GetInfoSource() );
	logProb.SetSample( action );
	// logProb.modName = "logprob";

	logExpectedAdvantage.SetSource( &logProb );
	logExpectedAdvantage.SetScale( actionAdvantage );
}

void ContinuousLogGradientModule::Foreprop()
{
	networkInput.Foreprop();
	network->Foreprop();
}

void ContinuousLogGradientModule::Invalidate()
{
	networkInput.Invalidate();
	network->Invalidate();
}

ContinuousLogGradientModule::SourceType*
ContinuousLogGradientModule::GetOutputSource()
{
	return &logExpectedAdvantage;
}

}