#include "paraset/PolicyLogGradientModules.h"

namespace argus
{

DiscreteLogGradientModule::DiscreteLogGradientModule( const NetworkType& net,
                                                      const VectorType& input,
                                                      unsigned int actionIndex,
                                                      double actionAdvantage )
: network( net )
{
	networkInput.SetOutput( input );

	network.SetSource( &networkInput );

	logProb.SetSource( &network.GetProbabilitySource() );
	logProb.SetIndex( actionIndex );

	logExpectation.SetSource( &logProb );
	logExpectation.SetScale( actionAdvantage );
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
	return &logExpectation;
}

ContinuousLogGradientModule::ContinuousLogGradientModule( const NetworkType& net,
                                                          const VectorType& input,
                                                          const VectorType& action,
                                                          double actionAdvantage )
: network( net )
{
	networkInput.SetOutput( input );

	network.reg.SetSource( &networkInput );

	logProb.SetMeanSource( &network.GetMeanSource() );
	logProb.SetInfoSource( &network.GetInfoSource() );
	logProb.SetSample( action );

	logExpectation.SetSource( &logProb );
	logExpectation.SetScale( actionAdvantage );
}

void ContinuousLogGradientModule::Foreprop()
{
	networkInput.Foreprop();
	network.Foreprop();
}

void ContinuousLogGradientModule::Invalidate()
{
	networkInput.Invalidate();
	network.Invalidate();
}

ContinuousLogGradientModule::SourceType*
ContinuousLogGradientModule::GetOutputSource()
{
	return &logExpectation;
}

}