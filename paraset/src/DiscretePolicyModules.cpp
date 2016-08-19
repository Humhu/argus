#include "paraset/DiscretePolicyModules.h"

namespace argus
{

DiscretePolicyModule::DiscretePolicyModule() {}

DiscretePolicyModule::~DiscretePolicyModule() {}

NormalizedPerceptron::NormalizedPerceptron( unsigned int inputDim,
                                            unsigned int outDim,
                                            unsigned int numHiddenLayers,
                                            unsigned int layerWidth )
: factors( inputDim, 
           outDim, 
           numHiddenLayers, 
           layerWidth,
           percepto::SigmoidActivation(), 
           percepto::PerceptronNet::OUTPUT_RECTIFIED )
{
	probabilities.SetSource( &factors.GetOutputSource() );
}

NormalizedPerceptron::NormalizedPerceptron( const NormalizedPerceptron& other )
: factors( other.factors )
{
	probabilities.SetSource( &factors.GetOutputSource() );
}

void NormalizedPerceptron::SetInputSource( VectorSourceType* src )
{
	factors.SetSource( src );
}

NormalizedPerceptron::VectorSourceType&
NormalizedPerceptron::GetProbabilitySource()
{
	return probabilities;
}

void NormalizedPerceptron::Foreprop()
{}

void NormalizedPerceptron::Invalidate()
{}

percepto::Parameters::Ptr NormalizedPerceptron::CreateParameters()
{
	return factors.CreateParameters();
}

void NormalizedPerceptron::InitializeProbabilities( const VectorType& probs )
{
	factors.SetOutputOffsets( probs );
}

}