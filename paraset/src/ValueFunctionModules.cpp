#include "paraset/ValueFunctionModules.h"
#include <sstream>

namespace argus
{

PerceptronFunctionApproximator::PerceptronFunctionApproximator( unsigned int inputDim,
                                                                unsigned int outputDim,
                                                                unsigned int numHiddenLayers,
                                                                unsigned int layerWidth )
: network( inputDim,
           outputDim,
           numHiddenLayers,
           layerWidth,
           percepto::SigmoidActivation(),
           percepto::PerceptronNet::OUTPUT_UNRECTIFIED ) 
{
	output.SetIndex( 0 );
	output.SetSource( &network.GetOutputSource() );
}

PerceptronFunctionApproximator::PerceptronFunctionApproximator( const PerceptronFunctionApproximator& other )
: network( other.network ),
  output( other.output )
{
	output.SetSource( &network.GetOutputSource() );
}

void PerceptronFunctionApproximator::SetInputSource( InputSourceType* src )
{
	network.SetSource( src );
}

PerceptronFunctionApproximator::OutputSourceType&
PerceptronFunctionApproximator::GetOutputSource()
{
	return output;
}

void PerceptronFunctionApproximator::Foreprop() {}

void PerceptronFunctionApproximator::Invalidate() {}

percepto::Parameters::Ptr PerceptronFunctionApproximator::CreateParameters()
{
	return network.CreateParameters();
}

void PerceptronFunctionApproximator::InitializeOutput( const double& o )
{
	VectorType out( 1 );
	out( 0 ) = o;
	network.SetOutputOffsets( out );
}

std::string PerceptronFunctionApproximator::Print() const
{
	std::stringstream ss;
	ss << network;
	return ss.str();
}

}
