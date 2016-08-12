#include "paraset/PolicyModules.h"

#define POSDEF_OFFSET_SCALE (1E-9)
#define RELU_LEAKY_SLOPE (1E-3)

using namespace percepto;

namespace argus
{

NormalizedPerceptron::NormalizedPerceptron( unsigned int inputDim,
                                            unsigned int outDim,
                                            unsigned int numHiddenLayers,
                                            unsigned int layerWidth )
: _net( inputDim, outDim, numHiddenLayers, layerWidth,
        percepto::SigmoidActivation(), percepto::PerceptronNet::OUTPUT_RECTIFIED ),
  _outputPort( this )
{
	_pmf.SetSource( &_net.GetOutputSource() );
	_pmf.RegisterConsumer( &_outputPort );
}

NormalizedPerceptron::NormalizedPerceptron( const NormalizedPerceptron& other )
: _net( other._net ), _outputPort( this )
{
	_pmf.SetSource( &_net.GetOutputSource() );
	_pmf.RegisterConsumer( &_outputPort );
}

Parameters::Ptr NormalizedPerceptron::CreateParameters()
{
	return _net.CreateParameters();
}

void NormalizedPerceptron::SetSource( InputSourceType* s )
{
	_net.SetSource( s );
}

NormalizedPerceptron::OutputSourceType&
NormalizedPerceptron::GetProbabilitySource()
{
	return _pmf;
}

void NormalizedPerceptron::SetOutputOffsets( const VectorType& probs )
{
	_net.SetOutputOffsets( probs );
}

void NormalizedPerceptron::Foreprop()
{
	OutputSourceType::SetOutput( _outputPort.GetInput() );
	OutputSourceType::Foreprop();
}

void NormalizedPerceptron::BackpropImplementation( const MatrixType& nextDodx )
{
	_outputPort.Backprop( nextDodx );
}

VarReLUGaussian::VarReLUGaussian( unsigned int inputDim, 
                                  unsigned int matDim,
                                  unsigned int numHiddenLayers,
                                  unsigned int layerWidth )
: reg( inputDim, matDim * 2, numHiddenLayers, layerWidth, 
       percepto::HingeActivation( 1.0, RELU_LEAKY_SLOPE ) ),
  lReg( matDim*(matDim-1)/2 )
{
	meanWrapper.SetSource( &reg.GetOutputSource() );
	meanWrapper.SetInds( 0, matDim );
	varWrapper.SetSource( &reg.GetOutputSource() );
	varWrapper.SetInds( matDim, matDim );

	expModule.SetSource( &varWrapper );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
	pdModule.SetOffset( POSDEF_OFFSET_SCALE * 
	                    MatrixType::Identity( matDim, matDim ) );
}

// Copy assignment rewires all connections and should result in
// shared parameters with the original
VarReLUGaussian::VarReLUGaussian( const VarReLUGaussian& other )
: reg( other.reg ),
  lReg( other.lReg ),
  meanWrapper( other.meanWrapper ),
  varWrapper( other.varWrapper ),  
  expModule( other.expModule ),
  pdModule( other.pdModule )
{
	meanWrapper.SetSource( &reg.GetOutputSource() );
	varWrapper.SetSource( &reg.GetOutputSource() );

	expModule.SetSource( &varWrapper );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
}

VarReLUGaussian::VectorSourceType&
VarReLUGaussian::GetMeanSource()
{
	return meanWrapper;
}

VarReLUGaussian::MatrixSourceType&
VarReLUGaussian::GetInfoSource()
{
	return pdModule;
}

void VarReLUGaussian::SetOutputOffsets( const VectorType& means,
                                        const VectorType& vars )
{
	VectorType offsets( means.size() + vars.size() );
	offsets.segment( 0, means.size() ) = means;
	offsets.segment( means.size(), vars.size() ) = vars;
	reg.SetOutputOffsets( offsets );
}

Parameters::Ptr VarReLUGaussian::CreateParameters()
{
	Parameters::Ptr lParams = lReg.CreateParameters();
	Parameters::Ptr dParams = reg.CreateParameters();
	
	ParameterWrapper::Ptr params = std::make_shared<ParameterWrapper>();
	params->AddParameters( lParams );
	params->AddParameters( dParams );
	return params;
}

void VarReLUGaussian::Foreprop()
{
	lReg.Foreprop();
}

void VarReLUGaussian::Invalidate()
{
	lReg.Invalidate();
}

std::ostream& operator<<( std::ostream& os, const VarReLUGaussian& vrlg )
{
	os << "Mean and variance regressor: " << std::endl << vrlg.reg << std::endl;
	os << "Correlation: " << std::endl << vrlg.psdModule.GetLOutput();
	return os;
}


}