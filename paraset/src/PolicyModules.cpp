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

Parameters::Ptr NormalizedPerceptron::CreateParameters()
{
	return _net.CreateParameters();
}

void NormalizedPerceptron::SetSource( InputSourceType* s )
{
	_net.SetSource( s );
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

PolicyModule::PolicyModule()
: _outputPort( this ) {}

PolicyModule::~PolicyModule() {}

void PolicyModule::SetOutputModule( OutputSourceType* outputModule )
{
	outputModule->RegisterConsumer( &_outputPort );
}

void PolicyModule::Foreprop()
{
	if( _outputPort.IsValid() )
	{
		OutputSourceType::SetOutput( _outputPort.GetInput() );
		OutputSourceType::Foreprop();
	}
}

void PolicyModule::BackpropImplementation( const MatrixType& nextDodx )
{
	_outputPort.Backprop( nextDodx );
}


VarReLUGaussian::VarReLUGaussian( unsigned int inputDim, 
                                  unsigned int matDim,
                                  unsigned int numHiddenLayers,
                                  unsigned int layerWidth )
: reg( inputDim, matDim * 2, numHiddenLayers, layerWidth, 
       percepto::HingeActivation( 1.0, RELU_LEAKY_SLOPE ) ),
  lReg( matDim*(matDim-1)/2 ),
  _inputPort( this )
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
	PolicyModule::SetOutputModule( &pdModule );
}

// Copy assignment rewires all connections and should result in
// shared parameters with the original
VarReLUGaussian::VarReLUGaussian( const VarReLUGaussian& other )
: reg( other.reg ),
  lReg( other.lReg ),
  meanWrapper( other.meanWrapper ),
  varWrapper( other.varWrapper ),  
  expModule( other.expModule ),
  pdModule( other.pdModule ),
  _inputPort( this )
{
	meanWrapper.SetSource( &reg.GetOutputSource() );
	varWrapper.SetSource( &reg.GetOutputSource() );

	expModule.SetSource( &varWrapper );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
	PolicyModule::SetOutputModule( &pdModule );
}

void VarReLUGaussian::SetSource( InputSourceType* s )
{
	// We splice in a fake input sink to allow us to manually trigger
	// the lReg foreprop
	// NOTE We have to send a fake backprop from the sink as a result
	s->RegisterConsumer( &_inputPort );
	reg.SetSource( s );
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

void VarReLUGaussian::Invalidate()
{
	// lReg will not get invalidated by dReg's input invalidation foreprop
	// We have to manually invalidate it and avoid infinite-looping here
	if( lReg.IsValid() ) { lReg.Invalidate(); }

	// Must call output invalidate here because lReg invalidate will always
	// follow dReg invalidate and thus will not propogate all the way to the
	// end again
	OutputSourceType::Invalidate();
}

void VarReLUGaussian::Foreprop()
{
	// Need to manually trigger lReg's foreprop since it has no input
	// The foreprop call from dReg's input will then trigger lReg
	// which will result in another call to this function. This second call
	// will continue past the output port
	if( _inputPort.IsValid() && !_outputPort.IsValid() ) { 
		lReg.Foreprop(); 
	}
	else { PolicyModule::Foreprop(); }
}

void VarReLUGaussian::BackpropImplementation( const MatrixType& nextDodx )
{
	// This first call will terminate at the input source b/c it is expecting
	// two sources. We have to send a fake backprop signal
	PolicyModule::BackpropImplementation( nextDodx );
	VectorType input = _inputPort.GetInput();
	_inputPort.Backprop( MatrixType::Zero( nextDodx.rows(), input.size() ) );
}

}