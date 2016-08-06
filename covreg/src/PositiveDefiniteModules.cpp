#include "covreg/PositiveDefiniteModules.h"

#define POSDEF_OFFSET_SCALE (1E-9)
#define RELU_LEAKY_SLOPE (1E-3)

using namespace percepto;

namespace argus
{

PosDefModule::PosDefModule()
: _outputPort( this ) {}

PosDefModule::~PosDefModule() {}

void PosDefModule::SetOutputModule( OutputSourceType* outputModule )
{
	outputModule->RegisterConsumer( &_outputPort );
}

void PosDefModule::Foreprop()
{
	if( _outputPort.IsValid() )
	{
		OutputSourceType::SetOutput( _outputPort.GetInput() );
		OutputSourceType::Foreprop();
	}
}

void PosDefModule::BackpropImplementation( const MatrixType& nextDodx )
{
	_outputPort.Backprop( nextDodx );
}

ConstantPosDefModule::ConstantPosDefModule( unsigned int matDim )
: lReg( matDim*(matDim-1)/2 ),
  dReg( matDim ),
  // expModule( EXP_LOWER_THRESH, EXP_UPPER_THRESH ),
  _inputPort( this )
{
	expModule.SetSource( &dReg );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
	PosDefModule::SetOutputModule( &pdModule );
	pdModule.SetOffset( POSDEF_OFFSET_SCALE * 
	                    MatrixType::Identity( matDim, matDim ) );
}

// Copy assignment rewires all connections and should result in
// shared parameters with the original
ConstantPosDefModule::ConstantPosDefModule( const ConstantPosDefModule& other )
: lReg( other.lReg ), 
  dReg( other.dReg ), 
  expModule( other.expModule ),
  pdModule( other.pdModule ),
  _inputPort( this )
{
	expModule.SetSource( &dReg );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
	PosDefModule::SetOutputModule( &pdModule );
}

ParameterWrapper::Ptr ConstantPosDefModule::CreateParameters()
{
	Parameters::Ptr lParams = lReg.CreateParameters();
	Parameters::Ptr dParams = dReg.CreateParameters();
	ParameterWrapper::Ptr params = std::make_shared<ParameterWrapper>();
	params->AddParameters( lParams );
	params->AddParameters( dParams );
	return params;
}

void ConstantPosDefModule::SetSource( InputSourceType* s )
{
	s->RegisterConsumer( &_inputPort );
}

void ConstantPosDefModule::Invalidate()
{
	if( lReg.IsValid() && dReg.IsValid() ) 
	{
		lReg.Invalidate();
		dReg.Invalidate();
	}
	OutputSourceType::Invalidate();
}

void ConstantPosDefModule::Foreprop()
{
	if( _inputPort.IsValid() && !_outputPort.IsValid() )
	{
		lReg.Foreprop();
		dReg.Foreprop();
	}
	else { PosDefModule::Foreprop(); }
}

LinearPosDefModule::LinearPosDefModule( unsigned int inputDim, 
	                                      unsigned int matDim )
: lReg( matDim*(matDim-1)/2 ),
  dReg( inputDim, matDim ),
  // expModule( EXP_LOWER_THRESH, EXP_UPPER_THRESH ),
  _inputPort( this )
{
	expModule.SetSource( &dReg );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
	pdModule.SetOffset( POSDEF_OFFSET_SCALE * 
	                    MatrixType::Identity( matDim, matDim ) );
	PosDefModule::SetOutputModule( &pdModule );
}

// Copy assignment rewires all connections and should result in
// shared parameters with the original
LinearPosDefModule::LinearPosDefModule( const LinearPosDefModule& other )
: lReg( other.lReg ), 
  dReg( other.dReg ), 
  expModule( other.expModule ),
  pdModule( other.pdModule ),
  _inputPort( this )
{
	expModule.SetSource( &dReg );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
	PosDefModule::SetOutputModule( &pdModule );
}

void LinearPosDefModule::SetSource( InputSourceType* s )
{
	// We splice in a fake input sink to allow us to manually trigger
	// the lReg foreprop
	// NOTE We have to send a fake backprop from the sink as a result
	s->RegisterConsumer( &_inputPort );
	dReg.SetSource( s );
}

ParameterWrapper::Ptr LinearPosDefModule::CreateParameters()
{
	Parameters::Ptr lParams = lReg.CreateParameters();
	Parameters::Ptr dParams = dReg.CreateParameters();
	
	ParameterWrapper::Ptr params = std::make_shared<ParameterWrapper>();
	params->AddParameters( lParams );
	params->AddParameters( dParams );
	return params;
}

void LinearPosDefModule::Invalidate()
{
	// lReg will not get invalidated by dReg's input invalidation foreprop
	// We have to manually invalidate it and avoid infinite-looping here
	if( lReg.IsValid() ) { lReg.Invalidate(); }

	// Must call output invalidate here because lReg invalidate will always
	// follow dReg invalidate and thus will not propogate all the way to the
	// end again
	OutputSourceType::Invalidate();
}

void LinearPosDefModule::Foreprop()
{
	// Need to manually trigger lReg's foreprop since it has no input
	// The foreprop call from dReg's input will then trigger lReg
	// which will result in another call to this function. This second call
	// will continue past the output port
	if( _inputPort.IsValid() && !_outputPort.IsValid() ) { 
		lReg.Foreprop(); 
	}
	else { PosDefModule::Foreprop(); }
}

void LinearPosDefModule::BackpropImplementation( const MatrixType& nextDodx )
{
	// This first call will terminate at the input source b/c it is expecting
	// two sources. We have to send a fake backprop signal
	PosDefModule::BackpropImplementation( nextDodx );
	VectorType input = _inputPort.GetInput();
	_inputPort.Backprop( MatrixType::Zero( nextDodx.rows(), input.size() ) );
}

VarReLUPosDefModule::VarReLUPosDefModule( unsigned int inputDim, 
	                                      unsigned int matDim,
	                                      unsigned int numHiddenLayers,
	                                      unsigned int layerWidth )
: lReg( matDim*(matDim-1)/2 ),
  dReg( inputDim, matDim, numHiddenLayers, layerWidth,
  percepto::HingeActivation( 1.0, RELU_LEAKY_SLOPE ) ),
  // percepto::SigmoidActivation() ),
  // expModule( EXP_LOWER_THRESH, EXP_UPPER_THRESH ),
  _inputPort( this )
{
	expModule.SetSource( &dReg.GetOutputSource() );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
	pdModule.SetOffset( POSDEF_OFFSET_SCALE * 
	                    MatrixType::Identity( matDim, matDim ) );
	PosDefModule::SetOutputModule( &pdModule );
}

// Copy assignment rewires all connections and should result in
// shared parameters with the original
VarReLUPosDefModule::VarReLUPosDefModule( const VarReLUPosDefModule& other )
: lReg( other.lReg ), 
  dReg( other.dReg ), 
  expModule( other.expModule ),
  pdModule( other.pdModule ),
  _inputPort( this )
{
	expModule.SetSource( &dReg.GetOutputSource() );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
	PosDefModule::SetOutputModule( &pdModule );
}

void VarReLUPosDefModule::SetSource( InputSourceType* s )
{
	// We splice in a fake input sink to allow us to manually trigger
	// the lReg foreprop
	// NOTE We have to send a fake backprop from the sink as a result
	s->RegisterConsumer( &_inputPort );
	dReg.SetSource( s );
}

ParameterWrapper::Ptr VarReLUPosDefModule::CreateParameters()
{
	Parameters::Ptr lParams = lReg.CreateParameters();
	Parameters::Ptr dParams = dReg.CreateParameters();
	
	ParameterWrapper::Ptr params = std::make_shared<ParameterWrapper>();
	params->AddParameters( lParams );
	params->AddParameters( dParams );
	return params;
}

void VarReLUPosDefModule::Invalidate()
{
	// lReg will not get invalidated by dReg's input invalidation foreprop
	// We have to manually invalidate it and avoid infinite-looping here
	if( lReg.IsValid() ) { lReg.Invalidate(); }

	// Must call output invalidate here because lReg invalidate will always
	// follow dReg invalidate and thus will not propogate all the way to the
	// end again
	OutputSourceType::Invalidate();
}

void VarReLUPosDefModule::Foreprop()
{
	// Need to manually trigger lReg's foreprop since it has no input
	// The foreprop call from dReg's input will then trigger lReg
	// which will result in another call to this function. This second call
	// will continue past the output port
	if( _inputPort.IsValid() && !_outputPort.IsValid() ) { 
		lReg.Foreprop(); 
	}
	else { PosDefModule::Foreprop(); }
}

void VarReLUPosDefModule::BackpropImplementation( const MatrixType& nextDodx )
{
	// This first call will terminate at the input source b/c it is expecting
	// two sources. We have to send a fake backprop signal
	PosDefModule::BackpropImplementation( nextDodx );
	VectorType input = _inputPort.GetInput();
	_inputPort.Backprop( MatrixType::Zero( nextDodx.rows(), input.size() ) );
}

}