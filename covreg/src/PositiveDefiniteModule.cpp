#include "covreg/ModuleDefinitions.h"

#define POSDEF_OFFSET_SCALE (1E-9)

namespace argus
{

PositiveDefiniteModule::PositiveDefiniteModule( unsigned int inputDim, 
	                                            unsigned int matDim,
	                                            unsigned int numHiddenLayers,
	                                            unsigned int layerWidth )
: lReg( matDim*(matDim-1)/2 ),
  // dReg( inputDim, matDim, numHiddenLayers, layerWidth,
//          percepto::HingeActivation( 1.0, 1E-3 ) )
  dReg( matDim )
{
	// dReg.SetSource( &dInput );
	// expModule.SetSource( &dReg.GetOutputSource() );
	expModule.SetSource( &dReg );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
	pdModule.SetOffset( POSDEF_OFFSET_SCALE * 
	                    MatrixType::Identity( matDim, matDim ) );
}

// Copy assignment rewires all connections and should result in
// shared parameters with the original
PositiveDefiniteModule::PositiveDefiniteModule( const PositiveDefiniteModule& other )
: lReg( other.lReg ), 
  dReg( other.dReg ), 
  pdModule( other.pdModule )
{
	// dReg.SetSource( &dInput );
	// expModule.SetSource( &dReg.GetOutputSource() );
	expModule.SetSource( &dReg );
	psdModule.SetLSource( &lReg );
	psdModule.SetDSource( &expModule );
	pdModule.SetSource( &psdModule );
}

percepto::Source<MatrixType>* 
PositiveDefiniteModule::GetOutputSource()
{
	return &pdModule;
}

void PositiveDefiniteModule::Invalidate()
{
	// dInput.Invalidate();
	dReg.Invalidate();
	lReg.Invalidate();
}

void PositiveDefiniteModule::Foreprop()
{
	// dInput.Foreprop();
	dReg.Foreprop();
	lReg.Foreprop();
}

MatrixType PositiveDefiniteModule::GetOutput() const 
{ 
	return pdModule.GetOutput(); 
}

MatrixType PositiveDefiniteModule::Evaluate( const VectorType& in )
{
	dInput.SetOutput( in );
	Invalidate();
	Foreprop();
	return pdModule.GetOutput();
}

}