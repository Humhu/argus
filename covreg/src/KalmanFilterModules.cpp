#include "covreg/ModuleDefinitions.h"

namespace argus
{

KalmanFilterPredictModule::KalmanFilterPredictModule( percepto::Source<MatrixType>* Sprev,
	                                                  const PositiveDefiniteModule& q,
	                                                  const VectorType& input,
	                                                  const MatrixType& F )
: Q( q )
{
	Q.dInput.SetOutput( input );
	FSFT.SetSource( Sprev );
	FSFT.SetTransform( F );
	Sminus.SetSourceA( Q.GetOutputSource() );
	Sminus.SetSourceB( &FSFT );
}

void KalmanFilterPredictModule::SetRootSource( percepto::Source<MatrixType>* Sprev )
{
	FSFT.SetSource( Sprev );
}

percepto::Source<MatrixType>* 
KalmanFilterPredictModule::GetTailSource()
{
	return &Sminus;
}

// NOTE We do not invalidate Sprev because we can't foreprop it
void KalmanFilterPredictModule::Invalidate() 
{ 
	Q.Invalidate(); 
}

void KalmanFilterPredictModule::Foreprop() 
{ 
	Q.Foreprop();
	// Q.GetOutput();
	// Sminus.GetOutput();
}

std::ostream& operator<<( std::ostream& os, const KalmanFilterPredictModule& module )
{
	os << "Predict module: " << std::endl;
	os << "Q: " << std::endl << module.Q.GetOutput();
	return os;
}

KalmanFilterUpdateModule::KalmanFilterUpdateModule( percepto::Source<MatrixType>* sPrev,
	                                                const PositiveDefiniteModule& r,
	                                                const VectorType& input,
	                                                const MatrixType& H,
	                                                const VectorType& innovation )
: R( r ), Sprev( sPrev )
{
	R.dInput.SetOutput( input );
	HSHT.SetSource( Sprev );
	HSHT.SetTransform( H );
	V.SetSourceA( &HSHT );
	V.SetSourceB( R.GetOutputSource() );
	//Vinv.SetSource( &V );
	HTVinvH.SetSource( &Vinv );
	HTVinvH.SetTransform( H.transpose() );
	// SHTVinvH.SetLeftSource( Sprev );
	SHTVinvH.SetRightSource( &HTVinvH );
	SHTVinvHS.SetLeftSource( &SHTVinvH );
	// SHTVinvHS.SetRightSource( Sprev );
	// Splus.SetPlusSource( Sprev );
	Splus.SetMinusSource( &SHTVinvHS );
	innovationLL.SetSource( &V );
	innovationLL.SetSample( innovation );
}

percepto::Source<MatrixType>* 
KalmanFilterUpdateModule::GetTailSource()
{
	// HACK
	Vinv.SetSource( &V );
	SHTVinvH.SetLeftSource( Sprev );
	SHTVinvHS.SetRightSource( Sprev );
	Splus.SetPlusSource( Sprev );
	return &Splus;
}

void KalmanFilterUpdateModule::Invalidate() 
{ 
	R.Invalidate(); 
}

void KalmanFilterUpdateModule::Foreprop() 
{ 
	R.Foreprop(); 
}

std::ostream& operator<<( std::ostream& os, const KalmanFilterUpdateModule& module )
{
	os << "Update module: " << std::endl;
	os << "R: " << std::endl << module.R.GetOutput() << std::endl;
	os << "V: " << std::endl << module.V.GetOutput() << std::endl;
	os << "inno: " << module.innovationLL.GetSample().transpose() << std::endl;
	os << "innoLL: " << module.innovationLL.GetOutput();
	return os;
}

}