#include "covreg/KalmanFilterModules.h"

namespace argus
{

KalmanFilterPredictModule::KalmanFilterPredictModule( percepto::Source<MatrixType>* Sprev,
	                                                  const VarReLUPosDefModule& q,
	                                                  const VectorType& input,
	                                                  const MatrixType& F )
: Q( q )
{
	qInput.SetOutput( input );
	Q.SetSource( &qInput );
	FSFT.SetSource( Sprev );
	FSFT.SetTransform( F );
	Sminus.SetSourceA( &Q );
	Sminus.SetSourceB( &FSFT );
}

percepto::Source<MatrixType>* 
KalmanFilterPredictModule::GetTailSource()
{
	return &Sminus;
}

// NOTE We do not invalidate Sprev because we can't foreprop it
void KalmanFilterPredictModule::Invalidate() 
{ 
	qInput.Invalidate(); 
}

void KalmanFilterPredictModule::Foreprop() 
{ 
	qInput.Foreprop();
}

std::ostream& operator<<( std::ostream& os, const KalmanFilterPredictModule& module )
{
	os << "Predict module: " << std::endl;
	os << "Q: " << std::endl << module.Q.GetOutput();
	return os;
}

KalmanFilterUpdateModule::KalmanFilterUpdateModule( percepto::Source<MatrixType>* sPrev,
	                                                const VarReLUPosDefModule& r,
	                                                const VectorType& input,
	                                                const MatrixType& H,
	                                                const VectorType& innovation )
: R( r ), Sprev( sPrev )
{
	rInput.SetOutput( input );
	R.SetSource( &rInput );
	HSHT.SetSource( Sprev );
	HSHT.SetTransform( H );
	V.SetSourceA( &HSHT );
	V.SetSourceB( &R );
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
	rInput.Invalidate(); 
}

void KalmanFilterUpdateModule::Foreprop() 
{ 
	rInput.Foreprop(); 
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