#include "covreg/KalmanFilterModules.h"

namespace argus
{

KalmanFilterPredictModule::KalmanFilterPredictModule( percepto::Source<MatrixType>* sprev,
	                                                  const CovarianceEstimator::ModuleType& q,
	                                                  double dt,
	                                                  const VectorType& input,
	                                                  const MatrixType& F )
: Q( q ), Sprev( sprev )
{
	qInput.SetOutput( input );
	Q.SetSource( &qInput );
	Qdt.SetSource( &Q );
	Qdt.SetScale( dt );
	FSFT.SetSource( Sprev );
	FSFT.SetTransform( F );
	Sminus.SetSourceA( &Qdt );
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
	os << "Sprev: " << std::endl << module.Sprev->GetOutput() << std::endl;
	os << "Q: " << std::endl << module.Q.GetOutput() << std::endl;
	os << "Sminus: " << std::endl << module.Sminus.GetOutput() << std::endl;
	return os;
}

KalmanFilterUpdateModule::KalmanFilterUpdateModule( percepto::Source<MatrixType>* sPrev,
	                                                const CovarianceEstimator::ModuleType& r,
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
	os << "Update module: " << module.sourceName << std::endl;
	os << "Sprev: " << std::endl << module.Sprev->GetOutput() << std::endl;
	os << "HSHT: " << std::endl << module.HSHT.GetOutput() << std::endl;
	os << "R: " << std::endl << module.R.GetOutput() << std::endl;
	os << "V: " << std::endl << module.V.GetOutput() << std::endl;
	os << "Splus: " << std::endl << module.Splus.GetOutput() << std::endl;
	os << "inno: " << module.innovationLL.GetSample().transpose() << std::endl;
	os << "innoLL: " << module.innovationLL.GetOutput() << std::endl;
	return os;
}

}