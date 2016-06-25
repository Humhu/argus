#include "covreg/KalmanFilterModules.h"

namespace argus
{

KalmanFilterPredictModule::KalmanFilterPredictModule( percepto::Source<VectorType>* xPrev,
	                                                  percepto::Source<MatrixType>* sprev,
	                                                  const CovarianceEstimator::ModuleType& q,
	                                                  double dt,
	                                                  const VectorType& input,
	                                                  const MatrixType& F )
: Q( q ), xprev( xPrev ), Sprev( sprev )
{
	xminus.SetSource( xprev );
	xminus.SetTransform( F );

	qInput.SetOutput( input );
	Q.SetSource( &qInput );
	Qdt.SetSource( &Q );
	Qdt.SetScale( dt );
	FSFT.SetSource( Sprev );
	FSFT.SetTransform( F );
	Sminus.SetSourceA( &Qdt );
	Sminus.SetSourceB( &FSFT );
}

percepto::Source<VectorType>*
KalmanFilterPredictModule::GetTailState()
{
	return &xminus;
}

percepto::Source<MatrixType>* 
KalmanFilterPredictModule::GetTailCov()
{
	return &Sminus;
}

// NOTE We do not invalidate xprev or Sprev because we can't foreprop it
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
	os << "xprev: " << module.xprev->GetOutput().transpose() << std::endl;
	os << "xminus: " << module.xminus.GetOutput().transpose() << std::endl;
	os << "xminus dodx: " << module.xminus.GetDodxAcc() << std::endl;
	os << "Sprev: " << std::endl << module.Sprev->GetOutput() << std::endl;
	os << "Q: " << std::endl << module.Q.GetOutput() << std::endl;
	os << "Q dodx: " << std::endl << module.Q.GetDodxAcc() << std::endl;
	os << "dReg dodx: " << std::endl << module.Q.dReg.GetOutputSource().GetDodxAcc() << std::endl;
	os << "Sminus: " << std::endl << module.Sminus.GetOutput() << std::endl;
	return os;
}

KalmanFilterUpdateModule::KalmanFilterUpdateModule( percepto::Source<VectorType>* xPrev,
	                                                percepto::Source<MatrixType>* sPrev,
	                                                const CovarianceEstimator::ModuleType& r,
	                                                const VectorType& input,
	                                                const VectorType& obs,
	                                                const MatrixType& H )
: R( r ), xprev( xPrev ), Sprev( sPrev ), active( false )
{
	HTVinvv.SetTransform( H.transpose() );

	y.SetOutput( obs );
	ypred.SetSource( xPrev );
	ypred.SetTransform( H );
	innov.SetPlusSource( &y );
	innov.SetMinusSource( &ypred );

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
	innovationLL.SetCovSource( &V );
	innovationLL.SetSampleSource( &innov );
}

void KalmanFilterUpdateModule::Activate()
{
	if( !active )
	{
		Vinvv.SetMatSource( &Vinv );
		Vinvv.SetVecSource( &innov );
		HTVinvv.SetSource( &Vinvv );
		xcorr.SetMatSource( Sprev );
		xcorr.SetVecSource( &HTVinvv );
		xplus.SetSourceA( xprev );
		xplus.SetSourceB( &xcorr );

		Vinv.SetSource( &V );
		SHTVinvH.SetLeftSource( Sprev );
		SHTVinvHS.SetRightSource( Sprev );
		Splus.SetPlusSource( Sprev );
		active = true;
	}
}

percepto::Source<VectorType>*
KalmanFilterUpdateModule::GetTailState()
{
	if( !active )
	{
		Activate();
	}
	return &xplus;
}

percepto::Source<MatrixType>* 
KalmanFilterUpdateModule::GetTailCov()
{
	if( !active )
	{
		Activate();
	}
	return &Splus;
}

void KalmanFilterUpdateModule::Invalidate() 
{ 
	rInput.Invalidate(); 
	y.Invalidate();
}

void KalmanFilterUpdateModule::Foreprop() 
{ 
	rInput.Foreprop(); 
	y.Foreprop();
}

std::ostream& operator<<( std::ostream& os, const KalmanFilterUpdateModule& module )
{
	os << "Update module: " << module.sourceName << std::endl;
	os << "Sprev: " << std::endl << module.Sprev->GetOutput() << std::endl;
	os << "HSHT: " << std::endl << module.HSHT.GetOutput() << std::endl;
	// os << "HSHT dodx: " << std::endl << module.HSHT.GetDodxAcc() << std::endl;
	os << "R: " << std::endl << module.R.GetOutput() << std::endl;
	os << "R dodx: " << std::endl << module.R.GetDodxAcc() << std::endl;
	os << "dReg dodx: " << std::endl << module.R.dReg.GetOutputSource().GetDodxAcc() << std::endl;
	os << "V: " << std::endl << module.V.GetOutput() << std::endl;
	os << "V dodx: " << std::endl << module.V.GetDodxAcc() << std::endl;
	os << "inno: " << module.innov.GetOutput().transpose() << std::endl;
	os << "inno dodx: " << module.innov.GetDodxAcc() << std::endl;
	os << "innoLL: " << module.innovationLL.GetOutput() << std::endl;
	if( module.active )
	{
		os << "Splus: " << std::endl << module.Splus.GetOutput() << std::endl;
		// os << "Splus dodx: " << std::endl << module.Splus.GetDodxAcc() << std::endl;
	}
	return os;
}

}