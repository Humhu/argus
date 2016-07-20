#include "covreg/KalmanFilterModules.h"

namespace argus
{

KalmanFilterPredictModule::KalmanFilterPredictModule( percepto::Source<VectorType>* xPrev,
	                                                  percepto::Source<MatrixType>* sprev,
	                                                  const CovarianceEstimator::ModuleType& q,
	                                                  // const MatrixType& q,
	                                                  double dt,
	                                                  const VectorType& input,
	                                                  const MatrixType& F )
: Q( q ), xprev( xPrev ), Sprev( sprev )
{
	xminus.SetSource( xprev );
	xminus.SetTransform( F );

	qInput.SetOutput( input );
	Q.SetSource( &qInput );
	// Q.SetOutput( q );
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
	// Q.Invalidate();
}

void KalmanFilterPredictModule::Foreprop() 
{ 
	qInput.Foreprop();
	// Q.Foreprop();
}

std::ostream& operator<<( std::ostream& os, const KalmanFilterPredictModule& module )
{
	os << "Predict module: " << std::endl;
	// os << "xprev: " << module.xprev->GetOutput().transpose() << std::endl;
	// os << "xminus: " << module.xminus.GetOutput().transpose() << std::endl;
	os << "Sprev: " << std::endl << module.Sprev->GetOutput() << std::endl;
	os << "Q: " << std::endl << module.Q.GetOutput() << std::endl;
	if( module.Q.GetDodxAcc().size() > 0 )
	{
		// os << "Q dodx: " << std::endl << module.Q.GetDodxAcc() << std::endl;
		os << "dReg dodx: " << std::endl << module.Q.dReg.GetOutputSource().GetDodxAcc() << std::endl;
		// os << "dReg dodx: " << std::endl << module.Q.GetDodxAcc() << std::endl;
		// os << "dReg dodx: " << std::endl << module.Q.dReg.GetDodxAcc() << std::endl;
	}
	os << "Sminus: " << std::endl << module.Sminus.GetOutput() << std::endl;
	return os;
}

KalmanFilterUpdateModule::KalmanFilterUpdateModule( percepto::Source<VectorType>* xPrev,
	                                                percepto::Source<MatrixType>* sPrev,
	                                                const CovarianceEstimator::ModuleType& r,
	                                                const VectorType& input,
	                                                const VectorType& obs,
	                                                const MatrixType& H,
	                                                const VectorType& inno )
: R( r ), xprev( xPrev ), Sprev( sPrev ), active( false )
{
	HTVinvv.SetTransform( H.transpose() );

	y.SetOutput( obs );
	ypred.SetSource( xPrev );
	ypred.SetTransform( H );
	innov.SetPlusSource( &y );
	innov.SetMinusSource( &ypred );

	R.SetSource( &rInput );
	// Rinv.SetSource( &rInput );
	// R.SetSource( &Rinv );

	// finnov.SetOutput( inno );

	rInput.SetOutput( input );
	// R.SetSource( &rInput );
	HSHT.SetSource( Sprev );
	HSHT.SetTransform( H );
	V.SetSourceA( &HSHT );
	V.SetSourceB( &R );
	Vinv.SetSource( &V );

	HTVinvH.SetTransform( H.transpose() );
	
	innovationLL.SetInfoSource( &Vinv );
	innovationLL.SetSampleSource( &innov );
	// innovationLL.SetSampleSource( &finnov );
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

		HTVinvH.SetSource( &Vinv );
		SHTVinvH.SetLeftSource( Sprev );
		SHTVinvH.SetRightSource( &HTVinvH );
		SHTVinvHS.SetLeftSource( &SHTVinvH );
		SHTVinvHS.SetRightSource( Sprev );
		Splus.SetMinusSource( &SHTVinvHS );
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
	// finnov.Invalidate();
}

void KalmanFilterUpdateModule::Foreprop() 
{ 
	rInput.Foreprop(); 
	y.Foreprop();
	// finnov.Foreprop();
}

std::ostream& operator<<( std::ostream& os, const KalmanFilterUpdateModule& module )
{
	os << "Update module: " << module.sourceName << std::endl;
	os << "Sprev: " << std::endl << module.Sprev->GetOutput() << std::endl;
	os << "HSHT: " << std::endl << module.HSHT.GetOutput() << std::endl;
	os << "R: " << std::endl << module.R.GetOutput() << std::endl;
	// os << "Rinv: " << std::endl << module.Rinv.GetOutput() << std::endl;
	if( module.R.GetDodxAcc().size() > 0 )
	{
		os << "R dodx: " << std::endl << module.R.GetDodxAcc() << std::endl;
		// os << "Rinv dodx: " << std::endl << module.Rinv.GetDodxAcc() << std::endl;
		os << "dReg dodx: " << std::endl << module.R.dReg.GetOutputSource().GetDodxAcc() << std::endl;
		// os << "dReg dodx: " << std::endl << module.Rinv.dReg.GetDodxAcc() << std::endl;
	}
	os << "V: " << std::endl << module.V.GetOutput() << std::endl;
	os << "Vinv: " << std::endl << module.Vinv.GetOutput() << std::endl;
	// os << "HTVinvH: " << std::endl << module.HTVinvH.GetOutput() << std::endl;
	// os << "SHTVinvHS: " << std::endl << module.SHTVinvHS.GetOutput() << std::endl;
	// os << "inno: " << module.innov.GetOutput().transpose() << std::endl;
	os << "inno: " << module.innov.GetOutput().transpose() << std::endl;
	if( module.V.GetDodxAcc().size() > 0 )
	{
		os << "V dodx: " << std::endl << module.V.GetDodxAcc() << std::endl;
		os << "Vinv dodx: " << std::endl << module.Vinv.GetDodxAcc() << std::endl;
		os << "inno dodx: " << module.innov.GetDodxAcc() << std::endl;
	}
	os << "innoLL: " << module.innovationLL.GetOutput() << std::endl;
	if( module.active )
	{
		os << "Splus: " << std::endl << module.Splus.GetOutput() << std::endl;
	}
	return os;
}

}