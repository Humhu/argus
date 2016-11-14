#include "fieldtrack/PoseDerivativeFilter.h"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

MatrixType IntegralMatrix( double dt, unsigned int order )
{
	unsigned int N = PoseSE3::TangentDimension * (order + 1);
	MatrixType mat = MatrixType::Identity( N, N );

	double intTerm = 1.0;
	for( unsigned int o = 1; o <= order; ++o )
	{
		intTerm = intTerm * dt / o;
		for( int i = 0; i < PoseSE3::TangentDimension*(order-o+1); ++i )
		{
			mat( i, i + PoseSE3::TangentDimension*o ) = intTerm;
		}
	}
	return mat;
}

PoseDerivativeFilter::PoseDerivativeFilter( unsigned int order )
: _order( order )
{
	_derivs = VectorType::Zero( DerivsDim() );
	_cov = MatrixType::Identity( CovDim(), CovDim() );
	_tfunc = boost::bind( &IntegralMatrix, _1, order );
}

unsigned int PoseDerivativeFilter::PoseDim() const
{
	return PoseSE3::TangentDimension;
}

unsigned int PoseDerivativeFilter::DerivsDim() const 
{
	return _order * PoseDim(); 
}

unsigned int PoseDerivativeFilter::CovDim() const
{
	return (_order + 1) * PoseDim();
}

PoseSE3& PoseDerivativeFilter::Pose()
{
	return _pose;
}

const PoseSE3& PoseDerivativeFilter::Pose() const
{
	return _pose;
}

VectorType& PoseDerivativeFilter::Derivs()
{
	return _derivs;
}

const VectorType& PoseDerivativeFilter::Derivs() const
{
	return _derivs;
}

Eigen::Block<MatrixType> PoseDerivativeFilter::PoseCov() 
{
	return _cov.topLeftCorner( PoseDim(), PoseDim() );
}

Eigen::Block<const MatrixType> PoseDerivativeFilter::PoseCov() const 
{
	return _cov.topLeftCorner( PoseDim(), PoseDim() ); 
}
Eigen::Block<MatrixType> PoseDerivativeFilter::DerivsCov() 
{
	return _cov.bottomRightCorner( DerivsDim(), DerivsDim() );
}
Eigen::Block<const MatrixType> PoseDerivativeFilter::DerivsCov() const 
{
	return _cov.bottomRightCorner( DerivsDim(), DerivsDim() );
}

MatrixType& PoseDerivativeFilter::FullCov()
{
	return _cov;
}

const MatrixType& PoseDerivativeFilter::FullCov() const
{
	return _cov;
}

void PoseDerivativeFilter::WorldDisplace( const PoseSE3& d, const PoseSE3::CovarianceMatrix& Q )
{
	PoseSE3::AdjointMatrix adj = PoseSE3::Adjoint( _pose );
	_pose = d * _pose;
	_cov.topLeftCorner( PoseDim(), PoseDim() ) =
		adj * _cov.topLeftCorner( PoseDim(), PoseDim() ) * adj.transpose() + Q;
}

PredictInfo PoseDerivativeFilter::Predict( const MatrixType& Q, double dt )
{
	VectorType x( CovDim() );
	x.head( PoseDim() ) = VectorType::Zero( PoseDim() );
	x.tail( DerivsDim() ) = _derivs;
	
	PredictInfo info;
	info.xpre = x;
	info.Spre = _cov;
	info.dt = dt;
	info.Q = Q;

	// We use A first without the adjoint in the upper left block, but that 
	// block does not affect the derivatives anyways
	MatrixType A = _tfunc( dt );
	VectorType xup = A * x;
	PoseSE3 displacement = PoseSE3::Exp( xup.segment( PoseDim(), PoseDim() ) * dt );
	A.topLeftCorner( PoseDim(), PoseDim() ) = PoseSE3::Adjoint( displacement );
	_derivs = xup.tail( DerivsDim() );
	_pose = _pose * displacement;
	_cov = A * _cov * A.transpose() + Q;

	info.F = A;
	return info;
}

MatrixType PoseDerivativeFilter::ComputeKalmanGain( const MatrixType& C,
                                                    const MatrixType& R )
{
	// Compute Kalman gain
	MatrixType V = C * _cov * C.transpose() + R;
	Eigen::LDLT<MatrixType> Vinv( V ); // TODO Make sure this succeeds!
	if( Vinv.info() != Eigen::Success )
	{
		throw std::runtime_error( "Could not invert innovation noise matrix!" );
	}
	return _cov * C.transpose() * Vinv.solve( MatrixType::Identity( V.rows(), V.cols() ) );
}

void PoseDerivativeFilter::UpdateCovariance( const MatrixType& K,
                                             const MatrixType& C,
                                             const MatrixType& R )
{
	// Update the estimate covariance
	// Joseph form of the update is more numerically stable
	MatrixType l = MatrixType::Identity( CovDim(), CovDim() ) - K*C;
	_cov = l * _cov * l.transpose() + K * R * K.transpose();
}

UpdateInfo 
PoseDerivativeFilter::operator()( const PoseObservation& obs )
{
	// Record pre-update filter information
	UpdateInfo info;
	info.xpre = VectorType( CovDim() );
	info.xpre.head( PoseDim() ) = PoseSE3::Log( _pose );
	info.xpre.tail( DerivsDim() ) = _derivs;
	info.Spre = _cov;

	MatrixType C = MatrixType::Zero( PoseDim(), CovDim() );
	C.block( 0, 0, PoseDim(), PoseDim() ) = MatrixType::Identity( PoseDim(), PoseDim() );
	VectorType v = PoseSE3::Log( _pose.Inverse() * obs.pose );

	// Update the estimate mean
	MatrixType K = ComputeKalmanGain( C, obs.covariance );
	VectorType correction = K * v;
	_pose = _pose * PoseSE3::Exp( correction.head( PoseDim() ) );
	_derivs = _derivs + correction.tail( DerivsDim() );
	UpdateCovariance( K, C, obs.covariance );

	// TODO
	// Record post-update filter information
	// info.observation = obs;
	info.innovation = v;
	info.post_innovation = PoseSE3::Log( _pose.Inverse() * obs.pose );
	info.delta_x = correction;
	info.Spost = _cov;
	info.H = C;
	info.R = obs.covariance;
	return info;
}

// TODO
UpdateInfo 
PoseDerivativeFilter::operator()( const PositionObservation& obs )
{
	throw std::runtime_error( "Position updates are not yet implemented." );
}

// TODO
UpdateInfo 
PoseDerivativeFilter::operator()( const OrientationObservation& obs )
{
	throw std::runtime_error( "Orientation updates are not yet implemented." );
}

UpdateInfo 
PoseDerivativeFilter::operator()( const DerivObservation& obs )
{
	// Record pre-update filter information
	UpdateInfo info;
	info.xpre = VectorType( CovDim() );
	info.xpre.head( PoseDim() ) = PoseSE3::Log( _pose );
	info.xpre.tail( DerivsDim() ) = _derivs;
	info.Spre = _cov;

	unsigned int zDim = obs.indices.size();
	MatrixType C = MatrixType::Zero( zDim, CovDim() );
	for( unsigned int i = 0; i < zDim; i++ )
	{
		unsigned int ind = PoseDim() + obs.indices[i];
		C( i, ind ) = 1;
	}
	
	VectorType derivsObsed( zDim );
	GetSubmatrix( _derivs, derivsObsed, obs.indices );
	VectorType v = obs.derivatives - derivsObsed;

	// Update the estimate mean
	MatrixType K = ComputeKalmanGain( C, obs.covariance );
	VectorType correction = K * v;
	_pose = _pose * PoseSE3::Exp( correction.head( PoseDim() ) );
	_derivs = _derivs + correction.tail( DerivsDim() );
	UpdateCovariance( K, C, obs.covariance );

	// TODO
	// Record post-update filter information
	// info.observation = obs;
	info.innovation = v;
	GetSubmatrix( _derivs, derivsObsed, obs.indices );
	info.post_innovation = derivsObsed - obs.derivatives;
	info.delta_x = correction;
	info.Spost = _cov;
	info.H = C;
	info.R = obs.covariance;
	return info;
}

ObservationLikelihoodVisitor::ObservationLikelihoodVisitor( const PoseDerivativeFilter& filter )
: _filter( filter ) {}

double ObservationLikelihoodVisitor::operator()( const PoseObservation& obs )
{
	unsigned int P = _filter.PoseDim();
	MatrixType C = MatrixType::Zero( P, _filter.CovDim() );
	C.block( 0, 0, P, P ) = MatrixType::Identity( P, P );
	VectorType v = PoseSE3::Log( _filter.Pose().Inverse() * obs.pose );
	MatrixType V = C * _filter.FullCov() * C.transpose() + obs.covariance;
	return GaussianPDF( V, v );
}

double ObservationLikelihoodVisitor::operator()( const PositionObservation& obs )
{
	throw std::runtime_error( "Position updates are not yet implemented." );
}

double ObservationLikelihoodVisitor::operator()( const OrientationObservation& obs )
{
	throw std::runtime_error( "Orientation updates are not yet implemented." );
}

double ObservationLikelihoodVisitor::operator()( const DerivObservation& obs )
{
	MatrixType C = MatrixType::Zero( obs.indices.size(), _filter.CovDim() );
	for( unsigned int i = 0; i < obs.indices.size(); ++i )
	{
		C( i, obs.indices[i] ) = 1;
	}
	VectorType v = C * _filter.Derivs() - obs.derivatives;
	MatrixType V = C * _filter.FullCov() * C.transpose() + obs.covariance;
	return GaussianPDF( V, v );
}

}