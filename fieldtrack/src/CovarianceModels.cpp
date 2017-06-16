#include "fieldtrack/CovarianceModels.h"
#include <boost/foreach.hpp>
#include <Eigen/Cholesky>

namespace argus
{
CovarianceModel::CovarianceModel() {}
CovarianceModel::~CovarianceModel() {}

VectorType CovarianceModel::GetParameters() const { return VectorType(); }

void CovarianceModel::SetParameters( const VectorType& p )
{
	if( p.size() > 0 )
	{
		throw std::invalid_argument( "Covariance model has no parameters" );
	}
}

MatrixType CovarianceModel::GetBackpropValue() const { return MatrixType(); }

PassCovariance::PassCovariance() {}

void PassCovariance::Foreprop()
{
	BOOST_FOREACH( ConstantModule & mod, _covs )
	{
		mod.Foreprop();
	}
}

void PassCovariance::Invalidate()
{
	BOOST_FOREACH( ConstantModule & mod, _covs )
	{
		mod.Invalidate();
	}
}

void PassCovariance::UnregisterAll()
{
	// TODO Shift unregistration functionality into ModuleBase destructor
	BOOST_FOREACH( ConstantModule & mod, _covs )
	{
		mod.UnregisterAllConsumers();
	}
	_covs.clear();
}

void PassCovariance::BindPredictModule( PredictModule& pred,
                                        const PredictInfo& info )
{
	_covs.emplace_back();
	ConstantModule& cov = _covs.back();
	cov.SetValue( info.trans_noise_cov );
	link_ports( cov.GetOutput(), pred.GetQIn() );
}

void PassCovariance::BindUpdateModule( UpdateModule& upd,
                                       const UpdateInfo& info )
{
	_covs.emplace_back();
	ConstantModule& cov = _covs.back();
	cov.SetValue( info.obs_noise_cov );
	link_ports( cov.GetOutput(), upd.GetRIn() );
}

FixedCovariance::FixedCovariance()
	: _enableL( true ), _enableD( true )
{
	LinkPorts();
}

FixedCovariance::FixedCovariance( const FixedCovariance& other )
	: _enableL( other._enableL ), _enableD( other._enableD )
{
	LinkPorts();

	MatrixType lBase, dBase;
	std::vector<IndPair> lInds, dInds;
	other._lReshape.GetShapeParams( lBase, lInds );
	other._dReshape.GetShapeParams( dBase, dInds );
	_lReshape.SetShapeParams( lBase, lInds );
	_dReshape.SetShapeParams( dBase, dInds );
	_logD.SetValue( other._logD.GetValue() );
	_lVec.SetValue( other._lVec.GetValue() );
}

void FixedCovariance::Foreprop()
{
	_logD.Foreprop();
	_lVec.Foreprop();
}

void FixedCovariance::Invalidate()
{
	_logD.Invalidate();
	_lVec.Invalidate();
}

void FixedCovariance::UnregisterAll()
{
	_ldlt.UnregisterAllConsumers( false );
}

void FixedCovariance::BindPredictModule( PredictModule& pred,
                                         const PredictInfo& info )
{
	link_ports( _ldlt.GetSOut(), pred.GetQIn() );
}

void FixedCovariance::BindUpdateModule( UpdateModule& upd,
                                        const UpdateInfo& info )
{
	link_ports( _ldlt.GetSOut(), upd.GetRIn() );
}

VectorType FixedCovariance::GetParameters() const
{
	VectorType L = GetL();
	VectorType D = GetLogD();
	VectorType out( L.size() + D.size() );
	out.head( L.size() ) = L;
	out.tail( D.size() ) = D;
	return out;
}

void FixedCovariance::SetParameters( const VectorType& p )
{
	VectorType L = GetL();
	VectorType D = GetLogD();
	if( p.size() != (L.size() + D.size() ) )
	{
		throw std::invalid_argument( "Got invalid sized parameter vector" );
	}
	VectorType Lp = p.head( L.size() );
	VectorType Dp = p.tail( D.size() );
	SetL( Lp );
	SetLogD( Dp );
}

MatrixType FixedCovariance::GetBackpropValue() const
{
	const MatrixType& dL = GetLBackpropValue();
	const MatrixType& dD = GetLogDBackpropValue();
	MatrixType out( dL.rows(), dL.cols() + dD.cols() );
	out.leftCols( dL.cols() ) = dL;
	out.rightCols( dD.cols() ) = dD;
	return out;
}

void FixedCovariance::EnableL( bool enable ) { _enableL = enable; }
void FixedCovariance::EnableD( bool enable ) { _enableD = enable; }

void FixedCovariance::Initialize( const MatrixType& cov )
{
	Eigen::LDLT<MatrixType> ldlt( cov );
	if( ldlt.info() != Eigen::ComputationInfo::Success )
	{
		throw std::invalid_argument( "Input matrix does not appear to be PSD" );
	}

	unsigned int dim = cov.rows();

	std::vector<IndPair> trilInds = gen_trilc_inds( dim, 1 );
	VectorType lInit( trilInds.size() );
	MatrixType matL = ldlt.matrixL();
	for( unsigned int i = 0; i < trilInds.size(); ++i )
	{
		lInit( trilInds[i].first ) = matL( trilInds[i].second );
	}
	_lVec.SetValue( lInit );
	_lReshape.SetShapeParams( MatrixType::Identity( dim, dim ), trilInds );

	std::vector<IndPair> diagInds = gen_vec_to_diag_inds( dim );
	VectorType dInit = (ldlt.transpositionsP().transpose() * ldlt.vectorD()).array().log().matrix();
	_logD.SetValue( dInit );
	_dReshape.SetShapeParams( MatrixType::Zero( dim, dim ), diagInds );
}

void FixedCovariance::LinkPorts()
{
	link_ports( _logD.GetOutput(), _expD.GetInput() );
	link_ports( _expD.GetOutput(), _dReshape.GetInput() );
	link_ports( _dReshape.GetOutput(), _ldlt.GetCIn() );

	link_ports( _lVec.GetOutput(), _lReshape.GetInput() );
	link_ports( _lReshape.GetOutput(), _ldlt.GetXIn() );
}

void FixedCovariance::SetL( const VectorType& L )
{
	if( L.size() != _lVec.GetValue().size() )
	{
		throw std::invalid_argument( "L vector incorrect size" );
	}
	_lVec.SetValue( L );
}

VectorType FixedCovariance::GetL() const
{
	const MatrixType& L = _lVec.GetValue();
	Eigen::Map<const VectorType> l( L.data(), L.size(), 1 );
	return l;
}

void FixedCovariance::SetLogD( const VectorType& D )
{
	if( D.size() != _logD.GetValue().size() )
	{
		throw std::invalid_argument( "D vector incorrect size" );
	}
	_logD.SetValue( D );
}

VectorType FixedCovariance::GetLogD() const
{
	const MatrixType& D = _logD.GetValue();
	Eigen::Map<const VectorType> d( D.data(), D.size(), 1 );
	return d;
}

MatrixType FixedCovariance::GetLBackpropValue() const
{
	MatrixType b = _lVec.GetBackpropValue();
	if( !_enableL ) { b.setZero(); }
	return b;
}

MatrixType FixedCovariance::GetLogDBackpropValue() const
{
	MatrixType b = _logD.GetBackpropValue();
	if( !_enableD ) { b.setZero(); }
	return b;
}

MatrixType FixedCovariance::GetValue() const
{
	// Can you say MEGAHACK? Still, doesn't seem a much cleaner way to do this...
	FixedCovariance copy( *this );
	SinkModule cov;
	link_ports( copy.GetCovOut(), cov.GetInput() );
	copy.Foreprop();
	return cov.GetValue();
}

MatrixType FixedCovariance::GetLValue() const
{
	FixedCovariance copy( *this );
	SinkModule L;
	link_ports( copy._lReshape.GetOutput(), L.GetInput() );
	copy.Foreprop();
	return L.GetValue();
}

VectorType FixedCovariance::GetDValue() const
{
	FixedCovariance copy( *this );
	SinkModule d;
	link_ports( copy._expD.GetOutput(), d.GetInput() );
	copy.Foreprop();
	return d.GetValue();
}

OutputPort& FixedCovariance::GetCovOut()
{
	return _ldlt.GetSOut();
}

TimeScaledCovariance::TimeScaledCovariance() {}

void TimeScaledCovariance::UnregisterAll()
{
	_scalers.clear();
	FixedCovariance::UnregisterAll();
}

void TimeScaledCovariance::BindPredictModule( PredictModule& pred,
                                              const PredictInfo& info )
{
	_scalers.emplace_back();
	ScaleModule& scale = _scalers.back();
	scale.SetScale( info.step_dt );
	link_ports( FixedCovariance::GetCovOut(), scale.GetInput() );
	link_ports( scale.GetOutput(), pred.GetQIn() );
}

void TimeScaledCovariance::BindUpdateModule( UpdateModule& upd,
                                             const UpdateInfo& info )
{
	throw std::runtime_error( "Time scaled covariance does not support update modules" );
}

AdaptiveCovariance::AdaptiveCovariance()
	: _lastUpdate( nullptr ), _windowSize( 1 ) {}

void AdaptiveCovariance::SetWindowSize( unsigned int s )
{
	_windowSize = s;
}

void AdaptiveCovariance::SetDefaultValue( const MatrixType& S )
{
	_defaultCov.SetValue( S );
}

void AdaptiveCovariance::Foreprop()
{
	_defaultCov.Foreprop();
}

void AdaptiveCovariance::Invalidate()
{
	_defaultCov.Invalidate();
}

void AdaptiveCovariance::UnregisterAll()
{
	_defaultCov.UnregisterAllConsumers( false );
	_pointRs.clear();
	_estRs.clear();
	_lastUpdate = nullptr;
}

void AdaptiveCovariance::BindPredictModule( PredictModule& pred,
                                            const PredictInfo& info )
{
	// TODO
	throw std::runtime_error( "Adaptive covariance doesn't support predict yet" );
}

void AdaptiveCovariance::BindUpdateModule( UpdateModule& upd,
                                           const UpdateInfo& info )
{
	if( _lastUpdate )
	{
		_pointRs.emplace_back( *_lastUpdate );
		_estRs.emplace_back();

		MeanModule& currR = _estRs.back();
		// NOTE We have to explicitly cast to int since otherwise we will get large
		// positive numbers instead of negative, as desired
		int lagHead = (int) _pointRs.size() - _windowSize;
		unsigned int minInd = std::max( lagHead, 0 );
		for( unsigned int i = minInd; i < _pointRs.size(); ++i )
		{
			currR.RegisterSource( _pointRs[i].GetOutput() );
		}
		_lastUpdate = nullptr;
	}

	if( _estRs.empty() )
	{
		link_ports( _defaultCov.GetOutput(), upd.GetRIn() );
	}
	else
	{
		link_ports( _estRs.back().GetOutput(), upd.GetRIn() );
	}
	_lastUpdate = &upd;
}

AdaptiveCovariance::PointEstimate::PointEstimate( UpdateModule& upd )
{
	const VectorType& y = upd.GetObs();
	unsigned int dim = y.size();

	link_ports( upd.GetUOut(), _uOp.GetInput() );
	link_ports( _uOp.GetOutput(), _denseR.GetLeftIn() );
	link_ports( upd.GetPOut(), _CPCT.GetCIn() );
	_CPCT.SetX( upd.GetObsMatrix().transpose() );
	link_ports( _CPCT.GetSOut(), _denseR.GetRightIn() );
	_diagR.SetShapeParams( MatrixType::Zero( dim, dim ),
	                       gen_dense_to_diag_inds( dim ) );
	link_ports( _denseR.GetOutput(), _diagR.GetInput() );
}

OutputPort& AdaptiveCovariance::PointEstimate::GetOutput()
{
	return _diagR.GetOutput();
}
}
