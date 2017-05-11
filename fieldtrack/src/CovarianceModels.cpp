#include "fieldtrack/CovarianceModels.h"
#include <Eigen/Cholesky>

namespace argus
{
CovarianceModel::CovarianceModel() {}
CovarianceModel::~CovarianceModel() {}

FixedCovariance::FixedCovariance()
{
	link_ports( _logD.GetOutput(), _expD.GetInput() );
	link_ports( _expD.GetOutput(), _dReshape.GetInput() );
	link_ports( _dReshape.GetOutput(), _ldlt.GetCIn() );

	link_ports( _lVec.GetOutput(), _lReshape.GetInput() );
	link_ports( _lReshape.GetOutput(), _ldlt.GetXIn() );
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

void FixedCovariance::BindPredictModule( PredictModule& pred )
{
	link_ports( _ldlt.GetSOut(), pred.GetQIn() );
}

void FixedCovariance::BindUpdateModule( UpdateModule& upd )
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

void FixedCovariance::Initialize( const MatrixType& cov )
{
	Eigen::LDLT<MatrixType> ldlt( cov );
	if( ldlt.info() != Eigen::ComputationInfo::Success )
	{
		throw std::invalid_argument( "Input matrix does not appear to be PSD" );
	}

	unsigned int dim = cov.rows();

	std::vector<unsigned int> trilInds = gen_trilc_inds( dim, 1 );
	VectorType lInit( trilInds.size() );
	MatrixType matL = ldlt.matrixL();
	for( unsigned int i = 0; i < trilInds.size(); ++i )
	{
		lInit( i ) = matL( trilInds[i] );
	}
	_lVec.SetValue( lInit );
	_lReshape.SetShapeParams( MatrixType::Identity( dim, dim ), trilInds );

	std::vector<unsigned int> diagInds = gen_diag_inds( dim );
	VectorType dInit = ldlt.vectorD().array().log().matrix();
	_logD.SetValue( dInit );
	_dReshape.SetShapeParams( MatrixType::Zero( dim, dim ), diagInds );
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

const MatrixType& FixedCovariance::GetLBackpropValue() const
{
	return _lVec.GetBackpropValue();
}

const MatrixType& FixedCovariance::GetLogDBackpropValue() const
{
	return _logD.GetBackpropValue();
}

OutputPort& FixedCovariance::GetCovOut()
{
	return _ldlt.GetSOut();
}

AdaptiveCovariance::AdaptiveCovariance()
	: _windowSize( 1 ) {}

void AdaptiveCovariance::SetWindowSize( unsigned int s )
{
	_windowSize = s;
}

void AdaptiveCovariance::SetDefaultValue( const MatrixType& S )
{
	_defaultCov.SetValue( S );
}

void AdaptiveCovariance::Foreprop()
{}

void AdaptiveCovariance::Invalidate()
{}

void AdaptiveCovariance::UnregisterAll()
{
	_defaultCov.UnregisterAllConsumers( false );
	_pointRs.clear();
	_estRs.clear();
}

void AdaptiveCovariance::BindPredictModule( PredictModule& pred )
{
	// TODO
	throw std::runtime_error( "Adaptive covariance doesn't support predict yet" );
}

void AdaptiveCovariance::BindUpdateModule( UpdateModule& upd )
{
	if( _estRs.empty() )
	{
		link_ports( _defaultCov.GetOutput(), upd.GetRIn() );
	}
	else
	{
		link_ports( _estRs.back().GetOutput(), upd.GetRIn() );
	}

	_pointRs.emplace_back( upd );
	
	_estRs.emplace_back();
	MeanModule& currR = _estRs.back();
	unsigned int minInd = std::max( _pointRs.size() - _windowSize, (size_t) 0 );
	for( unsigned int i = minInd; i < _pointRs.size(); ++i )
	{
		currR.RegisterSource( _pointRs[i].GetOutput() );
	}
}

VectorType AdaptiveCovariance::GetParameters() const { return VectorType(); }

void AdaptiveCovariance::SetParameters( const VectorType& p )
{
	if( p.size() > 0 )
	{
		throw std::invalid_argument( "Adaptive covariance has no parameters" );
	}
}

MatrixType AdaptiveCovariance::GetBackpropValue() const { return MatrixType(); }

AdaptiveCovariance::PointEstimate::PointEstimate( UpdateModule& upd )
{
	link_ports( upd.GetUOut(), _uOp.GetInput() );
	link_ports( _uOp.GetOutput(), _estR.GetLeftIn() );
	link_ports( upd.GetPOut(), _estR.GetRightIn() );
}

OutputPort& AdaptiveCovariance::PointEstimate::GetOutput()
{
	return _estR.GetOutput();
}

}