#include "fieldtrack/CovarianceModels.h"
#include <Eigen/Cholesky>

namespace argus
{
FixedCovariance::FixedCovariance()
{
	link_ports( _logD.GetOutput(), _expD.GetInput() );
	link_ports( _expD.GetOutput(), _dReshape.GetInput() );
	link_ports( _dReshape.GetOutput(), _ldlt.GetCIn() );

	link_ports( _lVec.GetOutput(), _lReshape.GetInput() );
	link_ports( _lReshape.GetOutput(), _ldlt.GetXIn() );
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
}