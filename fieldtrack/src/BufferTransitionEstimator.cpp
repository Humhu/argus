#include "fieldtrack/BufferTransitionEstimator.h"
#include <boost/foreach.hpp>

namespace argus
{
BufferTransitionEstimator::BufferTransitionEstimator()
: _alpha( 0.1 ) {}

void BufferTransitionEstimator::Initialize( const MatrixType& Q )
{
	if( !_transCov )
	{
		_transCov = std::make_shared<FixedCovariance>();
	}
	_transCov->Initialize( Q );
}

void BufferTransitionEstimator::SetStepSize( double alpha )
{
	_alpha = alpha;
}

void BufferTransitionEstimator::Step()
{
	// First foreprop the chain
	_chain->Foreprop();
	BOOST_FOREACH( ConstantModule& mod, _Rs )
	{
		mod.Foreprop();
	}
	_transCov->Foreprop();

	// Backprop
	_meanLL->Backprop( MatrixType::Identity(1,1) );
	const MatrixType& dQdL = _transCov->GetLBackpropValue();
	const MatrixType& dQdD = _transCov->GetLogDBackpropValue();
	Eigen::Map<const VectorType> dQdLvec( dQdL.data(), dQdL.size(), 1 );
	Eigen::Map<const VectorType> dQdDvec( dQdD.data(), dQdD.size(), 1 );
	VectorType L = _transCov->GetL() + _alpha * dQdLvec;
	VectorType logD = _transCov->GetLogD() + _alpha * dQdDvec;	

	// Clear
	_chain.reset();
	_Rs.clear();
	_transCov.reset();
	_meanLL.reset();

	// Reinitialize
	_transCov = std::make_shared<FixedCovariance>();
	_transCov->SetL( L );
	_transCov->SetLogD( logD );
}

void BufferTransitionEstimator::ProcessInfo( const VectorType& x,
                                             const VectorType& P,
                                             const FilterInfo& info )
{
	if( !_chain )
	{
		_chain = std::make_shared<KalmanChain>();
		_chain->Initialize( x, P );
		_meanLL = std::make_shared<SinkModule>();
		link_ports( _chain->GetMeanLikelihood(), _meanLL->GetInput() );
	}
	boost::apply_visitor( *this, info );
}

void BufferTransitionEstimator::operator()( const PredictInfo& info )
{
	_chain->AddLinearPredict( info.trans_jacobian, _transCov->GetCovOut() );
}

void BufferTransitionEstimator::operator()( const UpdateInfo& info )
{
	_Rs.emplace_back();
	_Rs.back().SetValue( info.obs_noise_cov );
	_chain->AddLinearUpdate( info.obs_jacobian, _Rs.back().GetOutput() );
}

}