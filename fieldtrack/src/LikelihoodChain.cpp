#include "fieldtrack/LikelihoodChain.h"
#include <boost/foreach.hpp>

namespace argus
{
LikelihoodChain::LikelihoodChain()
	: _initialized( false )
{}

LikelihoodChain::~LikelihoodChain()
{
	ClearChain();
}

OutputPort& LikelihoodChain::GetMeanLL()
{
	return _meanLL.GetOutput();
}

void LikelihoodChain::InitializeChain( const VectorType& x,
                                       const MatrixType& P )
{
	KalmanChain::Initialize( x, P );
}

void LikelihoodChain::ClearChain()
{
	_transCov->UnregisterAll();
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item & item, _obsModels )
	{
		CovarianceModel::Ptr& model = item.second;
		model->UnregisterAll();
	}
	_glls.clear();
	_scalers.clear();
	_meanLL.UnregisterAllSources( false );
	KalmanChain::Clear();
	_initialized = false;
}

void LikelihoodChain::RegisterTransCov( const CovarianceModel::Ptr& model )
{
	_transCov = model;
}

void LikelihoodChain::RegisterObsSource( const std::string& name,
                                         const CovarianceModel::Ptr& model )
{
	if( _obsModels.count( name ) > 0 )
	{
		throw std::invalid_argument( "Source " + name + " already registered." );
	}
	_obsModels[name] = model;
}

void LikelihoodChain::ProcessInfo( const FilterInfo& info )
{
	boost::apply_visitor( *this, info );
}

void LikelihoodChain::operator()( const PredictInfo& info )
{
	// TODO Bounded?
	_accPreds.emplace_back( info );
}

void LikelihoodChain::operator()( const UpdateInfo& info )
{
	const std::string& source = info.frameId;
	if( _obsModels.count( source ) == 0 )
	{
		throw std::invalid_argument( "Source " + source + " unregistered!" );
	}

	ProcessPreds();

	if( !_initialized )
	{
		InitializeChain( info.prior_state, info.prior_state_cov );
		_initialized = true;
	}

	UpdateModulePtr upd = KalmanChain::AddLinearUpdate( info.obs_jacobian,
	                                                    info.obs );
	_obsModels[source]->BindUpdateModule( *upd, info );

	_glls.emplace_back();
	_scalers.emplace_back();
	GaussianLikelihoodModule& gll = _glls.back();

	_meanLL.RegisterSource( gll.GetLLOut() );
	link_ports( upd->GetVOut(), gll.GetXIn() );
	link_ports( upd->GetSOut(), gll.GetSIn() );
}

void LikelihoodChain::ProcessPreds()
{
	BOOST_FOREACH( const PredictInfo &info, _accPreds )
	{
		if( !_initialized )
		{
			InitializeChain( info.prior_state, info.prior_state_cov );
			_initialized = true;
		}
		PredictModulePtr pred = KalmanChain::AddLinearPredict( info.trans_jacobian );
		_transCov->BindPredictModule( *pred, info );
	}
	_accPreds.clear();
}
}