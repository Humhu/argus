#include "fieldtrack/LikelihoodChainEstimator.h"
#include <boost/foreach.hpp>

namespace argus
{
LikelihoodChainEstimator::LikelihoodChainEstimator() 
{
	link_ports( _meanLL.GetOutput(), _meanSink.GetInput() );
}

void LikelihoodChainEstimator::Foreprop()
{
	_transCov->Foreprop();
	_chain.Foreprop();
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item& item, _obsModels )
	{
		CovarianceModel::Ptr& model = item.second;
		model->Foreprop();
	}
}

void LikelihoodChainEstimator::Backprop()
{
	_meanSink.Backprop( MatrixType::Identity(1,1) );
}

void LikelihoodChainEstimator::Invalidate()
{
	_transCov->Invalidate();
	_chain.Invalidate();
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item& item, _obsModels )
	{
		CovarianceModel::Ptr& model = item.second;
		model->Invalidate();
	}
}

double LikelihoodChainEstimator::GetMeanLL()
{
	return _meanSink.GetValue()(0);
}

void LikelihoodChainEstimator::InitializeChain( const VectorType& x,
                                                const MatrixType& P )
{
	_chain.Initialize( x, P );
}

void LikelihoodChainEstimator::ClearChain()
{
	_chain.Clear();
	_glls.clear();
	_meanLL.UnregisterAllSources( false );
}

void LikelihoodChainEstimator::RegisterTransCov( const CovarianceModel::Ptr& model )
{
	_transCov = model;
}

void LikelihoodChainEstimator::RegisterObsSource( const std::string& name,
                                                  const CovarianceModel::Ptr& model )
{
	if( _obsModels.count( name ) > 0 )
	{
		throw std::invalid_argument( "Source " + name + " already registered." );
	}
	_obsModels[name] = model;
}

void LikelihoodChainEstimator::ProcessInfo( const FilterInfo& info )
{
	boost::apply_visitor( *this, info );
}

void LikelihoodChainEstimator::operator()( const PredictInfo& info )
{
	PredictModulePtr pred = _chain.AddLinearPredict( info.trans_jacobian );
	_transCov->BindPredictModule( *pred );
}

void LikelihoodChainEstimator::operator()( const UpdateInfo& info )
{
	const std::string& source = info.frameId;
	if( _obsModels.count( source ) == 0 )
	{
		throw std::invalid_argument( "Source " + source + " unregistered!" );
	}

	UpdateModulePtr upd = _chain.AddLinearUpdate( info.obs_jacobian,
	                                              info.obs );
	_obsModels[source]->BindUpdateModule( *upd );
	
	_glls.emplace_back();
	GaussianLikelihoodModule& gll = _glls.back();
	_meanLL.RegisterSource( gll.GetLLOut() );
	link_ports( upd->GetVOut(), gll.GetXIn() );
	link_ports( upd->GetSOut(), gll.GetSIn() );
}
}