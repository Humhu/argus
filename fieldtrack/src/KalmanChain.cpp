#include "fieldtrack/KalmanChain.h"

namespace argus
{
KalmanChain::KalmanChain() {}

KalmanChain::~KalmanChain() {}

void KalmanChain::Initialize( const VectorType& x0,
                              const MatrixType& P0 )
{
	_prior.SetX( x0 );
	_prior.SetP( P0 );
}

void KalmanChain::Foreprop()
{
	_prior.Foreprop();
}

void KalmanChain::Invalidate()
{
	_prior.Invalidate();
}

size_t KalmanChain::NumModules() const { return _types.size(); }
size_t KalmanChain::NumPredicts() const { return _predicts.size(); }
size_t KalmanChain::NumUpdates() const { return _updates.size(); }

KalmanChain::ModulePtrPair KalmanChain::RemoveEarliest()
{
	KalmanChain::ModulePtrPair out;
	out.first = nullptr;
	out.second = nullptr;
	if( _types.empty() ) { return out; }

	// NOTE All this specialized logic is sort of a nightmare... can we simplify it?
	if( _types.front() == CHAIN_PREDICT )
	{
		unlink_kalman_ports( _prior, *_predicts[0] );
		if( _types.size() > 1 )
		{
			if( _types[1] == CHAIN_PREDICT )
			{
				unlink_kalman_ports( *_predicts[0], *_predicts[1] );
				link_kalman_ports( _prior, *_predicts[1] );
			}
			else if( _types[1] == CHAIN_UPDATE )
			{
				unlink_kalman_ports( *_predicts[0], *_updates[0] );
				link_kalman_ports( _prior, *_updates[0] );
			}
		}
		_prior.SetX( _predicts[0]->GetX() );
		_prior.SetP( _predicts[0]->GetP() );

		out.first = _predicts.front();
		_predicts.pop_front();
	}
	else // _types.front() == CHAIN_UPDATE
	{
		unlink_kalman_ports( _prior, *_updates[0] );
		if( _types.size() > 1 )
		{
			if( _types[1] == CHAIN_PREDICT )
			{
				unlink_kalman_ports( *_updates[0], *_predicts[0] );
				link_kalman_ports( _prior, *_predicts[0] );
			}
			else if( _types[1] == CHAIN_UPDATE )
			{
				unlink_kalman_ports( *_updates[0], *_updates[1] );
				link_kalman_ports( _prior, *_updates[1] );
			}
		}
		_prior.SetX( _updates[0]->GetX() );
		_prior.SetP( _updates[0]->GetP() );
		out.second = _updates.front();
		_updates.pop_front();
	}
	_types.pop_front();
	return out;
}

void KalmanChain::Clear()
{
	_types.clear();
	_predicts.clear();
	_updates.clear();
	_prior.UnregisterAllConsumers( false );
}

KalmanChain::PredictModulePtr KalmanChain::AddLinearPredict( const MatrixType& A )
{
	PredictModulePtr pred = std::make_shared<PredictModule>();
	KalmanOut& prev = GetLastModule();
	_predicts.emplace_back( pred );
	pred->SetLinearParams( A );
	link_kalman_ports( prev, *pred );
	_types.push_back( CHAIN_PREDICT );
	return pred;
}

KalmanChain::UpdateModulePtr KalmanChain::AddLinearUpdate( const MatrixType& C,
                                                           const VectorType& y )
{
	UpdateModulePtr upd = std::make_shared<UpdateModule>();
	KalmanOut& prev = GetLastModule();	
	_updates.emplace_back( upd );
	upd->SetLinearParams( C, y );
	link_kalman_ports( prev, *upd );
	_types.push_back( CHAIN_UPDATE );
	return upd;
}

KalmanIn& KalmanChain::GetFirstModule()
{
	if( _types.empty() )
	{
		throw std::runtime_error( "Cannot get first module with no modules" );
	}
	else if( _types.front() == CHAIN_PREDICT )
	{
		return *_predicts.front();
	}
	else // _types.front() == CHAIN_UPDATE
	{
		return *_updates.front();
	}
}

KalmanOut& KalmanChain::GetLastModule()
{
	if( _types.empty() )
	{
		return _prior;
	}
	else if( _types.back() == CHAIN_PREDICT )
	{
		return *_predicts.back();
	}
	else // _types.back() == CHAIN_UPDATE
	{
		return *_updates.back();
	}
}
}