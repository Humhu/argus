#include "fieldtrack/KalmanChain.h"

namespace argus
{
KalmanChain::KalmanChain() {}

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

void KalmanChain::RemoveEarliest()
{
	if( _types.empty() ) { return; }

	// NOTE All this specialized logic is sort of a nightmare... can we simplify it?
	if( _types.front() == CHAIN_PREDICT )
	{
		unlink_kalman_ports( _prior, _predicts[0] );
		if( _types.size() > 1 )
		{
			if( _types[1] == CHAIN_PREDICT )
			{
				unlink_kalman_ports( _predicts[0], _predicts[1] );
				link_kalman_ports( _prior, _predicts[1] );
			}
			else if( _types[1] == CHAIN_UPDATE )
			{
				unlink_kalman_ports( _predicts[0], _updates[0] );
				link_kalman_ports( _prior, _updates[0] );
			}
		}
		_prior.SetX( _predicts[0].GetX() );
		_prior.SetP( _predicts[0].GetP() );
		_predicts.pop_front();
	}
	else // _types.front() == CHAIN_UPDATE
	{
		unlink_kalman_ports( _prior, _updates[0] );
		unlink_ports( _updates[0].GetVOut(), _likelihoods[0].GetXIn() );
		unlink_ports( _updates[0].GetSOut(), _likelihoods[0].GetSIn() );
		_meanLikelihood.UnregisterSource( _likelihoods[0].GetLLOut() );
		
		if( _types.size() > 1 )
		{
			if( _types[1] == CHAIN_PREDICT )
			{
				unlink_kalman_ports( _updates[0], _predicts[0] );
				link_kalman_ports( _prior, _predicts[0] );
			}
			else if( _types[1] == CHAIN_UPDATE )
			{
				unlink_kalman_ports( _updates[0], _updates[1] );
				link_kalman_ports( _prior, _updates[1] );
			}
		}
		_prior.SetX( _updates[0].GetX() );
		_prior.SetP( _updates[0].GetP() );
		_updates.pop_front();
		_likelihoods.pop_front();
	}
}

void KalmanChain::AddLinearPredict( const MatrixType& A,
                                    OutputPort& Qsrc )
{
	_predicts.emplace_back();
	PredictModule& pred = _predicts.back();
	pred.SetLinearParams( A );

	link_kalman_ports( GetLastModule(), pred );
	link_ports( Qsrc, pred.GetQIn() );
	_types.push_back( CHAIN_PREDICT );
}

void KalmanChain::AddLinearUpdate( const MatrixType& C,
                                   const VectorType& y,
                                   OutputPort& Rsrc )
{
	_updates.emplace_back();
	_likelihoods.emplace_back();
	UpdateModule& upd = _updates.back();
	GaussianLikelihoodModule& ll = _likelihoods.back();
	
	upd.SetLinearParams( C, y );
	link_ports( upd.GetVOut(), ll.GetXIn() );
	link_ports( upd.GetSOut(), ll.GetSIn() );
	link_kalman_ports( GetLastModule(), upd );
	link_ports( Rsrc, upd.GetRIn() );
	_types.push_back( CHAIN_UPDATE );
	_meanLikelihood.RegisterSource( ll.GetLLOut() );
}

OutputPort& KalmanChain::GetMeanLikelihood()
{
	return _meanLikelihood.GetOutput();
}

KalmanIn& KalmanChain::GetFirstModule()
{
	if( _types.empty() )
	{
		throw std::runtime_error("Cannot get first module with no modules");
	}
	else if( _types.front() == CHAIN_PREDICT )
	{
		return _predicts.front();
	}
	else // _types.front() == CHAIN_UPDATE
	{
		return _updates.front();
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
		return _predicts.back();
	}
	else // _types.back() == CHAIN_UPDATE
	{
		return _updates.back();
	}
}
}