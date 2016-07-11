#include "covreg/KalmanFilterEpisode.h"

namespace argus
{

KalmanFilterEpisode::KalmanFilterEpisode( const VectorType& xinit,
                                          const MatrixType& Sinit,
                                          const ros::Time& t )
: tailType( CLIP_TYPE_NONE ), startTime( t )
{
	initState.SetOutput( xinit );
	initCov.SetOutput( Sinit );
	sumInnoLL.AddSource( &offsetInnoLL );
	offsetInnoLL.SetOutput( 0.0 );
}

size_t KalmanFilterEpisode::NumUpdates() const { return updates.size(); }

percepto::Source<VectorType>* 
KalmanFilterEpisode::GetTailState()
{
	if( tailType == CLIP_TYPE_NONE )
	{
		return &initState;
	}
	else if( tailType == CLIP_TYPE_PREDICT )
	{
		return predicts.back().GetTailState();
	}
	else if( tailType == CLIP_TYPE_UPDATE )
	{
		return updates.back().GetTailState();
	}
	else
	{
		throw std::runtime_error( "Invalid tail type." );
	}
}

percepto::Source<MatrixType>* 
KalmanFilterEpisode::GetTailCov()
{
	if( tailType == CLIP_TYPE_NONE )
	{
		return &initCov;
	}
	else if( tailType == CLIP_TYPE_PREDICT )
	{
		percepto::Source<MatrixType>* s = predicts.back().GetTailCov();
		return s;
	}
	else if( tailType == CLIP_TYPE_UPDATE )
	{
		percepto::Source<MatrixType>* s = updates.back().GetTailCov();
		return s;
	}
	else
	{
		throw std::runtime_error( "Invalid tail type." );
	}
}

percepto::Source<double>* KalmanFilterEpisode::GetLL()
{
	return &sumInnoLL;
}

// NOTE: We do not invalidate Sprev because we can't foreprop it
void KalmanFilterEpisode::Invalidate()
{
	// NOTE Shouldn't have to invalidate initCov, but to be safe...
	initState.Invalidate();
	initCov.Invalidate();
	offsetInnoLL.Invalidate();
	BOOST_FOREACH( KalmanFilterPredictModule& pred, predicts )
	{
		pred.Invalidate();
	}
	BOOST_FOREACH( KalmanFilterUpdateModule& upd, updates )
	{
		upd.Invalidate();
	}
}

void KalmanFilterEpisode::Foreprop()
{
	initState.Foreprop();
	initCov.Foreprop();
	offsetInnoLL.Foreprop();

	BOOST_FOREACH( KalmanFilterPredictModule& pred, predicts )
	{
		pred.Foreprop();
	}
	BOOST_FOREACH( KalmanFilterUpdateModule& upd, updates )
	{
		upd.Foreprop();
	}
}

void KalmanFilterEpisode::ForepropAll()
{
	Foreprop();
}

std::ostream& operator<<( std::ostream& os, const KalmanFilterEpisode& episode )
{
	os << "Kalman filter episode:" << std::endl;
	unsigned int predInd = 0;
	unsigned int updInd = 0;
	for( unsigned int i = 0; i < episode.order.size(); i++ )
	{
		if( episode.order[i] == KalmanFilterEpisode::ClipType::CLIP_TYPE_PREDICT )
		{
			os << episode.predicts[ predInd ] << std::endl;
			predInd++;
		}
		else if( episode.order[i] == KalmanFilterEpisode::ClipType::CLIP_TYPE_UPDATE )
		{
			os << episode.updates[ updInd ] << std::endl;
			updInd++;
		}
	}
	return os;
}

}