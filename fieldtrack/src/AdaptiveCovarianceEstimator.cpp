#include "fieldtrack/AdaptiveCovarianceEstimator.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include <boost/foreach.hpp>
#include <sstream>

using namespace argus_msgs;
using namespace argus;

namespace argus
{

AdaptiveTransitionCovarianceEstimator::AdaptiveTransitionCovarianceEstimator() {}

void AdaptiveTransitionCovarianceEstimator::Initialize( ros::NodeHandle& ph )
{
	GetParamRequired( ph, "max_window_samples", _maxSamples );
	GetParam( ph, "min_window_samples", _minSamples, _maxSamples );
	if( _minSamples < 2 )
	{
		throw std::invalid_argument( "Min samples must be at least 2" );
	}

	double dur;
	GetParamRequired( ph, "max_sample_age", dur );
	_maxAge = ros::Duration( dur );

	unsigned int dim;
	GetParamRequired( ph, "dim", dim );
	_initialCov = MatrixType( dim, dim );
	GetParamRequired( ph, "initial_covariance", _initialCov );

	_offset = MatrixType( dim, dim );
	GetParam( ph, "offset", _offset, 1E-3 * MatrixType::Identity( dim, dim ) );

	GetParam( ph, "use_diag", _useDiag, true );
	GetParam( ph, "decay_rate", _decayRate, 1.0 );
	_decayRate = std::log( _decayRate );

	_tx.InitializePushStream( "adaptive_Q", ph, 3, {"x", "y", "w"} );
}

unsigned int AdaptiveTransitionCovarianceEstimator::NumSamples() const
{
	return _innoProds.size();
}

MatrixType AdaptiveTransitionCovarianceEstimator::GetQ( const ros::Time& time )
{
	CheckBuffer( time );
	if( NumSamples() < _minSamples ) { return _initialCov; }

	double wAcc = 0;
	MatrixType Qacc = MatrixType::Zero( _offset.rows(), _offset.cols() );
	for( unsigned int i = 0; i < _innoProds.size(); ++i )
	{
		double t = ( time - _innoProds[i].first ).toSec();
		double w = std::exp( _decayRate * t );
		Qacc += _innoProds[i].second * w;
		wAcc += w;
	}
	// TODO This keeps giving Negative-definite matrices!
	// MatrixType adaptQ = Qacc / ( NumSamples() * wAcc ) + _currSpost - _lastFSpostFT + _offset;
	// NOTE Results in horrible aliasing bugs if we don't make a copy of K transpose...!!
	MatrixType KT = _lastK.transpose();
	// MatrixType adaptQ = _lastK * (Qacc / wAcc) * KT + _offset;
	MatrixType adaptQ = Qacc/wAcc + _offset;
	double timeSpan = ( _innoProds.front().first - _innoProds.back().first ).toSec();
	double averageDt = timeSpan / NumSamples();
	MatrixType adaptQRate = adaptQ / averageDt;

	VectorType feats(3);
	feats << adaptQRate(6,6), adaptQRate(7,7), adaptQRate(11,11);
	_tx.Publish( time, feats );

	// Check for diagonal
	if( _useDiag )
	{
		adaptQRate = Eigen::DiagonalMatrix<double, Eigen::Dynamic>( adaptQRate.diagonal() );
	}
	return adaptQRate;
}

void AdaptiveTransitionCovarianceEstimator::Update( const ros::Time& time,
                                                    const PredictInfo& predict,
                                                    const UpdateInfo& update )
{
	// Initialization catch
	if( _lastFSpostFT.size() == 0 )
	{
		unsigned int dim = update.state_delta.size();
		_currSpost = MatrixType::Zero( dim, dim );
	}

	MatrixType op = update.state_delta * update.state_delta.transpose();
	_innoProds.emplace_front( time, op );

	_lastFSpostFT = predict.trans_jacobian * _currSpost * 
	                predict.trans_jacobian.transpose();
	_currSpost = update.post_state_cov;
	_lastK = update.kalman_gain;
}

void AdaptiveTransitionCovarianceEstimator::Reset()
{
	_lastFSpostFT = MatrixType();
	_currSpost = MatrixType();
	_innoProds.clear();
}

void AdaptiveTransitionCovarianceEstimator::CheckBuffer( const ros::Time& now )
{
	while( !_innoProds.empty() &&
	       ( NumSamples() > _maxSamples || 
	       ( now - _innoProds.back().first ) > _maxAge ) )
	{
		_innoProds.pop_back();
	}
}

AdaptiveObservationCovarianceEstimator::AdaptiveObservationCovarianceEstimator() {}

void AdaptiveObservationCovarianceEstimator::Initialize( ros::NodeHandle& ph )
{
	GetParamRequired( ph, "max_window_samples", _maxSamples );
	GetParam( ph, "min_window_samples", _minSamples, _maxSamples );

	double dur;
	GetParamRequired( ph, "max_sample_age", dur );
	_maxAge = ros::Duration( dur );

	// TODO HACK!!
	_initialCov = MatrixType( 6, 6 );
	GetParamRequired( ph, "initial_covariance", _initialCov );

	unsigned int dim;
	GetParamRequired( ph, "dim", dim );
	_offset = MatrixType( dim, dim );
	GetParam( ph, "offset", _offset, 1E-3 * MatrixType::Identity( dim, dim ) );

	GetParam( ph, "use_diag", _useDiag, true );
	GetParam( ph, "decay_rate", _decayRate, 1.0 );
	_decayRate = std::log( _decayRate );
}

unsigned int AdaptiveObservationCovarianceEstimator::NumSamples() const
{
	return _innoProds.size();
}

MatrixType AdaptiveObservationCovarianceEstimator::GetR( const ros::Time& time )
{
	CheckBuffer( time );
	if( NumSamples() < _minSamples ) { return _initialCov; }

	double wAcc = 0;
	MatrixType Racc = MatrixType::Zero( _offset.rows(), _offset.cols() );
	for( unsigned int i = 0; i < _innoProds.size(); ++i )
	{
		double dt = ( time - _innoProds[i].first ).toSec();
		double w = std::exp( _decayRate * dt );
		Racc += _innoProds[i].second * w;
		wAcc += w;
	}
	MatrixType adaptR = Racc / wAcc + _lastHPHT + _offset;

	// Check for diagonal
	if( _useDiag )
	{
		adaptR = Eigen::DiagonalMatrix<double, Eigen::Dynamic>( adaptR.diagonal() );
	}
	return adaptR;
}

void AdaptiveObservationCovarianceEstimator::Update( const ros::Time& time,
                                                     const UpdateInfo& update )
{
	if( _lastHPHT.size() == 0 )
	{
		unsigned int dim = update.post_obs_error.size();
		_lastHPHT = MatrixType::Zero( dim, dim );
	}

	// Update is R = Cv+ + H * P+ * H^
	_lastHPHT = update.obs_jacobian * update.post_state_cov * 
	            update.obs_jacobian.transpose();
	MatrixType op = update.post_obs_error * update.post_obs_error.transpose();
	_innoProds.emplace_front( time, op );
}

void AdaptiveObservationCovarianceEstimator::Reset()
{
	_innoProds.clear();
	_lastHPHT = MatrixType();
}

void AdaptiveObservationCovarianceEstimator::CheckBuffer( const ros::Time& now )
{
	while( !_innoProds.empty() &&
	       ( NumSamples() > _maxSamples || 
	       ( now - _innoProds.back().first ) > _maxAge ) )
	{
		_innoProds.pop_back();
	}
}

}