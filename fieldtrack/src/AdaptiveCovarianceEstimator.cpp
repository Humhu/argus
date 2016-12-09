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

	double dur;
	GetParamRequired( ph, "max_sample_age", dur );
	_maxAge = ros::Duration( dur );

	unsigned int dim;
	GetParamRequired( ph, "dim", dim );
	_initialCov = MatrixType( dim, dim );
	GetParamRequired( ph, "initial_covariance", _initialCov );

	GetParam( ph, "use_diag", _useDiag, true );
	GetParam( ph, "decay_rate", _decayRate, 1.0 );
	_decayRate = std::log( _decayRate );
}

unsigned int AdaptiveTransitionCovarianceEstimator::NumSamples() const
{
	return _innoProds.size();
}

MatrixType AdaptiveTransitionCovarianceEstimator::GetQ( const ros::Time& time )
{
	CheckBuffer( time );
	if( NumSamples() < _minSamples ) { return _initialCov; }

	ros::Time startTime = _innoProds[0].first;
	double wAcc = std::exp( 0 );
	MatrixType Qacc = std::exp( 0 ) * _innoProds[0].second;
	for( unsigned int i = 1; i < _innoProds.size(); ++i )
	{
		double dt = ( startTime - _innoProds[i].first ).toSec();
		double w = std::exp( _decayRate * dt );
		Qacc += _innoProds[i].second * w;
		wAcc += w;
	}
	double timeSpan = ( _innoProds.front().first - _innoProds.back().first ).toSec();
	double avgDt = timeSpan / NumSamples();
	MatrixType adaptQ = Qacc / ( wAcc * avgDt ) + _currSpost + _lastFSpostFT;

	// Check for diagonal
	if( _useDiag )
	{
		adaptQ = Eigen::DiagonalMatrix<double, Eigen::Dynamic>( adaptQ.diagonal() );
	}
	return adaptQ;
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
		_lastFSpostFT = MatrixType::Zero( dim, dim );
	}

	MatrixType op = update.state_delta * update.state_delta.transpose();
	_innoProds.emplace_front( time, op );

	_lastFSpostFT = predict.trans_jacobian * _currSpost * 
	                predict.trans_jacobian.transpose();
	_currSpost = update.post_state_cov;
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

	unsigned int dim;
	GetParamRequired( ph, "dim", dim );
	_initialCov = MatrixType( dim, dim );
	GetParamRequired( ph, "initial_covariance", _initialCov );

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

	ros::Time startTime = _innoProds[0].first;
	double wAcc = std::exp( 0 );
	MatrixType Racc = std::exp( 0 ) * _innoProds[0].second;
	for( unsigned int i = 1; i < _innoProds.size(); ++i )
	{
		double dt = ( startTime - _innoProds[i].first ).toSec();
		double w = std::exp( _decayRate * dt );
		Racc += _innoProds[i].second * w;
		wAcc += w;
	}
	MatrixType adaptR = Racc / wAcc + _lastHPHT;

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