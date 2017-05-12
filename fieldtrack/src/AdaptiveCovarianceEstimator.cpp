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

	double dur;
	GetParamRequired( ph, "max_sample_age", dur );
	_maxAge = ros::Duration( dur );

	unsigned int dim;
	GetParamRequired( ph, "dim", dim );
	_priorCov = MatrixType( dim, dim );
	GetParam( ph, "prior_cov", _priorCov, 1E-3 * MatrixType::Identity( dim, dim ) );
	GetParam( ph, "prior_age", _priorAge, 1.0 );
	GetParam( ph, "prior_dt", _priorDt, 1.0 );

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

	double wAcc = std::exp( _decayRate * _priorAge );
	MatrixType Qacc = _priorCov * wAcc;
	for( unsigned int i = 0; i < _innoProds.size(); ++i )
	{
		double t = ( time - _innoProds[i].first ).toSec();
		double w = std::exp( _decayRate * t );
		Qacc += _innoProds[i].second * w;
		wAcc += w;
	}

	// Version using single adjusted state deltas
	// MatrixType adaptQ = Qacc / wAcc + _currSpost - _lastFSpostFT;

	// Version using innovations or pre-adjusted state deltas
	MatrixType adaptQ = Qacc / wAcc;

	double timeSpan;
	if( _innoProds.size() < 2 )
	{
		timeSpan = 0;
	}
	else
	{
		timeSpan = ( _innoProds.front().first - _innoProds.back().first ).toSec();
	}
	timeSpan += _priorDt;
	double averageDt = timeSpan / ( NumSamples() + 1 );
	MatrixType adaptQRate = adaptQ / averageDt;

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

	// Version using state deltas
	// MatrixType op = update.state_delta * update.state_delta.transpose();

	// Version incorporating estimate covariance adjustment
	// MatrixType op = update.state_delta * update.state_delta.transpose() + _currSpost - _lastFSpostFT;

	// Version using innovations
	VectorType Kv = update.kalman_gain * update.prior_obs_error;
	MatrixType op = Kv * Kv.transpose();

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

void AdaptiveObservationCovarianceEstimator::Initialize( unsigned int dim,
                                                         ros::NodeHandle& ph )
{
	GetParamRequired( ph, "max_window_samples", _maxSamples );

	double dur;
	GetParamRequired( ph, "max_sample_age", dur );
	_maxAge = ros::Duration( dur );

	_dim = dim;
	_priorCov = MatrixType( _dim, _dim );
	GetParam( ph, "prior_cov", _priorCov, 1E-3 * MatrixType::Identity( _dim, _dim ) );
	GetParam( ph, "prior_age", _priorAge, 1.0 );
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

	double wAcc = std::exp( _decayRate * _priorAge );
	MatrixType Racc = _priorCov * wAcc;
	for( unsigned int i = 0; i < _innoProds.size(); ++i )
	{
		const InnoStamped& data = _innoProds[i];
		const ros::Time& stamp = data.first;
		const MatrixType& Rhat = data.second;

		double dt = ( time - stamp ).toSec();
		double w = std::exp( _decayRate * dt );
		Racc += Rhat * w;
		wAcc += w;
	}
	MatrixType adaptR = Racc / wAcc;

	// Check for diagonal
	if( _useDiag )
	{
		adaptR = Eigen::DiagonalMatrix<double, Eigen::Dynamic>( adaptR.diagonal() );
	}
	return adaptR;
}

const MatrixType& AdaptiveObservationCovarianceEstimator::GetPriorCov() const
{
	return _priorCov;
}


// MatrixType AdaptiveObservationCovarianceEstimator::GetR( const ros::Time& time,
//                                                          const UpdateInfo& preview )
// {
//  CheckBuffer( time );

//  double wAcc = std::exp( _decayRate * _priorAge );
//  MatrixType Racc = _priorCov * wAcc;
//  for( unsigned int i = 0; i < _innoProds.size(); ++i )
//  {
//      const InnoStamped& data = _innoProds[i];
//      const ros::Time& stamp = data.first;
//      const MatrixType& Rhat = data.second;

//      double dt = ( time - stamp ).toSec();
//      double w = std::exp( _decayRate * dt );
//      Racc += Rhat * w;
//      wAcc += w;
//  }

//  // Add current preview in
//  // Update is R = Cv- + H * P- * H^T
//  MatrixType HPHT = update.obs_jacobian * update.pre_state_cov *
//                    update.obs_jacobian.transpose();
//  MatrixType op = update.pre_obs_error * update.pre_obs_error.transpose();
//  wAcc += 1.0;
//  Racc += op - HPHT;

//  MatrixType adaptR = Racc / wAcc;

//  // Check for diagonal
//  if( _useDiag )
//  {
//      adaptR = Eigen::DiagonalMatrix<double, Eigen::Dynamic>( adaptR.diagonal() );
//  }
//  return adaptR;
// }

void AdaptiveObservationCovarianceEstimator::Update( const ros::Time& time,
                                                     const UpdateInfo& update )
{
	// Update is R = Cv+ + H * P+ * H^T
	MatrixType HPHT = update.obs_jacobian * update.post_state_cov *
	                  update.obs_jacobian.transpose();
	MatrixType op = update.post_obs_error * update.post_obs_error.transpose();
	_innoProds.emplace_front( time, op + HPHT );
}

void AdaptiveObservationCovarianceEstimator::Reset()
{
	_innoProds.clear();
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