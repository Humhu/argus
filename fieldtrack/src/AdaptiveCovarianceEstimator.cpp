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
	GetParamRequired( ph, "window_length", _windowLength );
	GetParam( ph, "min_window_ready", _minLength, _windowLength );
	GetParam( ph, "use_diag", _useDiag, true );

	double decayRate;
	GetParam( ph, "decay_rate", decayRate, 1.0 );
	
	_prodWeights = VectorType( _windowLength );
	_prodWeights(0) = 1.0;
	for( unsigned int i = 1; i < _windowLength; ++i )
	{
		_prodWeights(i) = _prodWeights(i-1) * decayRate;
	}
	_prodWeights = _prodWeights / _prodWeights.sum();
}

bool AdaptiveTransitionCovarianceEstimator::IsReady() const
{
	return _delXOuterProds.size() >= _minLength;
}

MatrixType AdaptiveTransitionCovarianceEstimator::GetQ() const
{
	if( !IsReady() ) 
	{
		throw std::runtime_error("Trasition covariance estimator not ready." );
	}

	MatrixType acc = MatrixType::Zero( _currSpost.rows(), _currSpost.cols() );
	for( unsigned int i = 0; i < _delXOuterProds.size(); i++ )
	{
		acc += _delXOuterProds[i] * _prodWeights(i);
	}
	MatrixType adaptQ = acc + _currSpost + _lastFSpostFT;

	// Check for diagonal
	if( _useDiag )
	{
		adaptQ = Eigen::DiagonalMatrix<double, Eigen::Dynamic>( adaptQ.diagonal() );
	}
	return adaptQ;
}

void AdaptiveTransitionCovarianceEstimator::Update( const PredictInfo& predict,
                                                    const UpdateInfo& update )
{
	// Initialization catch
	if( _lastFSpostFT.size() == 0 )
	{
		unsigned int dim = update.delta_x.size();
		_currSpost = MatrixType::Zero( dim, dim );
		_lastFSpostFT = MatrixType::Zero( dim, dim );
	}

	// TODO This should really be time since last update
	MatrixType op = update.delta_x * update.delta_x.transpose() / predict.dt;
	_delXOuterProds.push_front( op );
	while( _delXOuterProds.size() > _windowLength )
	{
		_delXOuterProds.pop_back();
	}

	_lastFSpostFT = predict.F * _currSpost * predict.F.transpose();
	_currSpost = update.Spost;
}

void AdaptiveTransitionCovarianceEstimator::Reset()
{
	_lastFSpostFT = MatrixType();
	_currSpost = MatrixType();
	_delXOuterProds.clear();
}

AdaptiveObservationCovarianceEstimator::AdaptiveObservationCovarianceEstimator() {}

void AdaptiveObservationCovarianceEstimator::Initialize( ros::NodeHandle& ph )
{
	GetParamRequired( ph, "window_length", _windowLength );
	GetParam( ph, "min_window_ready", _minLength, _windowLength );
	GetParam( ph, "use_diag", _useDiag, true );

	double decayRate;
	GetParam( ph, "decay_rate", decayRate, 1.0 );
	
	_prodWeights = VectorType( _windowLength );
	_prodWeights(0) = 1.0;
	for( unsigned int i = 1; i < _windowLength; ++i )
	{
		_prodWeights(i) = _prodWeights(i-1) * decayRate;
	}
	_prodWeights = _prodWeights / _prodWeights.sum();
}

bool AdaptiveObservationCovarianceEstimator::IsReady() const
{
	return _innoOuterProds.size() >= _minLength;
}

MatrixType AdaptiveObservationCovarianceEstimator::GetR() const
{
	if( !IsReady() )
	{
		throw std::runtime_error( "Obseration covariance estimator is not ready." );
	}

	MatrixType acc = MatrixType::Zero( _lastHPHT.rows(), _lastHPHT.cols() );
	for( unsigned int i = 0; i < _innoOuterProds.size(); ++i )
	{
		acc += _innoOuterProds[i] * _prodWeights(i);
	}
	MatrixType adaptR = acc + _lastHPHT;
	
	// Check for diagonal
	if( _useDiag )
	{
		adaptR = Eigen::DiagonalMatrix<double, Eigen::Dynamic>( adaptR.diagonal() );
	}
	return adaptR;
}

void AdaptiveObservationCovarianceEstimator::Update( const UpdateInfo& update )
{
	if( _lastHPHT.size() == 0 )
	{
		unsigned int dim = update.post_innovation.size();
		_lastHPHT = MatrixType::Zero( dim, dim );
	}

	// Update is R = Cv+ + H * P+ * H^
	_lastHPHT = update.H * update.Spost * update.H.transpose();
	MatrixType op = update.post_innovation * update.post_innovation.transpose();
	_innoOuterProds.push_front( op );

	// Remove old innovations
	while( _innoOuterProds.size() > _windowLength )
	{
		_innoOuterProds.pop_back();
	}
}

void AdaptiveObservationCovarianceEstimator::Reset()
{
	_innoOuterProds.clear();
	_lastHPHT = MatrixType();
}

}