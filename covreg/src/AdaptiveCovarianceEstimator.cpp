#include "covreg/AdaptiveCovarianceEstimator.h"
#include <boost/foreach.hpp>
#include "argus_utils/utils/ParamUtils.h"

using namespace argus_msgs;

namespace argus
{

AdaptiveTransitionCovarianceEstimator::AdaptiveTransitionCovarianceEstimator() 
: _lastDt( 1.0 ) {}

void AdaptiveTransitionCovarianceEstimator::Initialize( ros::NodeHandle& ph,
                                              const std::string& field )
{
	GetParamRequired( ph, field + "/window_length", _windowLength );
}

MatrixType AdaptiveTransitionCovarianceEstimator::GetQ() const
{
	MatrixType acc = MatrixType::Zero( _currSpost.rows(), _currSpost.cols() );
	BOOST_FOREACH( const MatrixType& op, _delXOuterProds )
	{
		acc += op;
	}
	MatrixType sum = acc / _delXOuterProds.size() + _currSpost + _lastFSpostFT + _offset;
	return Eigen::DiagonalMatrix<double,Eigen::Dynamic,Eigen::Dynamic>( sum.diagonal() );
}

void AdaptiveTransitionCovarianceEstimator::ProcessInfo( const FilterStepInfo& msg )
{
	// NOTE We should always have a predict first
	if( msg.isPredict )
	{
		PredictInfo info = MsgToPredict( msg );
		_lastF = info.F;
		_lastDt = info.dt;
	}
	else
	{
		UpdateInfo info = MsgToUpdate( msg );
		MatrixType op = info.delta_x * info.delta_x.transpose() / _lastDt;
		_delXOuterProds.push_back( op );
		while( _delXOuterProds.size() > _windowLength )
		{
			_delXOuterProds.pop_front();
		}

		_lastFSpostFT = _lastF * _currSpost * _lastF.transpose();
		_currSpost = info.Spost;

		if( _offset.size() == 0 )
		{
			_offset = 1E-9 * MatrixType::Identity( _currSpost.rows(), _currSpost.cols() );
		}
	}

}

bool AdaptiveTransitionCovarianceEstimator::IsReady() const
{
	return _delXOuterProds.size() >= _windowLength;
}

void AdaptiveTransitionCovarianceEstimator::Reset()
{
	_delXOuterProds.clear();
}


AdaptiveObservationCovarianceEstimator::AdaptiveObservationCovarianceEstimator() {}

void AdaptiveObservationCovarianceEstimator::Initialize( ros::NodeHandle& ph, const std::string& field )
{
	unsigned int dim;
	GetParamRequired<unsigned int>( ph, field + "/dim", dim );
	GetParamRequired( ph, field + "/window_length", _windowLength );

	_initCov = MatrixType::Identity( dim, dim );
	if( !GetMatrixParam<double>( ph, field + "/initial_cov", _initCov ) )
	{
		if( !GetDiagonalParam<double>( ph, field + "/initial_cov", _initCov ) )
		{
			ROS_WARN_STREAM( "No initial covariance specified. Using identity.");
		}
	}
}

bool AdaptiveObservationCovarianceEstimator::IsReady() const
{
	return _innoOuterProds.size() >= _windowLength;
}

MatrixType AdaptiveObservationCovarianceEstimator::GetR() const
{
	if( !IsReady() ) { return _initCov; }

	MatrixType acc = MatrixType::Zero( _lastHPHT.rows(), _lastHPHT.cols() );
	BOOST_FOREACH( const MatrixType& op, _innoOuterProds )
	{
		acc += op;
	}
	MatrixType sum = acc/_innoOuterProds.size() + _lastHPHT;
	return Eigen::DiagonalMatrix<double,Eigen::Dynamic,Eigen::Dynamic>( sum.diagonal() );
}

void AdaptiveObservationCovarianceEstimator::ProcessInfo( const argus_msgs::FilterStepInfo& msg )
{
	// Update is R = Cv+ + H * P+ * H^
	UpdateInfo info = MsgToUpdate( msg );
	_lastHPHT = info.H * info.Spost * info.H.transpose();

	MatrixType op = info.post_innovation * info.post_innovation.transpose();
	_innoOuterProds.push_back( op );
	while( _innoOuterProds.size() > _windowLength )
	{
		_innoOuterProds.pop_front();
	}
}

void AdaptiveObservationCovarianceEstimator::Reset()
{
	_innoOuterProds.clear();
}

}