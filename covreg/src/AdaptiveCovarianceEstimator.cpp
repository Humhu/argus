#include "covreg/AdaptiveCovarianceEstimator.h"
#include <boost/foreach.hpp>
#include "argus_utils/utils/ParamUtils.h"

using namespace argus_msgs;

namespace argus
{

AdaptiveCovarianceEstimator::AdaptiveCovarianceEstimator() 
: _lastDt( 1.0 ) {}

void AdaptiveCovarianceEstimator::Initialize( ros::NodeHandle& ph,
                                              const std::string& field )
{
	GetParamRequired( ph, field + "/window_length", _windowLength );
}

MatrixType AdaptiveCovarianceEstimator::GetQ() const
{
	MatrixType acc = MatrixType::Zero( _currSpost.rows(), _currSpost.cols() );
	BOOST_FOREACH( const MatrixType& op, _delXOuterProds )
	{
		acc += op;
	}
	MatrixType sum = acc / _delXOuterProds.size() + _currSpost + _lastFSpostFT + _offset;
	return Eigen::DiagonalMatrix<double,Eigen::Dynamic,Eigen::Dynamic>( sum.diagonal() );
}

void AdaptiveCovarianceEstimator::ProcessInfo( const FilterStepInfo& msg )
{
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

bool AdaptiveCovarianceEstimator::IsReady() const
{
	return _delXOuterProds.size() >= _windowLength;
}

}