#include "fieldtrack/StateEstimator.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>
#include <sstream>

#define POSE_DIM (PoseSE3::TangentDimension)

namespace argus
{

StateEstimator::StateEstimator() {}

void StateEstimator::Initialize( ros::NodeHandle& ph, 
                                 ExtrinsicsInterface::Ptr extrinsics )
{
	_extrinsicsManager = extrinsics;
	_stepCounter = 0;
	
	GetParamRequired( ph, "reference_frame", _referenceFrame );
	GetParamRequired( ph, "body_frame", _bodyFrame );
	GetParam( ph, 
	          "outlier_likelihood_threshold", 
	          _likelihoodThreshold, 
	          -std::numeric_limits<double>::infinity() );
	GetParam( ph, "two_dimensional", _twoDimensional, false );
	GetParam( ph, "no_pose", _noPose, false );
	GetParam( ph, "no_derivs", _noDerivs, false );
	GetParam( ph, "max_entropy_threshold", _maxEntropyThreshold, 
	          std::numeric_limits<double>::infinity() );

	unsigned int filterOrder;
	GetParamRequired( ph, "filter_order", filterOrder );
	_filter = PoseDerivativeFilter( filterOrder );

	_initialCovariance = MatrixType( _filter.CovDim(), 
		                             _filter.CovDim() );
	GetParamRequired( ph, "initial_covariance", _initialCovariance );

	std::string mode;
	ros::NodeHandle th( ph.resolveName("transition") );	
	_transCovRate = MatrixType( _filter.CovDim(), _filter.CovDim() );
	GetParamRequired( th, "covariance", _transCovRate );

	// Parse all update sources
	YAML::Node updateSources;
	GetParamRequired( ph, "update_sources", updateSources );
	YAML::Node::const_iterator iter;
	for( iter = updateSources.begin(); iter != updateSources.end(); iter++ )
	{
		const std::string& sourceName = iter->first.as<std::string>();
		ros::NodeHandle sh = ph.resolveName( "update_sources/" + sourceName );
		_sourceRegistry.emplace( std::piecewise_construct,
		                         std::forward_as_tuple( sourceName ), 
		                         std::forward_as_tuple( sh, 
		                                                _bodyFrame, 
		                                                _referenceFrame, 
		                                                _extrinsicsManager ) );
	}
}

void StateEstimator::Reset( const ros::Time& time )
{
	// Reset the filter state
	_filter.Pose() = PoseSE3();
	_filter.Derivs().setZero();
	_filter.FullCov() = _initialCovariance;
	_filterTime = time;

	_updateBuffer.clear();

	// Reset all observation covariance adapters
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item& item, _sourceRegistry )
	{
		item.second.Reset();
	}
}

std::vector<FilterInfo> StateEstimator::Process( const ros::Time& until )
{
	std::vector<FilterInfo> infos;
	if( until < _filterTime )
	{
		ROS_WARN_STREAM( "Cannot process to time " << until << " as it precedes filter time "
		                 << _filterTime );
		return infos;
	}

	while( !_updateBuffer.empty() )
	{
		UpdateBuffer::const_iterator oldest = _updateBuffer.begin();
		const ros::Time& earliestBufferTime = oldest->first;
		if( earliestBufferTime > until ) { break; }
		
		const SourceMsg& sourceMsg = oldest->second;
		const std::string& sourceName = sourceMsg.first;
		const ObservationMessage& msg = sourceMsg.second;

		// Perform the predict and update
		Observation obs = boost::apply_visitor( _sourceRegistry.at( sourceName ), msg );
		ros::Time timestamp = get_observation_timestamp( obs );
		std::string frame = get_observation_frame( obs );
		PredictInfo predInfo = PredictUntil( timestamp );
		predInfo.time = _filterTime;
		predInfo.frameId = "predict";
		predInfo.stepNum = _stepCounter;
		infos.emplace_back( predInfo );

		// Check likelihood after predict
		ObservationLikelihoodVisitor likelihoodCheck( _filter );
		double likelihood = boost::apply_visitor( likelihoodCheck, obs );
		if( std::isnan(likelihood) )
		{
			ROS_WARN_STREAM( "Likelihood is nan!" );
		}
		else if( likelihood < _likelihoodThreshold )
		{
			ROS_WARN_STREAM( "Rejecting observation from " << sourceName << " with likelihood "
			                 << likelihood << " less than threshold " << _likelihoodThreshold );
		}
		else
		{
			// Perform filter update
			UpdateInfo upInfo = boost::apply_visitor( _filter, obs );
			upInfo.time = _filterTime;
			upInfo.frameId = frame;
			upInfo.stepNum = _stepCounter;
			infos.emplace_back( upInfo );

			_sourceRegistry.at( sourceName ).Update( timestamp, upInfo );
		}

		_updateBuffer.erase( oldest );

		// Check filter while processing
		if( _noPose ) { SquashPose(); }
		if( _noDerivs ) { SquashDerivs(); }
		if( _twoDimensional ) { Enforce2D(); }
		CheckFilter();
	}

	// Predict the remainder of requested time
	PredictInfo predInfo = PredictUntil( until );
	infos.emplace_back( predInfo );

	// Have to check after final predict
	if( _noPose ) { SquashPose(); }
	if( _noDerivs ) { SquashDerivs(); }
	if( _twoDimensional ) { Enforce2D(); }
	CheckFilter();

	return infos;
}

TargetState StateEstimator::GetState() const
{
	TargetState state;
	state.referenceFrame = _referenceFrame;
	state.bodyFrame = _bodyFrame;
	state.timestamp = _filterTime;
	state.pose = _filter.Pose();
	state.derivatives = _filter.Derivs();
	state.covariance = _filter.FullCov();
	return state;
}

const MatrixType& StateEstimator::GetTransitionCovRate() const
{
	return _transCovRate;
}

void StateEstimator::SetTransitionCovRate( const MatrixType& Q )
{
	_transCovRate = Q;
}

MatrixType StateEstimator::GetTransitionCov( const ros::Time& time,
                                             double dt )
{
	return _transCovRate * dt;
}

PredictInfo StateEstimator::PredictUntil( const ros::Time& until )
{
	double dt = (until - _filterTime).toSec();
	if( dt < 0 )
	{
		std::stringstream ss;
		ss << "Predict to " << until << " requested from " << _filterTime;
		throw std::runtime_error( ss.str() );
	}
	_filterTime = until;
	return _filter.Predict( GetTransitionCov( until, dt ), dt );
}

void StateEstimator::CheckFilter()
{
	// Its expensive to validate the eigenvalues or determinant, but we can
	// check the LDLT relatively quickly
	Eigen::LDLT<MatrixType> ldlt( _filter.FullCov() );
	if( ldlt.info() != Eigen::Success )
	{
		ROS_ERROR_STREAM( "Filter covariance is not PD. Resetting." );
		Reset( _filterTime );
	}

	VectorType Dvec = ldlt.transpositionsP().transpose() * ldlt.vectorD();
	if( _noPose )
	{
		Dvec.head<POSE_DIM>() = FixedVectorType<6>::Constant( 1.0 );
	}
	if( _noDerivs )
	{
	  Dvec.tail( Dvec.size() - POSE_DIM ) = VectorType::Constant( Dvec.size() - POSE_DIM, 1, 1.0 );
	}
	if( _twoDimensional )
	{
		for( unsigned int i = 0; i < _filter.CovDim() /6; ++i )
		{
			Dvec.segment( 6*i + 2, 3 ) = FixedVectorType<3>::Ones();
		}
	}

	// LDLT-based entropy computation is more stable
	double entropy = Dvec.array().log().sum();
	if( entropy > _maxEntropyThreshold )
	{
		ROS_WARN_STREAM( "Filter entropy: " << entropy << " greater than max: " << 
		                 _maxEntropyThreshold << " Resetting filter..." );
		Reset( _filterTime );
	}
}

void StateEstimator::Enforce2D()
{
	FixedVectorType<7> poseVector = _filter.Pose().ToVector();
	PoseSE3 mean( poseVector(0), poseVector(1), 0, poseVector(3), 0, 0, poseVector(6) );
	_filter.Pose() = mean;

	VectorType derivs = _filter.Derivs();
	for( unsigned int i = 0; i < _filter.CovDim()/6; ++i )
	{
		_filter.FullCov().block( 6*i+2, 0, 3, _filter.CovDim() ).setZero();
		_filter.FullCov().block( 0, 6*i+2, _filter.CovDim(), 3 ).setZero();
	}
	for( unsigned int i = 0; i < _filter.CovDim()/6 - 1; ++i )
	{
		derivs.segment( (6*i) + 2, 3 ).setZero();
	}
	_filter.Derivs() = derivs;
}

void StateEstimator::SquashPose()
{
	_filter.Pose() = PoseSE3();
	_filter.PoseCov().setZero();
	_filter.FullCov().block( 0, POSE_DIM, POSE_DIM, _filter.CovDim() - POSE_DIM ).setZero();
	_filter.FullCov().block( POSE_DIM, 0, _filter.CovDim() - POSE_DIM, POSE_DIM ).setZero();
}

  void StateEstimator::SquashDerivs()
  {
    _filter.Derivs().setZero();
    _filter.DerivsCov().setZero();
    _filter.FullCov().block( 0, POSE_DIM, POSE_DIM, _filter.CovDim() - POSE_DIM ).setZero();
    _filter.FullCov().block( POSE_DIM, 0, _filter.CovDim() - POSE_DIM, POSE_DIM ).setZero();
  }

}
