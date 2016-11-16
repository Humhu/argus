#include "fieldtrack/StateEstimator.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>
#include <sstream>

namespace argus
{

StateEstimator::StateEstimator() {}

void StateEstimator::Initialize( ros::NodeHandle& ph, 
                                 ExtrinsicsInterface::Ptr extrinsics )
{
	_extrinsicsManager = extrinsics;
	
	GetParamRequired( ph, "reference_frame", _referenceFrame );
	GetParamRequired( ph, "body_frame", _bodyFrame );
	GetParam( ph, 
	          "outlier_likelihood_threshold", 
	          _likelihoodThreshold, 
	          -std::numeric_limits<double>::infinity() );
	GetParam( ph, "two_dimensional", _twoDimensional, false );
	GetParam( ph, "no_pose", _noPose, false );
	GetParam( ph, "max_entropy_threshold", _maxEntropyThreshold, 
	          std::numeric_limits<double>::infinity() );

	unsigned int filterOrder;
	GetParamRequired( ph, "filter_order", filterOrder );
	_filter = PoseDerivativeFilter( filterOrder );

	_initialCovariance = MatrixType( _filter.CovDim(), 
		                             _filter.CovDim() );
	GetParamRequired<double>( ph, "initial_covariance", _initialCovariance );

	std::string mode;
	ros::NodeHandle th( ph.resolveName("transition") );	
	GetParamRequired( th, "mode", mode );
	_transitionMode = StringToCovMode( mode );
	if( _transitionMode == COV_PASS )
	{
		throw std::invalid_argument( "Transition mode cannot be: pass" );
	}
	else if( _transitionMode == COV_FIXED )
	{
		_fixedTransCov = MatrixType( _filter.CovDim(), 
		                             _filter.CovDim() );
		GetParamRequired<double>( th, "covariance", _fixedTransCov );
	}
	else if( _transitionMode == COV_ADAPTIVE )
	{		
		_fixedTransCov = MatrixType( _filter.CovDim(), 
		                             _filter.CovDim() );
		GetParamRequired<double>( th, "initial_covariance", _fixedTransCov );
		_adaptiveTransCov.Initialize( th );
	}

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

	// Reset the transition covariance adapter
	if( _transitionMode == COV_ADAPTIVE )
	{
		_adaptiveTransCov.Reset();
	}

	// Reset all observation covariance adapters
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item& item, _sourceRegistry )
	{
		item.second.Reset();
	}
}

void StateEstimator::Process( const ros::Time& until )
{
	if( until < _filterTime )
	{
		ROS_WARN_STREAM( "Cannot process to time " << until << " as it precedes filter time "
		                 << _filterTime );
		return;
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
		ros::Time timestamp = boost::apply_visitor( ObservationTimestampVisitor(), obs );
		PredictInfo predInfo = PredictUntil( timestamp );

		// Check likelihood after predict
		ObservationLikelihoodVisitor likelihoodCheck( _filter );
		double likelihood = boost::apply_visitor( likelihoodCheck, obs );
		if( likelihood < _likelihoodThreshold )
		{
			ROS_WARN_STREAM( "Rejecting observation from " << sourceName << " with likelihood "
			                 << likelihood << " less than threshold " << _likelihoodThreshold );
		}
		else
		{
			// Perform filter update
			UpdateInfo upInfo = boost::apply_visitor( _filter, obs );

			// Update adaptive estimators if needed
			if( _transitionMode == COV_ADAPTIVE )
			{
				_adaptiveTransCov.Update( predInfo, upInfo );
			}
			_sourceRegistry.at( sourceName ).Update( upInfo );
		}

		_updateBuffer.erase( oldest );
		if( _noPose ) { SquashPose(); }
		if( _twoDimensional ) { Enforce2D(); }
		CheckFilter();
	}

	PredictUntil( until );
	if( _noPose ) { SquashPose(); }
	if( _twoDimensional ) { Enforce2D(); }
	CheckFilter();
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

MatrixType StateEstimator::GetTransitionCov( double dt ) const
{
	if( _transitionMode == COV_FIXED )
	{
		return _fixedTransCov * dt;
	}
	if( _transitionMode == COV_ADAPTIVE )
	{
		return _adaptiveTransCov.IsReady() ? _adaptiveTransCov.GetQ() * dt
		                                   : _fixedTransCov * dt;
	}
	throw std::runtime_error( "Unknown transition mode!" );
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
	return _filter.Predict( GetTransitionCov( dt ), dt );
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
		Dvec.head<6>() = FixedVectorType<6>::Constant( 1.0 );
	}
	if( _twoDimensional )
	{
		for( unsigned int i = 0; i < _filter.CovDim() /6; ++i )
		{
			Dvec.segment( 6*i + 2, 3 ) = FixedVectorType<3>::Ones();
		}
	}

	// LDLT-based entropy computation is more stable
	double entropy = std::log( Dvec.array().prod() );
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
	_filter.FullCov().block( 0, 0, PoseSE3::TangentDimension, _filter.CovDim() ).setZero();
	_filter.FullCov().block( 0, 0, _filter.CovDim(), PoseSE3::TangentDimension ).setZero();
}

}