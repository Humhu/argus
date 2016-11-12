#include "fieldtrack/StateEstimator.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>
#include <sstream>

namespace argus
{

CovarianceMode StringToCovMode( const std::string& str )
{
	if( str == "pass" ) { return COV_PASS; }
	if( str == "fixed" ) { return COV_FIXED; }
	if( str == "adaptive" ) { return COV_ADAPTIVE; }
	else
	{
		throw std::invalid_argument( "Unknown covariance mode: " + str );
	}
}

std::string CovModeToString( CovarianceMode mode )
{
	if( mode == COV_PASS ) { return "pass"; }
	if( mode == COV_FIXED ) { return "fixed"; }
	if( mode == COV_ADAPTIVE ) { return "adaptive"; }
	else
	{
		throw std::invalid_argument( "Unknown covariance mode: " + mode );
	}
}

StateEstimator::StateEstimator() {}

void StateEstimator::Initialize( ros::NodeHandle& ph )
{
	GetParamRequired( ph, "reference_frame", _referenceFrame );
	GetParamRequired( ph, "body_frame", _bodyFrame );
	GetParam( ph, 
	          "outlier_likelihood_threshold", 
	          _likelihoodThreshold, 
	          -std::numeric_limits<double>::infinity() );
	GetParam( ph, "two_dimensional", _twoDimensional, false );
	GetParam( ph, "no_pose", _noPose, false );

	_initialCovariance = MatrixType( FilterType::CovarianceDim, 
		                             FilterType::CovarianceDim );
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
		_fixedTransCov = MatrixType( FilterType::CovarianceDim, 
		                             FilterType::CovarianceDim );
		GetParamRequired<double>( th, "covariance", _fixedTransCov );
	}
	else if( _transitionMode == COV_ADAPTIVE )
	{
		_adaptiveTransCov.Initialize( th );
	}

	// Parse all update sources
	YAML::Node updateSources;
	GetParamRequired( ph, "update_sources", updateSources );
	YAML::Node::const_iterator iter;
	for( iter = updateSources.begin(); iter != updateSources.end(); iter++ )
	{
		const std::string& sourceName = iter->first.as<std::string>();
		const YAML::Node& info = iter->second;
		SourceRegistration& reg = _obsRegistry[sourceName];

		GetParamRequired( info, "mode", mode );
		reg.mode = StringToCovMode( mode );
		GetParamRequired( info, "dim", reg.dim );

		if( reg.mode == COV_PASS )
		{
			// Nothing to parse
		}
		else if( reg.mode == COV_FIXED )
		{
			reg.fixedCov = MatrixType( reg.dim, reg.dim );
			GetParamRequired<double>( info, "covariance", reg.fixedCov );
		}
		else if( reg.mode == COV_ADAPTIVE )
		{
			ros::NodeHandle ah( ph.resolveName( "update_sources/" + sourceName ) );
			reg.adaptiveCov.Initialize( ah );
		}
	}
}

void StateEstimator::Reset( const ros::Time& time )
{
	// Reset the filter state
	_filter.Pose() = FilterType::PoseType();
	_filter.Derivs() = FilterType::DerivsType::Zero();
	_filter.FullCov() = _initialCovariance;
	_filterTime = time;

	// Reset the transition covariance adapter
	if( _transitionMode == COV_ADAPTIVE )
	{
		_adaptiveTransCov.Reset();
	}

	// Reset all observation covariance adapters
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item& item, _obsRegistry )
	{
		if( item.second.mode == COV_ADAPTIVE )
		{
			item.second.adaptiveCov.Reset();
		}
	}
}

void StateEstimator::BufferUpdate( FilterUpdate update )
{
	if( _obsRegistry.count( update.sourceName ) == 0 )
	{
		throw std::invalid_argument( "Update from unknown source: " + update.sourceName );
	}
	if( update.timestamp < _filterTime )
	{
		std::stringstream ss;
		ss << "Update time " << update.timestamp << " predates filter time " << _filterTime;
		throw std::invalid_argument( ss.str() );
	}

	// For some reason the resolution on the timestamp compare is limited
	// We have to avoid overwriting observations this way
	while( _updateBuffer.count(update.timestamp) > 0 )
	{
		update.timestamp.nsec += 2; 
	}
	_updateBuffer[update.timestamp] = update;
}

void StateEstimator::Process( const ros::Time& until )
{
	while( !_updateBuffer.empty() )
	{
		UpdateBuffer::const_iterator oldest = _updateBuffer.begin();
		const ros::Time& earliestBufferTime = oldest->first;
		if( earliestBufferTime > until ) { break; }
		
		FilterUpdate update = oldest->second;
		 ProcessUpdate( update );
		_updateBuffer.erase( oldest );

		if( _noPose ) { SquashPose(); }
		if( _twoDimensional ) { Enforce2D(); }
		CheckFilter();
	}
}

TargetState StateEstimator::GetState() const
{
	TargetState state;
	state.referenceFrame = _referenceFrame;
	state.bodyFrame = _bodyFrame;
	state.timestamp = _filterTime;
	state.pose = _filter.Pose();
	state.poseCovariance  = _filter.PoseCov();
	state.velocity = _filter.Derivs().head<FilterType::TangentDim>();
	state.velocityCovariance = _filter.DerivsCov().topLeftCorner<FilterType::TangentDim,
	                                                             FilterType::TangentDim>();
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
		return _adaptiveTransCov.GetQ() * dt;
	}
	throw std::runtime_error( "Unknown transition mode!" );
}

MatrixType StateEstimator::GetObservationCov( const FilterUpdate& update ) const
{
	const SourceRegistration& reg = _obsRegistry.at( update.sourceName );
	if( reg.mode == COV_PASS )
	{
		return update.observationCov;
	}
	if( reg.mode == COV_FIXED )
	{
		return reg.fixedCov;
	}
	if( reg.mode == COV_ADAPTIVE )
	{
		return reg.adaptiveCov.GetR();
	}
	throw std::runtime_error( "Unknown observation mode for: " + update.sourceName );
}

PredictInfo StateEstimator::PredictUntil( const ros::Time& until )
{
	double dt = (until - _filterTime).toSec();
	if( dt < 0 )
	{
		throw std::runtime_error( "Negative dt predict requested." );
	}
	_filterTime = until;
	return _filter.Predict( GetTransitionCov( dt ), dt );
}

void StateEstimator::ProcessUpdate( FilterUpdate update )
{
	// Forward predict to the observation time
	PredictInfo predInfo = PredictUntil( update.timestamp );

	// Compute the observation covariance
	update.observationCov = GetObservationCov( update );

	// Figure out what kind of observation this is
	// TODO Case where there are no derivs in the filter?
	const MatrixType& C = update.observationCov;
	bool hasPosition = !( C.block( 0, 0, C.rows(), 2 ).array() == 0 ).all();
	bool hasOrientation = !( C.col(2).array() == 0 ).all();
	bool hasDerivs = !( C.rightCols( C.cols() - FilterType::TangentDim ).array() == 0 ).all();

	UpdateInfo upInfo;
	if( hasPosition && !hasOrientation && !hasDerivs )
	{
		throw std::runtime_error( "Position only updates are not supported yet." );
	}
	else if( !hasPosition && hasOrientation && !hasDerivs )
	{
		throw std::runtime_error( "Orientation only updates are not supported yet." );
	}
	else if( hasPosition && hasOrientation && !hasDerivs )
	{
		if( !PoseUpdate( update, upInfo ) ) { return; }
	}
	else if( !hasPosition && !hasOrientation && hasDerivs )
	{
		if( !DerivsUpdate( update, upInfo ) ) { return; }
	}
	else if( hasPosition && hasOrientation && hasDerivs )
	{
		// PoseType pose;
		// pose.FromVector( z.head(7) );
		// VectorType derivs = z.tail( z.size() - 7 );
		// info = JointUpdate( pose, derivs, C, R );
		throw std::runtime_error( "Joint update not supported yet." );
	}
	else
	{
		throw std::runtime_error( "Unsupported update combination." );
	}

	// TODO Update covariances before updating?
	if( _transitionMode == COV_ADAPTIVE )
	{
		_adaptiveTransCov.Update( predInfo, upInfo );
	}
	if( _obsRegistry[ update.sourceName ].mode == COV_ADAPTIVE )
	{
		_obsRegistry[ update.sourceName ].adaptiveCov.Update( upInfo );
	}
}

bool StateEstimator::PoseUpdate( const FilterUpdate& update,
                                 UpdateInfo& info )
{
	// TODO Check for pose outliers
	info = _filter.UpdatePose( PoseType( update.observation ), 
	                           update.observationCov );
	return true;
}

bool StateEstimator::DerivsUpdate( const FilterUpdate& update,
                                   UpdateInfo& info )
{
	const MatrixType& C = update.observationMatrix;
	FilterType::DerivObsMatrix Cderivs = C.rightCols( C.cols() - FilterType::TangentDim );

	// Check for outliers
	// TODO Different thresholds for different sources?
	double prob = _filter.DerivsLikelihood( update.observation, Cderivs, update.observationCov );
	if( prob < _likelihoodThreshold )
	{
		ROS_WARN_STREAM( "DervisUpdate: Rejecting outlier with likelihood: " << prob );
		return false;
	}

	info = _filter.UpdateDerivs( update.observation, Cderivs, update.observationCov );
	return true;
}

void StateEstimator::CheckFilter()
{
	// Its expensive to validate the eigenvalues or determinant, but we can
	// check the LDLT relatively quickly
	Eigen::LDLT<FilterType::FullCovType> ldlt( _filter.FullCov() );
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
		for( unsigned int i = 0; i < FilterType::CovarianceDim/6; ++i )
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

	FilterType::DerivsType derivs = _filter.Derivs();
	for( unsigned int i = 0; i < FilterType::CovarianceDim/6; ++i )
	{
		_filter.FullCov().block( 6*i+2, 0, 3, FilterType::CovarianceDim ).setZero();
		_filter.FullCov().block( 0, 6*i+2, FilterType::CovarianceDim, 3 ).setZero();
	}
	for( unsigned int i = 0; i < FilterType::CovarianceDim/6 - 1; ++i )
	{
		derivs.segment( (6*i) + 2, 3 ).setZero();
	}
	_filter.Derivs() = derivs;
}

void StateEstimator::SquashPose()
{
	_filter.Pose() = FilterType::PoseType();
	_filter.FullCov().block(0,0,FilterType::TangentDim,FilterType::CovarianceDim).setZero();
	_filter.FullCov().block(0,0,FilterType::CovarianceDim,FilterType::TangentDim).setZero();
}

}