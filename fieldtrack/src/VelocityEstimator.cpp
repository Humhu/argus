#include "fieldtrack/VelocityEstimator.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/random/MultivariateGaussian.hpp"
#include "argus_utils/utils/MathUtils.h"

#include <boost/foreach.hpp>
#include <sstream>

namespace argus
{
VelocityEstimator::VelocityEstimator() {}

void VelocityEstimator::Initialize( ros::NodeHandle& ph,
                                    ExtrinsicsInterface::Ptr extrinsics )
{
	_extrinsicsManager = extrinsics;
	_stepCounter = 0;

	GetParamRequired( ph, "body_frame", _bodyFrame );
	GetParam( ph, "log_likelihood_threshold",
	          _logLikelihoodThreshold,
	          -std::numeric_limits<double>::infinity() );
	GetParam( ph, "two_dimensional", _twoDimensional, false );
	GetParam( ph, "max_entropy_threshold", _maxEntropyThreshold,
	          std::numeric_limits<double>::infinity() );

	GetParamRequired( ph, "filter_order", _filterOrder );

	_initialCovariance = MatrixType( FullDim(), FullDim() );
	GetParamRequired( ph, "initial_covariance", _initialCovariance );
	_filter.Initialize( VectorType::Zero( FullDim() ), _initialCovariance );

	_transCovRate = MatrixType( FullDim(), FullDim() );
	GetParamRequired( ph, "transition_covariance", _transCovRate );

	// Parse all update sources
	YAML::Node updateSources;
	GetParamRequired( ph, "update_sources", updateSources );
	YAML::Node::const_iterator iter;
	for( iter = updateSources.begin(); iter != updateSources.end(); iter++ )
	{
		const std::string& sourceName = iter->first.as<std::string>();
		ros::NodeHandle sh = ph.resolveName( "update_sources/" + sourceName );
		if( _sourceRegistry.count( sourceName ) > 0 )
		{
			throw std::invalid_argument( "Source " + sourceName + " already registered!" );
		}
		_sourceRegistry[sourceName].Initialize( sh, _twoDimensional,
		                                        _filterOrder, _bodyFrame,
		                                        _extrinsicsManager );
	}
}

CovarianceModel::Ptr VelocityEstimator::InitTransCovModel() const
{
	TimeScaledCovariance::Ptr cov = std::make_shared<TimeScaledCovariance>();
	cov->Initialize( _transCovRate );
	return cov;
}

std::unordered_map<std::string, CovarianceModel::Ptr>
VelocityEstimator::InitObsCovModels() const
{
	std::unordered_map<std::string, CovarianceModel::Ptr> out;
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( const Item& item, _sourceRegistry )
	{
		const std::string& name = item.first;
		const VelocitySourceManager& manager = item.second;
		out[name] = manager.InitializeModel();
	}
	return out;
}


void VelocityEstimator::SetTransCovModel( const CovarianceModel& model )
{
	// TODO Different transition covariance modes
	try
	{
		const FixedCovariance& mod = dynamic_cast<const FixedCovariance&>( model );
		_transCovRate = mod.GetValue();
		ROS_INFO_STREAM( "Transition covariance rate updated to: " << std::endl << _transCovRate );
	}
	catch( std::bad_cast& e )
	{
		throw std::invalid_argument( "Transition cov type mismatch: " + std::string( e.what() ) );
	}
}

void VelocityEstimator::SetObsCovModel( const std::string& name,
                                        const CovarianceModel& model )
{
	if( _sourceRegistry.count( name ) == 0 )
	{
		throw std::invalid_argument( "Source " + name + " not registered!" );
	}
	_sourceRegistry[name].SetModel( model );
}

unsigned int VelocityEstimator::StateDim() const
{
	return _twoDimensional ? PoseSE2::TangentDimension :
	       PoseSE3::TangentDimension;
}

unsigned int VelocityEstimator::FullDim() const
{
	return (_filterOrder + 1) * StateDim();
}

void VelocityEstimator::Reset( const ros::Time& time )
{
	// Reset the filter state
	_filter.Initialize( VectorType::Zero( FullDim() ), _initialCovariance );
	_filterTime = time;

	_updateBuffer.clear();

	// Reset all observation covariance adapters
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item & item, _sourceRegistry )
	{
		item.second.Reset();
	}
}

std::vector<FilterInfo> VelocityEstimator::Process( const ros::Time& until )
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
		VelocitySourceManager& manager = _sourceRegistry.at( sourceName );
		DerivObservation obs = boost::apply_visitor( manager, msg );

		// Perform predict
		PredictInfo predInfo = PredictUntil( obs.timestamp );
		predInfo.time = _filterTime;
		predInfo.frameId = "predict";
		predInfo.stepNum = _stepCounter++;
		infos.emplace_back( predInfo );

		// Check observation likelihood
		// TODO HACK!
		const MatrixType& C = manager.GetObservationMatrix();
		VectorType v = obs.derivatives - C * _filter.GetState();
		MatrixType V = C * _filter.GetCovariance() * C.transpose() + obs.covariance;
		double ll = GaussianLogPdf( V, v );
		if( std::isnan( ll ) )
		{
			ROS_WARN_STREAM( "Log likelihood is nan!" );
		}
		else if( ll < _logLikelihoodThreshold )
		{
			ROS_WARN_STREAM( "Rejecting observation from " <<
			                 sourceName << " with log likelihood " <<
			                 ll << " less than threshold " <<
			                 _logLikelihoodThreshold );
		}
		else // Perform filter update
		{
			UpdateInfo upInfo = _filter.Update( obs.derivatives, C, obs.covariance );

			upInfo.time = _filterTime;
			upInfo.frameId = sourceName; //obs.referenceFrame;
			upInfo.stepNum = _stepCounter++;
			infos.emplace_back( upInfo );

			manager.Update( obs.timestamp, upInfo );
		}

		_updateBuffer.erase( oldest );

		// Check filter while processing
		CheckFilter();
	}

	// Predict the remainder of requested time
	PredictInfo predInfo = PredictUntil( until );
	infos.emplace_back( predInfo );

	// Have to check after final predict
	CheckFilter();

	return infos;
}

nav_msgs::Odometry VelocityEstimator::GetOdom() const
{
	nav_msgs::Odometry msg;
	msg.header.frame_id = _bodyFrame;
	msg.header.stamp = _filterTime;
	msg.child_frame_id = _bodyFrame;

	PoseSE3::TangentVector fullVel = PoseSE3::TangentVector::Zero();
	// TODO Hard-coded constants!
	MatrixType velCov = MatrixType::Zero( 6, 6 );
	if( _twoDimensional )
	{
		std::vector<unsigned int> inds;
		inds = std::vector<unsigned int>( {0, 1, 5} );
		PutSubmatrix( _filter.GetState().head<3>(),
		              fullVel, inds );
		PutSubmatrix( _filter.GetCovariance().topLeftCorner<3, 3>(),
		              velCov, inds, inds );
	}
	else
	{
		fullVel = _filter.GetState().head<6>();
		velCov = _filter.GetCovariance().topLeftCorner<6, 6>();
	}

	msg.twist.twist = TangentToMsg( fullVel );
	SerializeMatrix( velCov, msg.twist.covariance );
	return msg;
}

MatrixType VelocityEstimator::GetTransitionCov( double dt )
{
	return _transCovRate * dt;
}

PredictInfo VelocityEstimator::PredictUntil( const ros::Time& until )
{
	double dt = (until - _filterTime).toSec();
	if( dt < 0 )
	{
		std::stringstream ss;
		ss << "Predict to " << until << " requested from " << _filterTime;
		throw std::runtime_error( ss.str() );
	}
	_filterTime = until;
	MatrixType A = IntegralMatrix<double>( dt, StateDim(), _filterOrder );
	PredictInfo info = _filter.Predict( A, GetTransitionCov( dt ) );
	info.step_dt = dt;
	return info;
}

void VelocityEstimator::CheckFilter()
{
	double entropy = GaussianEntropy( _filter.GetCovariance() );
	if( entropy > _maxEntropyThreshold )
	{
		ROS_WARN_STREAM( "Filter entropy: " << entropy << " greater than max: " <<
		                 _maxEntropyThreshold << " Resetting filter..." );
		Reset( _filterTime );
	}
}
}
