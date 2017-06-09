#include "fieldtrack/PoseEstimator.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/random/MultivariateGaussian.hpp"

namespace argus
{
PoseEstimator::PoseEstimator() {}

// TODO Support 2D mode
void PoseEstimator::Initialize( ros::NodeHandle& ph,
                                ExtrinsicsInterface::Ptr extrinsics )
{
	GetParamRequired( ph, "reference_frame", _referenceFrame );
	GetParamRequired( ph, "body_frame", _bodyFrame );
	GetParam( ph, "log_likelihood_threshold",
	          _logLikelihoodThreshold,
	          -std::numeric_limits<double>::infinity() );
	GetParam( ph, "max_entropy_threshold",
	          _maxEntropyThreshold,
	          std::numeric_limits<double>::infinity() );
	GetParam( ph, "two_dimensional", _twoDimensional, false );

	GetParam( ph, "initial_pose", _initialPose, PoseSE3() );
	GetParamRequired( ph, "initial_covariance", _initialCovariance );
	_filter.Initialize( _initialPose, _initialCovariance );

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
		_sourceRegistry[sourceName].Initialize( sh,
		                                        _twoDimensional,
		                                        _referenceFrame,
		                                        _bodyFrame,
		                                        extrinsics );
	}
}

void PoseEstimator::BufferVelocity( const ros::Time& time,
                                    const VectorType& vel,
                                    const MatrixType& cov )
{
	if( time < GetFilterTime() )
	{
		ROS_WARN_STREAM( "Velocity info from time " << time <<
		                 " before filter time " << GetFilterTime() );
		return;
	}
	_velocityBuffer.emplace( std::piecewise_construct,
	                         std::forward_as_tuple( time ),
	                         std::forward_as_tuple( vel, cov ) );
}

nav_msgs::Odometry PoseEstimator::GetOdom() const
{
	nav_msgs::Odometry msg;
	msg.header.frame_id = _referenceFrame;
	msg.child_frame_id = _bodyFrame;
	msg.header.stamp = GetFilterTime();
	msg.pose.pose = PoseToMsg( _filter.GetState() );
	SerializeMatrix( _filter.GetCovariance(), msg.pose.covariance );

	msg.twist.twist = TangentToMsg( _lastVel );
	SerializeMatrix( _lastVelCov, msg.twist.covariance );

	return msg;
}

geometry_msgs::PoseStamped PoseEstimator::GetPose() const
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = _bodyFrame;
	msg.header.stamp = GetFilterTime();
	msg.pose = PoseToMsg( _filter.GetState() );
	return msg;
}

geometry_msgs::PoseWithCovarianceStamped PoseEstimator::GetPoseWithCovariance() const
{
	geometry_msgs::PoseWithCovarianceStamped msg;
	msg.header.frame_id = _bodyFrame;
	msg.header.stamp = GetFilterTime();
	msg.pose.pose = PoseToMsg( _filter.GetState() );
	SerializeMatrix( _filter.GetCovariance(), msg.pose.covariance );
	return msg;
}

CovarianceModel::Ptr PoseEstimator::InitTransCovModel() const
{
	TimeScaledCovariance::Ptr cov = std::make_shared<TimeScaledCovariance>();
	cov->Initialize( _transCovRate );
	return cov;
}

std::unordered_map<std::string, CovarianceModel::Ptr>
PoseEstimator::InitObsCovModels() const
{
	std::unordered_map<std::string, CovarianceModel::Ptr> out;
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( const Item &item, _sourceRegistry )
	{
		const std::string& name = item.first;
		const PoseSourceManager& manager = item.second;
		out[name] = manager.InitializeModel();
	}
	return out;
}

void PoseEstimator::SetTransCovModel( const CovarianceModel& model )
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

void PoseEstimator::SetObsCovModel( const std::string& name,
                                    const CovarianceModel& model )
{
	if( _sourceRegistry.count( name ) == 0 )
	{
		throw std::invalid_argument( "Source " + name + " not registered!" );
	}
	_sourceRegistry[name].SetModel( model );
}

void PoseEstimator::ResetDerived( const ros::Time& time,
                                  const VectorType& state,
                                  const MatrixType& cov )
{
	PoseSE3 initPose = ( state.size() == 0 ) ? _initialPose : PoseSE3( state );
	PoseSE3::CovarianceMatrix initCov = ( cov.size() == 0 ) ? _initialCovariance :
	                                    PoseSE3::CovarianceMatrix( cov );

	_filter.Initialize( initPose, initCov );
	_velocityBuffer.clear();

	// Reset all observation covariance adapters
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item & item, _sourceRegistry )
	{
		item.second.Reset();
	}

	_lastVel.setZero();
	_lastVelCov.setZero();
}

PredictInfo PoseEstimator::PredictUntil( const ros::Time& until )
{
	PoseSE3 displacement;
	PoseSE3::CovarianceMatrix covariance;
	IntegrateVelocities( GetFilterTime(), until, displacement, covariance );
	PredictInfo info = _filter.Predict( displacement, covariance );
	info.step_dt = (until - GetFilterTime() ).toSec();
	return info;
}

bool PoseEstimator::ProcessMessage( const std::string& source,
                                    const ObservationMessage& msg,
                                    UpdateInfo& info )
{
	PoseSourceManager& manager = _sourceRegistry.at( source );
	PoseObservation obs = boost::apply_visitor( manager, msg );

	// Check observation likelihood
	// TODO HACK!
	VectorType v = PoseSE3::Log( obs.pose.Inverse() * _filter.GetState() );
	MatrixType V = _filter.GetCovariance() + obs.covariance;
	double ll = GaussianLogPdf( V, v );
	if( std::isnan( ll ) )
	{
		ROS_WARN_STREAM( "Log likelihood is nan!" );
		return false;
	}
	else if( ll < _logLikelihoodThreshold )
	{
		ROS_WARN_STREAM( "Rejecting observation from " <<
		                 source << " with log likelihood " <<
		                 ll << " less than threshold " <<
		                 _logLikelihoodThreshold );
		return false;
	}
	else // Perform filter update
	{
		info = _filter.Update( obs.pose, obs.covariance );
		info.time = GetFilterTime();
		info.frameId = source;

		manager.Update( info );
	}
	return true;
}

void PoseEstimator::CheckFilter()
{
	double entropy = GaussianEntropy( _filter.GetCovariance() );
	if( entropy > _maxEntropyThreshold )
	{
		ROS_WARN_STREAM( "Filter entropy: " << entropy << " greater than max: " <<
		                 _maxEntropyThreshold << " Resetting filter..." );
		Reset( GetFilterTime() );
	}
}

void PoseEstimator::IntegrateVelocities( const ros::Time& from,
                                         const ros::Time& to,
                                         PoseSE3& disp,
                                         PoseSE3::CovarianceMatrix& cov )
{
	disp = PoseSE3();
	cov.setZero();

	bool initialized = false;
	// If we're not operating in velocity mode, these initial values
	// will default behavior to fixed covariance integration
	ros::Time lastTime = from;
	VectorType lastVel = PoseSE3::TangentVector::Zero();
	PoseSE3::CovarianceMatrix lastCov = _transCovRate;
	while( !_velocityBuffer.empty() )
	{
		VelocityBuffer::const_iterator oldest = _velocityBuffer.begin();
		const ros::Time& velTime = oldest->first;
		const VelocityInfo& info = oldest->second;
		const VectorType& vel = info.first;
		const MatrixType& velCov = info.second;

		if( velTime < from )
		{
			_velocityBuffer.erase( oldest );
			continue;
		}

		if( velTime > to )
		{
			break;
		}

		if( initialized )
		{
			double dt = (velTime - lastTime).toSec();
			disp = disp * PoseSE3::Exp( dt * lastVel );
			cov += dt * lastCov;
		}

		initialized = true;
		lastTime = velTime;
		lastVel = vel;
		lastCov = velCov;
		_velocityBuffer.erase( oldest );
	}

	double dt = (to - lastTime).toSec();
	disp = disp * PoseSE3::Exp( dt * lastVel );
	cov += dt * lastCov;

	_lastVel = lastVel;
	_lastVelCov = lastCov;
}
}