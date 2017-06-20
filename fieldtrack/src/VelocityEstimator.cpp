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
	GetParamRequired( ph, "body_frame", _bodyFrame );
	GetParam( ph, "max_entropy_threshold",
	          _maxEntropyThreshold,
	          std::numeric_limits<double>::infinity() );
	GetParam( ph, "two_dimensional", _twoDimensional, false );

	GetParamRequired( ph, "filter_order", _filterOrder );

	_initialState = VectorType::Zero( FullDim() );
	GetParam( ph, "initial_mean", _initialState );

	_initialCovariance = MatrixType( FullDim(), FullDim() );
	GetParamRequired( ph, "initial_covariance", _initialCovariance );
	_filter.Initialize( _initialState, _initialCovariance );

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
		_sourceRegistry[sourceName].Initialize( sh,
		                                        _twoDimensional,
		                                        _filterOrder,
		                                        _bodyFrame,
		                                        extrinsics );
	}
}

CovarianceModel::Ptr VelocityEstimator::InitTransCovModel() const
{
	TimeScaledCovariance::Ptr cov = std::make_shared<TimeScaledCovariance>();
	cov->Initialize( _transCovRate );
	cov->EnableL( false ); // TODO
	return cov;
}

std::unordered_map<std::string, CovarianceModel::Ptr>
VelocityEstimator::InitObsCovModels() const
{
	std::unordered_map<std::string, CovarianceModel::Ptr> out;
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( const Item &item, _sourceRegistry )
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
		ROS_INFO_STREAM( "Transition covariance rate updated to: " << std::endl <<
		                 _transCovRate << std::endl <<
		                 "L: " << mod.GetLValue() << std::endl <<
		                 "D: " << mod.GetDValue().transpose() );
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
	try
	{
		const FixedCovariance& mod = dynamic_cast<const FixedCovariance&>( model );
		MatrixType R = mod.GetValue();
		ROS_INFO_STREAM( name << " covariance updated to: " << std::endl <<
		                 R << std::endl <<
		                 "L: " << mod.GetLValue() << std::endl <<
		                 "D: " << mod.GetDValue().transpose() );
	}
	catch( std::bad_cast& e )
	{}

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

void VelocityEstimator::GetFullVels( PoseSE3::TangentVector& vel,
                                     PoseSE3::CovarianceMatrix& cov ) const
{
	vel.setZero();
	cov.setZero();

	if( _twoDimensional )
	{
		std::vector<unsigned int> inds;
		inds = std::vector<unsigned int>( {0, 1, 5} );
		PutSubmatrix( _filter.GetState().head<3>(),
		              vel, inds );
		PutSubmatrix( _filter.GetCovariance().topLeftCorner<3, 3>(),
		              cov, inds, inds );
	}
	else
	{
		vel = _filter.GetState().head<6>();
		cov = _filter.GetCovariance().topLeftCorner<6, 6>();
	}
}

nav_msgs::Odometry VelocityEstimator::GetOdom() const
{
	PoseSE3::TangentVector vel;
	PoseSE3::CovarianceMatrix cov;
	GetFullVels( vel, cov );

	nav_msgs::Odometry msg;
	msg.header.frame_id = _bodyFrame;
	msg.header.stamp = GetFilterTime();
	msg.child_frame_id = _bodyFrame;
	msg.twist.twist = TangentToMsg( vel );
	SerializeMatrix( cov, msg.twist.covariance );
	return msg;
}

geometry_msgs::TwistStamped VelocityEstimator::GetTwist() const
{
	PoseSE3::TangentVector vel;
	PoseSE3::CovarianceMatrix cov;
	GetFullVels( vel, cov );

	geometry_msgs::TwistStamped msg;
	msg.header.frame_id = _bodyFrame;
	msg.header.stamp = GetFilterTime();
	msg.twist = TangentToMsg( vel );
	return msg;
}

geometry_msgs::TwistWithCovarianceStamped
VelocityEstimator::GetTwistWithCovariance() const
{
	PoseSE3::TangentVector vel;
	PoseSE3::CovarianceMatrix cov;
	GetFullVels( vel, cov );

	geometry_msgs::TwistWithCovarianceStamped msg;
	msg.header.frame_id = _bodyFrame;
	msg.header.stamp = GetFilterTime();
	msg.twist.twist = TangentToMsg( vel );
	SerializeMatrix( cov, msg.twist.covariance );
	return msg;
}

MatrixType VelocityEstimator::GetTransitionCov( double dt )
{
	return _transCovRate * dt;
}

void VelocityEstimator::ResetDerived( const ros::Time& time,
                                      const VectorType& state,
                                      const MatrixType& cov )
{
	VectorType initState = _initialState;
	if( state.size() == StateDim() )
	{
		initState.head( StateDim() ) = state;
	}
	else if( state.size() == FullDim() )
	{
		initState = state;
	}

	MatrixType initCov = _initialCovariance;
	if( cov.rows() == StateDim() && cov.cols() == StateDim() )
	{
		initCov.topLeftCorner( StateDim(), StateDim() ) = cov;
	}
	else if( cov.rows() == FullDim() && cov.cols() == FullDim() )
	{
		initCov = cov;
	}

	// Reset the filter state
	ROS_INFO_STREAM( "Resetting filter mean to: " << initState.transpose() <<
	                 " and cov: " << std::endl << initCov );
	_filter.Initialize( initState, initCov );

	// Reset all observation covariance adapters
	typedef SourceRegistry::value_type Item;
	BOOST_FOREACH( Item & item, _sourceRegistry )
	{
		item.second.Reset();
	}
}

PredictInfo VelocityEstimator::PredictUntil( const ros::Time& until )
{
	double dt = (until - GetFilterTime() ).toSec();
	MatrixType A = IntegralMatrix<double>( dt, StateDim(), _filterOrder );
	PredictInfo info = _filter.Predict( A, GetTransitionCov( dt ) );
	info.step_dt = dt;
	return info;
}

bool VelocityEstimator::ProcessMessage( const std::string& source,
                                        const ObservationMessage& msg,
                                        UpdateInfo& info )
{
	VelocitySourceManager& manager = _sourceRegistry.at( source );
	DerivObservation obs = boost::apply_visitor( manager, msg );

	// Check observation likelihood
	// TODO HACK!
	const MatrixType& C = obs.C; //manager.GetObservationMatrix();
        //ROS_INFO_STREAM( "C: " << std::endl << obs.C );
        //ROS_INFO_STREAM( "Obs: " << obs.derivatives.transpose() << std::endl << "pred: " << C * _filter.GetState() );
	VectorType v = obs.derivatives - C * _filter.GetState();
	MatrixType V = C * _filter.GetCovariance() * C.transpose() + obs.covariance;
	double ll = GaussianLogPdf( V, v );
	if( !manager.CheckLogLikelihood( ll ) )
	{
		ROS_WARN_STREAM( "Rejecting observation from " <<
		                 source << " with log likelihood " << ll );
		return false;
	}

	info = _filter.Update( obs.derivatives, C, obs.covariance );
	info.time = GetFilterTime();
	info.frameId = source;

	manager.Update( info );
	return true;
}

void VelocityEstimator::CheckFilter()
{
	double entropy = GaussianEntropy( _filter.GetCovariance() );
	if( entropy > _maxEntropyThreshold )
	{
		ROS_WARN_STREAM( "Filter entropy: " << entropy << " greater than max: " <<
		                 _maxEntropyThreshold << " Resetting filter..." );
		ResetDerived( GetFilterTime() );
	}
}
}
