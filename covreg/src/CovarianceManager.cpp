#include "covreg/EstimatorInfoParsers.h"
#include "covreg/CovarianceManager.h"

#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>
#include <sstream>

using namespace covreg;

namespace argus
{

CovarianceManager::CovarianceManager()
: _estimator( nullptr ) {}

void CovarianceManager::Initialize( const std::string& sourceName,
                                    ros::NodeHandle& ph, 
                                    const std::string& subName )
{
	XmlRpc::XmlRpcValue xml;
	if( !ph.getParam( subName, xml ) )
	{
		std::stringstream ss;
		ss << "Could not initialize from parameters at: ~/" << subName << std::endl;
		throw std::runtime_error( ss.str() );
	}
	YAML::Node yaml = XmlToYaml( xml );
	double cacheTime;
	GetParam( ph, subName + "/feature_cache_time", cacheTime, 1.0 );
	Initialize( sourceName, yaml, cacheTime );

	_queryServer = ph.advertiseService( "query_covariance", 
	                                    &CovarianceManager::QueryCallback, 
	                                    this );
}

// TODO Parse from info -> Msg instead so we can have nicer error messages
// TODO Parse the receiver modes
void CovarianceManager::Initialize( const std::string& sourceName,
                                    const YAML::Node& info,
                                    double cacheTime )
{
	// Create the estimator
	_sourceName = sourceName;
	_estimator = std::make_shared<CovarianceEstimator>( _sourceName, info );

	std::vector<std::string> features = info["features"].as<std::vector<std::string>>();
	_receivers.clear();
	unsigned int fDim = 0;
	BOOST_FOREACH( const std::string& featureName, features )
	{
		_receivers.emplace_back();
		_receivers.back().Initialize( featureName, CLOSEST_BEFORE, cacheTime );
		fDim += _receivers.back().GetDim();
	}

	if( fDim != _estimator->InputDim() )
	{
		std::stringstream ss;
		ss << "Estimator: " << sourceName << " has input dim "
		   << _estimator->InputDim() << " but features have total dim: "
		   << fDim;
		throw std::runtime_error( ss.str() );
	}

	// Load params if we have any to load
	bool initialized = false;
	if( info[ "load_path" ] )
	{
		std::string paramFilePath = info["load_path"].as<std::string>();
		CovarianceEstimatorInfo info;
		if( !ReadInfo<CovarianceEstimatorInfo>( paramFilePath,
		                                        info ) )
		{
			ROS_WARN_STREAM( "Could not read params from: " << paramFilePath );
		}
		else
		{
			ROS_INFO_STREAM( "Loaded parameters for: " << _sourceName <<
			                 " from " << paramFilePath );
			_estimator->SetParamsMsg( info );
			initialized = true;
		}
	}
	if( !initialized )
	{
		ROS_WARN_STREAM( "No parameters specified for: " << _sourceName <<
		                 ". Using defaults." );
		_estimator->RandomizeVarianceParams();
		_estimator->ZeroCorrelationParams();
	}
}

void CovarianceManager::SetUpdateTopic( const std::string& topic )
{
	_paramSub = _nodeHandle.subscribe( topic, 
	                                  10, 
	                                  &CovarianceManager::ParamCallback, 
	                                  this );
}

bool CovarianceManager::IsReady() const
{
	if( !_estimator ) { return false; }
	BOOST_FOREACH( const BroadcastReceiver& rx, _receivers )
	{
		if( !rx.IsReady() ) 
		{
			ROS_WARN_STREAM( "Stream: " << rx.GetStreamName() << 
				             " has not received yet." );
			return false; 
		}
	}
	return true;
}

unsigned int CovarianceManager::OutputDim() const
{
	if( !_estimator ) { return 0; }
	return _estimator->OutputDim();
}

MatrixType CovarianceManager::EstimateCovariance( const ros::Time& time )
{
	VectorType feats( _estimator->InputDim() );
	unsigned int fInd = 0;
	BOOST_FOREACH( BroadcastReceiver& rx, _receivers )
	{
		StampedFeatures f;
		rx.ReadStream( time, f ); // TODO Handle failure
		feats.segment( fInd, rx.GetDim() ) = f.features;
		fInd += rx.GetDim();
	}
	return _estimator->Evaluate( feats );
}

void CovarianceManager::ParamCallback( const CovarianceEstimatorInfo::ConstPtr& msg )
{
	WriteLock lock( _estimatorMutex );
	if( msg->source_name != _sourceName )
	{
		ROS_WARN_STREAM( "Received params for: " << msg->source_name <<
		                 " but expected " << _sourceName );
	}
	ROS_INFO_STREAM( "Updating parameters for: " << _sourceName );
	_estimator->SetParamsMsg( *msg );
}

bool CovarianceManager::QueryCallback( QueryCovariance::Request& request,
	                                   QueryCovariance::Response& response )
{
	WriteLock lock( _estimatorMutex );
	VectorType feats = GetVectorView( request.features );
	if( feats.size() != _estimator->InputDim() ) 
	{ 
		ROS_WARN_STREAM( "Covariance query has feature dim: " << + feats.size() <<
		                 " but estimator input dim: " << _estimator->InputDim() );
		return false;
	}
	MatrixType cov = _estimator->Evaluate( feats );
	response.covariance = MatrixToMsg( cov );
	return true;
}

}