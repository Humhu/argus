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

// TODO Parse from info -> Msg instead so we can have nicer error messages
void CovarianceManager::Initialize( const std::string& sourceName,
                                    ros::NodeHandle& ph )
{
	YAML::Node estimatorYaml;
	GetParamRequired( ph, "estimator", estimatorYaml );

	_sourceName = sourceName;
	_estimator = std::make_shared<CovarianceEstimator>( _sourceName, estimatorYaml );

	ros::NodeHandle subh( ph.resolveName( "input_streams" ) );
	_receiver.Initialize( subh );

	if( _receiver.GetDim() != _estimator->InputDim() )
	{
		std::stringstream ss;
		ss << "Estimator: " << sourceName << " has input dim "
		   << _estimator->InputDim() << " but features have total dim: "
		   << _receiver.GetDim();
		throw std::runtime_error( ss.str() );
	}

	// Load params if we have any to load
	bool initialized = false;
	if( HasParam( estimatorYaml, "load_path" ) )
	{
		std::string paramFilePath;
		GetParam( estimatorYaml, "load_path", paramFilePath );
		CovarianceEstimatorInfo estInfo;
		if( !ReadInfo<CovarianceEstimatorInfo>( paramFilePath,
		                                        estInfo ) )
		{
			ROS_WARN_STREAM( "Could not read params from: " << paramFilePath );
		}
		else
		{
			ROS_INFO_STREAM( "Loaded parameters for: " << _sourceName <<
			                 " from " << paramFilePath );
			_estimator->SetParamsMsg( estInfo );
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

	_queryServer = ph.advertiseService( "query_covariance", 
                                    &CovarianceManager::QueryCallback, 
                                    this );
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
	return _receiver.IsReady();
}

unsigned int CovarianceManager::OutputDim() const
{
	if( !_estimator ) { return 0; }
	return _estimator->OutputDim();
}

MatrixType CovarianceManager::EstimateCovariance( const ros::Time& time )
{
	StampedFeatures f;
	if( !_receiver.ReadStream( time, f ) )
	{
		throw std::runtime_error( "CovarianceManager: Could not read stream." );
	}
	return _estimator->Evaluate( f.features );
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