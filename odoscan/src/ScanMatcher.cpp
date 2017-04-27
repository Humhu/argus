#include "odoscan/ScanMatcher.h"

namespace argus
{

ScanMatcher::ScanMatcher() {}

ScanMatcher::~ScanMatcher() {}

void ScanMatcher::Initialize( ros::NodeHandle& ph )
{
	_maxIters.InitializeAndRead( ph, 100, "max_iters", 
	                      "Maximum number of matching iterations." );
	_maxIters.AddCheck<GreaterThan>( 0 );

	_ransacIters.InitializeAndRead( ph, 10, "ransac_iters",
	                         "Maximum number of RANSAC iterations." );
	_ransacIters.AddCheck<GreaterThan>( 0 );

	_ransacThreshold.InitializeAndRead( ph, 0.1, "ransac_threshold", 
	                             "RANSAC outlier rejection distance." );
	_ransacThreshold.AddCheck<GreaterThan>( 0.0 );

	_maxCorrespondDist.InitializeAndRead( ph, 0.1, "max_correspond_dist",
	                               "Max correspondence distance" );
	_maxCorrespondDist.AddCheck<GreaterThan>( 0.0 );

	_logMinTransformEps.InitializeAndRead( ph, -1, "log_min_transform_eps",
	                             "Min log transformation change" );

	_logMinObjectiveEps.InitializeAndRead( ph, -1, "log_min_objective_eps",
	                             "Min log objective change" );

	_maxError.InitializeAndRead( ph, 0.25, "max_error",
	                      "Maximum alignment mean sum of squared errors." );
	_maxError.AddCheck<GreaterThan>( 0.0 );

	InitializeDerived( ph );
}

bool ScanMatcher::Match( const LaserCloudType::ConstPtr& key, 
                         const LaserCloudType::ConstPtr& tar,
                         PoseSE3& pose,
                         LaserCloudType::Ptr& aligned )
{
	if( !key || !tar )
	{
		throw std::invalid_argument( "ScanMatcher: Received null point clouds." );
	}

	// Set all the latest parameters
	MatcherType::Ptr matcher = CreateMatcher();
	matcher->setMaximumIterations( _maxIters );
	matcher->setRANSACIterations( _ransacIters );
	matcher->setRANSACOutlierRejectionThreshold( _ransacThreshold );
	matcher->setMaxCorrespondenceDistance( _maxCorrespondDist );
	matcher->setTransformationEpsilon( std::pow( 10, _logMinTransformEps ) );
	matcher->setEuclideanFitnessEpsilon( std::pow( 10, _logMinObjectiveEps ) );

	matcher->setInputTarget( key );
	matcher->setInputSource( tar );

	if( !aligned )
	{
		aligned = boost::make_shared<LaserCloudType>();
	}
	matcher->align( *aligned, pose.ToTransform().matrix().cast<float>() );

	if( !matcher->hasConverged() ) 
	{
		ROS_WARN_STREAM( "Scan match failed to converge." );
		return false; 
	}

	double fitness = matcher->getFitnessScore( _maxCorrespondDist );
	if( fitness > _maxError ) 
	{
		ROS_WARN_STREAM( "Scan match result has error " << fitness << 
		                 " greater than threshold " << _maxError );
		return false; 
	}

	FixedMatrixType<4,4> H = matcher->getFinalTransformation().cast<double>();
	pose = PoseSE3( H );

	return true;
}

}
