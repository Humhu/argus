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
	_maxIters.AddCheck<IntegerValued>();

	_ransacIters.InitializeAndRead( ph, 10, "ransac_iters",
	                                "Maximum number of RANSAC iterations." );
	_ransacIters.AddCheck<GreaterThan>( 0 );
	_ransacIters.AddCheck<IntegerValued>();

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

	_minNumPoints.InitializeAndRead( ph, 0, "min_num_points",
	                                 "Min number of cloud points required" );
	_minNumPoints.AddCheck<GreaterThanOrEqual>( 0.0 );

	InitializeDerived( ph );
}

ScanMatchResult ScanMatcher::Match( const LaserCloudType::ConstPtr& key,
                                    const LaserCloudType::ConstPtr& tar,
                                    const PoseSE3& guessPose,
                                    LaserCloudType::Ptr& aligned )
{
	ScanMatchResult result;
	result.success = false;

	if( !key || !tar )
	{
		throw std::invalid_argument( "ScanMatcher: Received null point clouds." );
	}
	if( key->size() < _minNumPoints || tar->size() < _minNumPoints )
	{
		ROS_WARN_STREAM( "Input cloud sizes " << key->size() << " and " <<
		                 tar->size() << " less than required min " << _minNumPoints );
		return result;
	}

	// Set all the latest parameters
	// TODO Revamp this interface to use a reference instead of smart pointer
	MatcherType::Ptr matcher = CreateMatcher();
	matcher->ToRegistrar().setMaximumIterations( _maxIters );
	matcher->ToRegistrar().setRANSACIterations( _ransacIters );
	matcher->ToRegistrar().setRANSACOutlierRejectionThreshold( _ransacThreshold );
	matcher->ToRegistrar().setMaxCorrespondenceDistance( _maxCorrespondDist );
	matcher->ToRegistrar().setTransformationEpsilon( std::pow( 10, _logMinTransformEps ) );
	matcher->ToRegistrar().setEuclideanFitnessEpsilon( std::pow( 10, _logMinObjectiveEps ) );

	matcher->ToRegistrar().setInputTarget( key );
	matcher->ToRegistrar().setInputSource( tar );

	if( !aligned )
	{
		aligned = boost::make_shared<LaserCloudType>();
	}
	matcher->ToRegistrar().align( *aligned, guessPose.ToTransform().matrix().cast<float>() );

	if( !matcher->ToRegistrar().hasConverged() )
	{
		ROS_WARN_STREAM( "Scan match failed to converge." );
		return result;
	}

	result.inliers = matcher->GetInliers( _maxCorrespondDist );
	// result.numInliers = matcher->CountCorrespondences( _maxCorrespondDist );
	result.fitness = matcher->ToRegistrar().getFitnessScore( _maxCorrespondDist );

	FixedMatrixType<4, 4> H = matcher->ToRegistrar().getFinalTransformation().cast<double>();
	result.transform = PoseSE3( H );
	result.success = true;
	return result;
}
}
