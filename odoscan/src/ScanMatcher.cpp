#include "odoscan/ScanMatcher.h"

namespace argus
{

ScanMatcher::ScanMatcher() {}

ScanMatcher::~ScanMatcher() {}

void ScanMatcher::Initialize( ros::NodeHandle& ph )
{
	unsigned int maxIters;
	GetParamRequired( ph, "max_iters", maxIters );
	_maxIters.Initialize( ph, maxIters, "max_iters", 
	                      "Maximum number of matching iterations." );
	_maxIters.AddCheck<GreaterThan>( 0 );

	unsigned int ransacIters;
	GetParamRequired( ph, "ransac_iters", ransacIters );
	_ransacIters.Initialize( ph, ransacIters, "ransac_iters",
	                         "Maximum number of RANSAC iterations." );
	_ransacIters.AddCheck<GreaterThan>( 0 );

	double ransacThreshold;
	GetParamRequired( ph, "ransac_threshold", ransacThreshold );
	_ransacThreshold.Initialize( ph, ransacThreshold, "ransac_threshold", 
	                             "RANSAC outlier rejection distance." );
	_ransacThreshold.AddCheck<GreaterThan>( 0.0 );

	double maxCorrespondDist;
	GetParamRequired( ph, "max_correspond_dist", maxCorrespondDist );
	_maxCorrespondDist.Initialize( ph, maxCorrespondDist, "max_correspond_dist",
	                               "Max correspondence distance" );
	_maxCorrespondDist.AddCheck<GreaterThan>( 0.0 );

	double minTransformEps;
	GetParamRequired( ph, "min_transform_eps", minTransformEps );
	_minTransformEps.Initialize( ph, minTransformEps, "min_transform_eps",
	                             "Min transformation change" );
	_minTransformEps.AddCheck<GreaterThan>( 0.0 );

	double minObjectiveEps;
	GetParamRequired( ph, "min_objective_eps", minObjectiveEps );
	_minObjectiveEps.Initialize( ph, minObjectiveEps, "min_objective_eps",
	                             "Min objective change" );
	_minObjectiveEps.AddCheck<GreaterThan>( 0.0 );

	double maxError;
	GetParamRequired( ph, "max_error", maxError );
	_maxError.Initialize( ph, maxError, "max_error",
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
	matcher->setTransformationEpsilon( _minTransformEps );
	matcher->setEuclideanFitnessEpsilon( _minObjectiveEps );

	matcher->setInputTarget( key );
	matcher->setInputSource( tar );

	if( !aligned )
	{
		aligned = boost::make_shared<LaserCloudType>();
	}
	matcher->align( *aligned, pose.ToTransform().matrix().cast<float>() );

	if( !matcher->hasConverged() ) { return false; }

	double fitness = matcher->getFitnessScore( _maxCorrespondDist );
	if( fitness > _maxError ) { return false; }

	FixedMatrixType<4,4> H = matcher->getFinalTransformation().cast<double>();
	pose = PoseSE3( H );

	return true;
}

}