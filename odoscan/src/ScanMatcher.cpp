#include "odoscan/ScanMatcher.h"

namespace argus
{

ScanMatcher::ScanMatcher() {}

ScanMatcher::~ScanMatcher() {}

void ScanMatcher::Initialize( ros::NodeHandle& ph )
{
	FullNumericRange maxIters;
	GetParamRequired( ph, "max_iters", maxIters );
	_maxIters.Initialize( ph, maxIters.init, "max_iters", 
	                      "Maximum number of matching iterations." );
	_maxIters.AddCheck<GreaterThanOrEqual>( maxIters.min );
	_maxIters.AddCheck<LessThanOrEqual>( maxIters.max );
	_maxIters.AddCheck<IntegerValued>( ROUND_CLOSEST );

	FullNumericRange ransacIters;
	GetParamRequired( ph, "ransac_iters", ransacIters );
	_ransacIters.Initialize( ph, ransacIters.init, "ransac_iters",
	                         "Maximum number of RANSAC iterations." );
	_ransacIters.AddCheck<GreaterThanOrEqual>( ransacIters.min );
	_ransacIters.AddCheck<LessThanOrEqual>( ransacIters.max );
	_ransacIters.AddCheck<IntegerValued>( ROUND_CLOSEST );

	FullNumericRange ransacThreshold;
	GetParamRequired( ph, "ransac_threshold", ransacThreshold );
	_ransacThreshold.Initialize( ph, ransacThreshold.init, "ransac_threshold", 
	                             "RANSAC outlier rejection distance." );
	_ransacThreshold.AddCheck<GreaterThanOrEqual>( ransacThreshold.min );
	_ransacThreshold.AddCheck<LessThanOrEqual>( ransacThreshold.max );

	FullNumericRange maxCorrespondDist;
	GetParamRequired( ph, "max_correspond_dist", maxCorrespondDist );
	_maxCorrespondDist.Initialize( ph, maxCorrespondDist.init, "max_correspond_dist",
	                               "Max correspondence distance" );
	_maxCorrespondDist.AddCheck<GreaterThanOrEqual>( maxCorrespondDist.min );
	_maxCorrespondDist.AddCheck<LessThanOrEqual>( maxCorrespondDist.max );

	FullNumericRange minTransformEps;
	GetParamRequired( ph, "min_transform_eps", minTransformEps );
	_minTransformEps.Initialize( ph, minTransformEps.init, "min_transform_eps",
	                             "Min transformation change" );
	_minTransformEps.AddCheck<GreaterThanOrEqual>( minTransformEps.min );
	_minTransformEps.AddCheck<LessThanOrEqual>( minTransformEps.max );

	FullNumericRange minObjectiveEps;
	GetParamRequired( ph, "min_objective_eps", minObjectiveEps );
	_minObjectiveEps.Initialize( ph, minObjectiveEps.init, "min_objective_eps",
	                             "Min objective change" );
	_minObjectiveEps.AddCheck<GreaterThanOrEqual>( minObjectiveEps.min );
	_minObjectiveEps.AddCheck<LessThanOrEqual>( minObjectiveEps.max );

	FullNumericRange maxError;
	GetParamRequired( ph, "max_error", maxError );
	_maxError.Initialize( ph, maxError.init, "max_error",
	                      "Maximum alignment mean sum of squared errors." );

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
	std::cout << "H: " << std::endl << H << std::endl;
	double yaw = atan2( H(0,1), H(0,0) );
	std::cout << "yaw: " << yaw << std::endl;
	pose = PoseSE3( H );

	return true;
}

}