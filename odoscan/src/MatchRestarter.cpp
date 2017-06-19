#include "odoscan/MatchRestarter.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"
#include <deque>

namespace argus
{
MatchRestarter::MatchRestarter()
{}

void MatchRestarter::Initialize( ros::NodeHandle& ph,
                                 ScanMatcher::Ptr matcher )
{
	_matcher = matcher;
	GetParam( ph, "two_dimensional", _twoDimensional, false );

	unsigned int dim = _twoDimensional ? PoseSE2::TangentDimension
					   : PoseSE3::TangentDimension;
	_guessDist = MultivariateGaussian<>( dim );
	GetParam( ph, "num_std_devs", _numStdDevs, 2.5 );

	_numRestarts.InitializeAndRead( ph, 0, "num_restarts",
	                                "Number of random re-initializations" );
	_numRestarts.AddCheck<GreaterThanOrEqual>( 0 );
	_numRestarts.AddCheck<IntegerValued>();

	unsigned int numThreads;
	GetParam( ph, "num_threads", numThreads, (unsigned int) 1 );
	_workers.SetNumWorkers( numThreads );
	_workers.StartWorkers();
}

void MatchRestarter::SetDisplacementDistribution( const PoseSE3& disp,
                                                  const PoseSE3::CovarianceMatrix& cov )
{
	if( _twoDimensional )
	{
		PoseSE2 disp2 = PoseSE2::FromSE3( disp );
		PoseSE2::CovarianceMatrix cov2;
		std::vector<int> inds = {0, 1, 5};
		GetSubmatrix( cov, cov2, inds, inds );
		_guessDist.SetMean( PoseSE2::Log( disp2 ) );
		_guessDist.SetCovariance( cov2 );
	}
	else
	{
		_guessDist.SetMean( PoseSE3::Log( disp ) );
		_guessDist.SetCovariance( cov );
	}
}

ScanMatchResult MatchRestarter::Match( const LaserCloudType::ConstPtr& key,
                                       const LaserCloudType::ConstPtr& tar,
                                       LaserCloudType::Ptr& aligned )
{
	unsigned int N = _numRestarts + 1;
	Semaphore sema;
	std::deque<MatchJobInfo> infos( N );
	for( unsigned int i = 0; i < N; ++i )
	{
		infos[i].key = key;
		infos[i].tar = tar;
		infos[i].guess = SampleInit( i == 0 );
		WorkerPool::Job job = boost::bind( &MatchRestarter::MatchJob,
		                                   this,
		                                   boost::ref( infos[i] ),
		                                   boost::ref( sema ) );
		_workers.EnqueueJob( job );
	}

	// Wait for jobs to finish
	sema.Decrement( N );

	// Find best result
	ScanMatchResult best;
	best.success = false;
	unsigned int bestNumInliers = 0;
	for( unsigned int i = 0; i < N; ++i )
	{
		if( !infos[i].result.success )
		{
			continue;
		}
		if( infos[i].result.inliers->size() > bestNumInliers )
		{
			best = infos[i].result;
			bestNumInliers = best.inliers->size();
			aligned = infos[i].aligned;
		}
	}

	// TODO Compare error & numInliers together
	return best;
}

void MatchRestarter::MatchJob( MatchJobInfo& info, Semaphore& sema )
{
	info.result = _matcher->Match( info.key,
	                               info.tar,
	                               info.guess,
	                               info.aligned );
	sema.Increment();
}

PoseSE3 MatchRestarter::SampleInit( bool meanOnly )
{
	if( _twoDimensional )
	{
		PoseSE2 init = meanOnly ?
		               PoseSE2::Exp( _guessDist.GetMean() ) :
		               PoseSE2::Exp( _guessDist.Sample() );
		return PoseSE3::FromSE2( init );
	}
	else
	{
		return meanOnly ?
		       PoseSE3::Exp( _guessDist.GetMean() ) :
		       PoseSE3::Exp( _guessDist.Sample() );
	}
}
}