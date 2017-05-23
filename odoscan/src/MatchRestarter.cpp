#include "odoscan/MatchRestarter.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"

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
	MatrixType cov( dim, dim );
	GetParam( ph, "sample_cov", cov, MatrixType::Identity( dim, dim ) );
	_guessDist = UniformEllipse<>( VectorType::Zero( dim ), cov );

	_numRestarts.InitializeAndRead( ph, 0, "num_restarts",
	                                "Number of random re-initializations" );
	_numRestarts.AddCheck<GreaterThanOrEqual>( 0 );
	_numRestarts.AddCheck<IntegerValued>();
}

ScanMatchResult MatchRestarter::Match( const LaserCloudType::ConstPtr& key,
                                       const LaserCloudType::ConstPtr& tar,
                                       const PoseSE3& guess,
                                       LaserCloudType::Ptr& aligned )
{
	ScanMatchResult best;
	best.success = false;
	best.numInliers = 0;
	LaserCloudType::Ptr bestAligned;

	for( unsigned int i = 0; i < _numRestarts + 1; ++i )
	{
		PoseSE3 trialGuess = (i == 0) ? guess : guess * SampleInit();
		//PoseSE3 trialGuess = guess * SampleInit();
		LaserCloudType::Ptr trialAligned;
		ScanMatchResult trial = _matcher->Match( key, tar, trialGuess,
		                                         trialAligned );
		if( !trial.success )
		{
			continue;
		}
		if( trial.numInliers > best.numInliers )
		{
			best = trial;
			bestAligned = trialAligned;
		}
		// TODO Compare error & numInliers together
	}
	aligned = bestAligned;
	return best;
}

PoseSE3 MatchRestarter::SampleInit()
{
	VectorType sample = _guessDist.Sample();
	if( _twoDimensional )
	{
		PoseSE2 init = PoseSE2::Exp( sample );
		return PoseSE3::FromSE2( init );
	}
	else
	{
		return PoseSE3::Exp( sample );
	}
}
}