#include "odoflow/ROSParser.h"

#include "odoflow/CornerPointDetector.h"
#include "odoflow/FASTPointDetector.h"
#include "odoflow/FixedPointDetector.h"

#include "odoflow/LKPointTracker.h"

#include "odoflow/FivePointEstimator.h"
#include "odoflow/RigidEstimator.h"

namespace odoflow
{

	InterestPointDetector::Ptr ParseDetector( ros::NodeHandle& h ) 
	{
		
		std::string detectorType;
		h.param<std::string>( "detector/type", detectorType, "corner" );
		
		if( detectorType.compare( "corner" ) == 0)
		{
			
			int detectorMaxPoints;
			double detectorQuality;
			double detectorSeparation;
			int detectorBlockSize;
			bool detectorUseHarris;
			double detectorHarrisK;
			
			bool refineEnable;
			int refineWindowDim;
			int refineMaxIters;
			double refineMinEps;
			
			h.param( "detector/max_num_points", detectorMaxPoints, 500 );
			h.param( "detector/min_quality", detectorQuality, 0.01 );
			h.param( "detector/min_separation", detectorSeparation, 10.0 );
			h.param( "detector/block_size", detectorBlockSize, 3 );
			h.param( "detector/window_size", refineWindowDim, 10 );
			h.param( "detector/use_harris", detectorUseHarris, false );
			h.param( "detector/harris_k", detectorHarrisK, 0.04 );
			
			h.param( "detector/refine_enable", refineEnable, true );
			h.param( "detector/refine_max_iters", refineMaxIters, 20 );
			h.param( "detector/refine_min_eps", refineMinEps, 0.03 );
			
			CornerPointDetector::Ptr detector = 
			std::make_shared<CornerPointDetector>();
			
			detector->SetMaxPoints( detectorMaxPoints );
			detector->SetMinQuality( detectorQuality );
			detector->SetMinSeparation( detectorSeparation );
			detector->SetQualityWindowSize( detectorBlockSize );
			detector->SetHarrisMode( detectorUseHarris );
			detector->SetHarrisK( detectorHarrisK );
			
			detector->SetRefinementMode( refineEnable );
			detector->SetRefinementWindow( refineWindowDim, refineWindowDim );
			detector->SetRefinementCriteria( refineMaxIters, refineMinEps );
			
			return detector;
		}
		else if( detectorType.compare( "fixed" ) == 0)
		{
			
			std::vector<int> grid;
			std::vector<int> gridDefault = {6,6};
			h.param( "detector/grid", grid, gridDefault );
			
			InterestPoints gridPoints;
			

			
			FixedPointDetector::Ptr detector = 
			std::make_shared<FixedPointDetector>();
			detector->SetPoints( gridPoints );
			
			return detector;
		}
		else if( detectorType.compare( "FAST" ) == 0 )
		{
			
			int threshold, maxPoints;
			bool enableNonMaxSup;
			std::string detectorType;
			
			h.param( "detector/intensity_threshold", threshold, 60 );
			h.param( "detector/enable_non_maximum_suppression", enableNonMaxSup, false );
			h.param( "detector/max_num_points", maxPoints, 50 );
			h.param<std::string>( "detector/detector_type", detectorType, "FAST_7_12" );
			
			FASTPointDetector::Ptr detector =
				std::make_shared<FASTPointDetector>();
			detector->SetIntensityThreshold( threshold );
			detector->SetNonMaxSuppression( enableNonMaxSup );
			detector->SetMaxPoints( maxPoints );
			
			FASTPointDetector::DetectorType type;
			if( detectorType.compare( "FAST_9_16" ) == 0 )
			{
				type = FASTPointDetector::FAST_9_16;
			}
			else if( detectorType.compare( "FAST_7_12" ) == 0 )
			{
				type = FASTPointDetector::FAST_7_12;
			}
			else if( detectorType.compare( "FAST_5_8" ) == 0 )
			{
				type = FASTPointDetector::FAST_5_8;
			}
			else
			{
				throw std::runtime_error( "Invalid FAST detector type." );
			}
			detector->SetDetectorType( type );
			
			return detector;
		}
		else
		{
			throw std::runtime_error( "Invalid detector detector type specified." );
		}
		
	}
		
	InterestPointTracker::Ptr ParseTracker( ros::NodeHandle& h ) {
		
		std::string trackerType;
		h.param<std::string>( "tracker/type", trackerType, "lucas_kanade" );
		
		if( trackerType.compare( "lucas_kanade" ) == 0)
		{
			
			int maxIterations;
			std::vector<int> windowSize;
			std::vector<int> windowDefaults = {31, 31};
			double minEpsilon, minEigen;
			
			h.param( "tracker/max_iters", maxIterations, 20 );
			h.param<double>( "tracker/min_eps", minEpsilon, 0.03 );
			h.param( "tracker/window_size", windowSize, windowDefaults );
			h.param<double>( "tracker/eigenvalue_threshold", minEigen, 0.001 );
			
			LKPointTracker::Ptr tracker = std::make_shared<LKPointTracker>();
			tracker->SetFlowCriteria( maxIterations, minEpsilon );
			tracker->SetFlowWindow( windowSize[0], windowSize[1] );
			tracker->SetFlowThreshold( minEigen );
			
			return tracker;
		}
		else
		{
			throw std::runtime_error( "Invalid tracker type." );
		}
			
	}

	MotionEstimator::Ptr ParseEstimator( ros::NodeHandle& h ) 
	{
		std::string estimatorType;
		h.param<std::string>( "estimator/type", estimatorType, "five_point" );
		
		MotionEstimator::Ptr estimator;
		
		if( estimatorType.compare( "five_point" ) == 0)
		{
			
			std::string method;

			double confidence, outlierThresh, outputScale;
			
			h.param<std::string>( "estimator/method", method, "RANSAC" );

			h.param<double>( "estimator/desired_confidence", confidence, 0.99 );
			h.param<double>( "estimator/outlier_threshold", outlierThresh, 1.0 );
			h.param<double>( "estimator/output_scale", outputScale, 1.0 );
			
			FivePointEstimator::Ptr est = std::make_shared<FivePointEstimator>();
			if( method.compare( "RANSAC" ) == 0 )
			{
				est->SetMethod( FivePointEstimator::RANSAC );
			}
			else if( method.compare( "MEDS" ) == 0 )
			{
				est->SetMethod( FivePointEstimator::MEDS );
			}
			else
			{
				throw std::runtime_error( "Invalid motion estimator method." );
			}
			
			est->SetFittingParameters( confidence, outlierThresh );
			est->SetOutputScale( outputScale );
			
			estimator = est;
		}
		else if( estimatorType.compare( "rigid" ) == 0 )
		{
			
			RigidEstimator::Ptr est = std::make_shared<RigidEstimator>();
			
			double outputScale;
			
			h.param<double>( "estimator/output_scale", outputScale, 1.0 );
			
			est->SetOutputScale( outputScale );
			estimator = est;
			
		}
		else
		{
			throw std::runtime_error( "Invalid motion estimator type." );
		}
		
		return estimator;
		
	}
	
}
	
