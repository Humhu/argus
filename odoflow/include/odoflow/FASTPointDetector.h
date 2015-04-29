#ifndef _OFLOW_FASTFINDER_H_
#define _OFLOW_FASTFINDER_H_

#include "odoflow/InterestPointDetector.h"

namespace odoflow
{

	class FASTPointDetector
	: public InterestPointDetector
	{
	public:
		
		typedef std::shared_ptr<FASTPointDetector> Ptr;
		
		enum DetectorType { FAST_9_16,
							FAST_7_12,
							FAST_5_8 };
		
		FASTPointDetector();
		
		virtual InterestPoints FindInterestPoints( const cv::Mat& image );
		
		void SetMaxPoints( int m );
		void SetIntensityThreshold( int thresh );
		void SetNonMaxSuppression( bool enable );
		void SetDetectorType( DetectorType type );
	
	private:
	
		int intensityThreshold;
		bool useNonMaxSuppression;
		DetectorType detectorType;
		size_t maxPoints;
	
		int ConvertDetectorType( DetectorType type );
		
	};
}

#endif
