#ifndef _OFLOW_EIGHTPOINT_ESTIMATOR_H_
#define _OFLOW_EIGHTPOINT_ESTIMATOR_H_

#include "odoflow/MotionEstimator.h"

namespace odoflow
{
	
	class EightPointEstimator
		: public MotionEstimator
	{
	public:
		
		typedef std::shared_ptr<EightPointEstimator> Ptr;
		
		enum Method
		{
			SEVEN_POINT,
			EIGHT_POINT,
			EIGHT_RANSAC,
			EIGHT_MEDS
		};
		
		EightPointEstimator();
		
		virtual bool EstimateMotion( const InterestPoints& firstPoints,
									 const InterestPoints& secondPoints,
									 Transform& transform );
		
		void SetMethod( Method method );
		
		// RANSAC epipolar threshold and confidence
		void SetFittingParameters( double prob, double thresh=1.0 );
		
	private:
		
		Method estimationMethod;
		double confidence;
		double outlierThreshold;
		
		int ConvertMethod( Method method );
		
	};
}

#endif
