#ifndef _OFLOW_FIVEPOINT_ESTIMATOR_H_
#define _OFLOW_FIVEPOINT_ESTIMATOR_H_

#include "odoflow/MotionEstimator.h"

#include <opencv2/calib3d/calib3d.hpp>

namespace odoflow
{
	
	/*! \brief Estimates camera motion between frames using the 5-point algorithm
	 * by Nister. */
	class FivePointEstimator
		: public MotionEstimator
	{
	public:
		
		typedef std::shared_ptr<FivePointEstimator> Ptr;
		
		enum Method 
		{
			RANSAC,
			MEDS
		};
		
		FivePointEstimator();
		
		virtual bool EstimateMotion( const InterestPoints& firstPoints,
									 const InterestPoints& secondPoints,
									 argus_utils::PoseSE3& transform );
		
		/*! \brief Set the method used to estimate the essential matrix. */
		void SetMethod( Method method );
		
		/*! \brief Set the RANSAC max epipolar distance in pixels and desired confidence. */
		void SetFittingParameters( double prob, double thresh=1.0 );
		
		/*! \brief Set the pixel to output scale factor. */
		void SetOutputScale( double scale );
		
	private:
		
		
		Method estimationMethod;
		double confidence;
		double outlierThreshold;
		
		double residualThreshold;

		// Converts pixel distances to output scale
		double outputScale;		

		int ConvertMethod( Method method );
		
	};
	
}

#endif
