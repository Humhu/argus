#ifndef _OFLOW_RIGIDESTIMATOR_H_
#define _OFLOW_RIGIDESTIMATOR_H_

#include "odoflow/MotionEstimator.h"

namespace odoflow
{
	
	class RigidEstimator
		: public MotionEstimator
	{
	public:
		
		typedef std::shared_ptr<RigidEstimator> Ptr;
		
		RigidEstimator();
		
		virtual bool EstimateMotion( const InterestPoints& firstPoints,
									 const InterestPoints& secondPoints,
									 argus_utils::PoseSE3& transform );
		
		void SetOutputScale( double scale );
		
	private:
		
		double outputScale;
	};
}

#endif
