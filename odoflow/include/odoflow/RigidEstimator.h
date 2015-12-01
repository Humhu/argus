#pragma once

#include "odoflow/MotionEstimator.h"

namespace odoflow
{

/*! \brief Estimates a 2D rigid transformation. */
class RigidEstimator
	: public MotionEstimator
{
public:
	
	typedef std::shared_ptr<RigidEstimator> Ptr;
	
	RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	virtual bool EstimateMotion( const InterestPoints& srcPoints,
									const InterestPoints& dstPoints,
									argus_utils::PoseSE3& transform );
	
	void SetOutputScale( double scale );
	
private:
	
	double scale;
};

} // end namespace odoflow
