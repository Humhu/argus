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
	
	virtual bool EstimateMotion( const InterestPoints& firstPoints,
									const InterestPoints& secondPoints,
									argus_utils::PoseSE3& transform );
	
	void SetOutputScale( double scale );
	
private:
	
	double outputScale;
};

} // end namespace odoflow
