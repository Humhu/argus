#pragma once

#include "odoflow/MotionEstimator.h"
#include "paraset/ParameterManager.hpp"

namespace argus
{

/*! \brief Estimates a 2D rigid transformation. */
class RigidEstimator
	: public MotionEstimator
{
public:
	
	typedef std::shared_ptr<RigidEstimator> Ptr;
	
	RigidEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	virtual bool EstimateMotion( FrameInterestPoints& key,
	                             FrameInterestPoints& tar,
	                             PoseSE3& transform );
	
private:
	
	double _scale;
	NumericParam _logReprojThreshold;
	NumericParam _maxIters;
	
};

} // end namespace odoflow
