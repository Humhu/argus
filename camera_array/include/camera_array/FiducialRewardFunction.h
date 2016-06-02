#pragma once

#include "camera_array/SystemModels.h"
#include "camera_array/FiducialDetectionModel.h"

namespace argus
{

/*! \brief Rewards observations of fiducials. Uses the fixed poses of the target
 * fiducials and rewards them equally. */
class FiducialRewardFunction
: public RobotArrayReward
{
public:
		
	FiducialRewardFunction( const FiducialDetectionModel::Ptr& model,
							const RobotArrayTransitionFunction::Ptr& trans );
	
	virtual double CalculateReward( const RobotTargetState& state, 
	                                const CameraArrayAction& action );

private:
	
	FiducialDetectionModel::Ptr detectionModel;
	RobotArrayTransitionFunction::Ptr transitionFunction;
	
};

}
