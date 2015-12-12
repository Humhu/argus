#include "camera_array/FiducialRewardFunction.h"
#include <boost/foreach.hpp>

using namespace argus_utils;

namespace camera_array
{

FiducialRewardFunction::FiducialRewardFunction( const FiducialDetectionModel::Ptr& model,
                                                const RobotArrayTransitionFunction::Ptr& trans )
: detectionModel( model ), transitionFunction( trans )
{}

double FiducialRewardFunction::CalculateReward( const RobotTargetState& state, 
                                                const CameraArrayAction& action )
{
	typedef std::vector<argus_msgs::FiducialDetection> Detections;
	
	RobotTargetState next = transitionFunction->Transition( state, action );
	
	double numDetections = 0;
	
	// Calculate observations for each active camera
	BOOST_FOREACH( const std::string cameraName, next.array.activeCameras )
	{
		BOOST_FOREACH( const RobotTargetState::TargetMap::value_type& item, next.targets )
		{
			const std::string& targetName = item.first;
			const PoseSE3& targetRelPose = item.second.pose;
			Detections detections = 
				detectionModel->GenerateDetections( cameraName,
				                                    targetName,
				                                    targetRelPose );
			numDetections += detections.size();
		}
		
	}
	return numDetections;
}

}
