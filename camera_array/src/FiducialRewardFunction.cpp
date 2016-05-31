#include "camera_array/FiducialRewardFunction.h"
#include "fiducials/FiducialCommon.h"
#include <boost/foreach.hpp>

using namespace argus;

namespace camera_array
{

FiducialRewardFunction::FiducialRewardFunction( const FiducialDetectionModel::Ptr& model,
                                                const RobotArrayTransitionFunction::Ptr& trans )
: detectionModel( model ), transitionFunction( trans )
{}

double FiducialRewardFunction::CalculateReward( const RobotTargetState& state, 
                                                const CameraArrayAction& action )
{
  typedef argus_msgs::FiducialDetection Detection;
	typedef std::vector<Detection> Detections;
	
	RobotTargetState next = transitionFunction->Transition( state, action );
	
	double reward = 0;
	
	// Calculate observations for each active camera
	BOOST_FOREACH( const std::string cameraName, next.array.activeCameras )
	{
	  double cameraSum = 0;
		BOOST_FOREACH( const RobotTargetState::TargetMap::value_type& item, next.targets )
		{
			const std::string& targetName = item.first;
			const PoseSE3& targetRelPose = item.second.pose;
			Detections detections = 
				detectionModel->GenerateDetections( cameraName,
				                                    targetName,
				                                    targetRelPose );
			BOOST_FOREACH( Detection& detection, detections )
			{
			  double r = fiducials::FindMinDistance( detection.points );
			  reward += r;
			  cameraSum += r;
			}
		}
		
	}
	return reward;
}

}
