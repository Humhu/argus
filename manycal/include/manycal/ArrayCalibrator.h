#pragma once

#include <ros/ros.h>
#include <yaml-cpp/yaml.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <isam/sclam_monocular.h>

#include "manycal/WriteCalibration.h"
#include "manycal/Registrations.h"
#include "graphopt/GraphOptimizer.h"

#include "lookup/LookupInterface.h"
#include "argus_msgs/ImageFiducialDetections.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

#include "camplex/FiducialInfoManager.h"

#include "argus_utils/geometry/PoseSE3.h"
#include "argus_utils/geometry/VelocityIntegrator.hpp"
#include "argus_utils/synchronization/SynchronizationTypes.h"

#include <map>
#include <unordered_map>

namespace argus
{
/*! \brief Calibrates arrays of cameras and fiducials. */
class ArrayCalibrator
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ArrayCalibrator( ros::NodeHandle& nh, ros::NodeHandle& ph );

	void ProcessUntil( const ros::Time& until );

private:

	ros::NodeHandle _nodeHandle;
	ros::NodeHandle _privHandle;
	ros::ServiceServer _writeServer;

	ros::Timer _spinTimer;

	Mutex _mutex;
	std::vector<ros::Subscriber> _detSubs;
	typedef std::map<double, ImageFiducialDetections> DetectionsBuffer;
	DetectionsBuffer _detBuffer;
	ros::Time _buffTime;

	/*! \brief Stores subscriptions to odometry and detection inputs. */
	typedef std::unordered_map<std::string, CameraRegistration::Ptr> CameraRegistry;
	typedef std::unordered_map<std::string, FiducialRegistration::Ptr> FiducialRegistry;
	std::vector<TargetRegistration> _targetRegistry;
	CameraRegistry _cameraRegistry;
	FiducialRegistry _fiducialRegistry;

	GraphOptimizer _graph;
	std::vector<isam::FiducialFactor::Ptr> _observations;

	PoseSE3::CovarianceMatrix _extInitCov;

	void TimerCallback( const ros::TimerEvent& event );

	void ProcessDetection( const std::string& sourceName,
	                       const ros::Time& time,
						   const FiducialDetection& det );
						   
	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
};
} // end namespace manycal
