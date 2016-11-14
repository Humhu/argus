#pragma once

#include <ros/ros.h>
#include <isam/sclam_monocular.h>

#include "lookup/LookupInterface.h"
#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInfoManager.h"
#include "graphopt/GraphOptimizer.h"
#include "manycal/ManycalCommon.h"
#include "manycal/sclam_fiducial.h"
#include "argus_msgs/ImageFiducialDetections.h"

namespace argus
{

// TODO Add service calls to write results

/*! \brief Estimates the extrinsics of an array of fiducials with a single image stream.
 *
 *  Fiducials are initialized as they are observed by the camera, with the first
 *  fiducial placed at the origin. Consecutive observations are initialized by using
 *  the last estimated pose of the camera. 
 *
 *  ROS Parameters:
 *  =======================
 *  ~reference_frame: string
 *      Name of the fiducial array reference frame. Will be used when outputting the
 *      calibration file.
 *  ~batch_period: unsigned int
 *      The number of observation factors in between batch optimizations.
 *  ~lookup_namespace: string (default: "/lookup")
 *      The lookup parameter base namespace.
 *
 *  ROS Topics:
 *  ===========
 *  detections: argus_msgs::ImageFiducialDetections
 *      Stream of detections coming from a single camera. Should be rectified and normalized.
 */
class FiducialArrayCalibrator
{
public:
	
	FiducialArrayCalibrator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	~FiducialArrayCalibrator();
	
	void WriteResults( const ros::TimerEvent& event );
	
private:
	
	ros::Subscriber _detSub;

	LookupInterface _lookupInterface;
	FiducialInfoManager _fiducialManager;
	ExtrinsicsInfoManager _extrinsicsManager;

	std::string _referenceFrame;
	isam::MonocularIntrinsics_Node::Ptr _cameraIntrinsics;

	unsigned int _detCounter;
	unsigned int _batchPeriod;
	
	struct FiducialRegistration
	{
		isam::PoseSE3_Node::Ptr extrinsics;
		isam::FiducialIntrinsics_Node::Ptr intrinsics;
		isam::PoseSE3_Prior::Ptr extrinsicsPrior;
	};
	
	GraphOptimizer _optimizer;
	isam::PoseSE3_Node::Ptr _fiducialReference;
	std::vector <isam::PoseSE3_Node::Ptr> _cameraPoses;
	std::vector <isam::FiducialFactor::Ptr> _observations;
	
	typedef std::unordered_map <std::string, FiducialRegistration> FiducialRegistry;
	FiducialRegistry _fiducialRegistry;
	
	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	
	/*! \brief Checks if the specified fiducial has a pose guess. */
	bool HasExtrinsicsPrior( const std::string& name );
	bool HasIntrinsicsPrior( const std::string& name );
	
	/*! \brief Creates the fiducial and reads its intrinsics via lookup. */
	bool RegisterFiducial( const std::string& name, const argus::PoseSE3& pose,
	                       bool addPrior = false );
	
};
	
}
