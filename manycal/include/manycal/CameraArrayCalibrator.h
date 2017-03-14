#pragma once

#include <ros/ros.h>
#include <isam/sclam_monocular.h>

#include "graphopt/GraphOptimizer.h"

#include "lookup/LookupInterface.h"
#include "fiducials/FiducialCommon.h"
#include "fiducials/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

#include "manycal/ManycalCommon.h"
#include "manycal/MessageSynchronizer.hpp"
#include "manycal/WriteCalibration.h"
#include "manycal/sclam_fiducial.h"
#include "argus_msgs/ImageFiducialDetections.h"

namespace argus
{

/*! \brief Calibrates an array of synchronized cameras. Assumes fiducials can move
 * around arbitrarily. */
class CameraArrayCalibrator
{
public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	
	CameraArrayCalibrator( ros::NodeHandle& nh, ros::NodeHandle& ph );
	
	bool WriteResults( manycal::WriteCalibration::Request& req, 
	                   manycal::WriteCalibration::Response& res );

	std::vector<FiducialCalibration> GetFiducials() const;
	// TODO Return priors also?
	std::vector<CameraCalibration> GetCameras() const;
	
private:
	
	ros::Subscriber _detSub;
	ros::ServiceServer _writeServer;

	typedef MessageSynchronizer<ImageFiducialDetections> DetectionSynchronizer;
	DetectionSynchronizer _sync;
	bool _useSynchronization;

	LookupInterface _lookupInterface;
	FiducialInfoManager _fiducialManager;
	ExtrinsicsInterface _extrinsicsManager;

	std::string _referenceFrame;
	PoseSE3::CovarianceMatrix _priorCovariance;
	
	struct CameraRegistration
	{
		isam::PoseSE3_Node::Ptr extrinsics;
		isam::MonocularIntrinsics_Node::Ptr intrinsics;
		isam::PoseSE3_Prior::Ptr extrinsicsPrior;
	};
	
	struct FiducialRegistration
	{
		isam::FiducialIntrinsics_Node::Ptr intrinsics;
		std::map<ros::Time, isam::PoseSE3_Node::Ptr> poses;
	};
	
	GraphOptimizer _optimizer;
	std::vector <isam::FiducialFactor::Ptr> _observations;

	std::vector <ImageFiducialDetections> _cachedObservations;
	
	typedef std::unordered_map <std::string, CameraRegistration> CameraRegistry;
	CameraRegistry _cameraRegistry;
	
	typedef std::unordered_map <std::string, FiducialRegistration> FiducialRegistry;
	FiducialRegistry _fiducialRegistry;

	void DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg );
	bool ProcessDetection( const ImageFiducialDetections& dets );
	bool InitializeCamera( const ImageFiducialDetections& dets );
	void ProcessCache();
	
	void RegisterCamera( const std::string& name, const argus::PoseSE3& pose,
						 bool addPrior = false );
	void RegisterFiducial( const std::string& name );
	
};
	
}
