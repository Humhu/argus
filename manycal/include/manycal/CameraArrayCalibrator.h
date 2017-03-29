#pragma once

#include <ros/ros.h>
#include <isam/sclam_monocular.h>

#include "graphopt/GraphOptimizer.h"

#include "lookup/LookupInterface.h"
#include "camplex/FiducialCommon.h"
#include "camplex/FiducialInfoManager.h"
#include "extrinsics_array/ExtrinsicsInterface.h"

#include "argus_utils/synchronization/MessageSynchronizer.hpp"
#include "argus_utils/synchronization/SynchronizationTypes.h"

#include "manycal/ManycalCommon.h"
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

	CameraArrayCalibrator( const ros::NodeHandle& nh );

	void ReadParams( const ros::NodeHandle& ph );

	void SetEnableSynchronization( bool e );
	void SetReferenceFrame( const std::string& f );
	void SetPriorCovariance( const PoseSE3::CovarianceMatrix& mat );

	void RegisterCamera( const std::string& name );
	void RegisterFiducial( const std::string& name );
	void BufferDetection( const ImageFiducialDetections& det );

	/*! \brief Process the queue and optimize. */
	void Spin();

	// TODO
	void WriteResults( const std::string& path );

	std::vector<FiducialObjectCalibration> GetFiducials() const;
	// TODO Return priors also?
	std::vector<CameraObjectCalibration> GetCameras() const;

private:

	mutable Mutex _mutex;

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
	std::vector<isam::FiducialFactor::Ptr> _observations;
	std::vector<ImageFiducialDetections> _detBuffer;

	typedef std::unordered_map<std::string, CameraRegistration> CameraRegistry;
	CameraRegistry _cameraRegistry;

	typedef std::unordered_map<std::string, FiducialRegistration> FiducialRegistry;
	FiducialRegistry _fiducialRegistry;

	bool ProcessDetection( const ImageFiducialDetections& dets,
	                       const WriteLock& lock );

	bool BootstrapInitializeCamera( const ImageFiducialDetections& dets,
	                                const WriteLock& lock );

	void FillCameraRegistration( const std::string& name,
	                             const argus::PoseSE3& pose,
	                             bool addPrior,
	                             const WriteLock& lock );
};

}
