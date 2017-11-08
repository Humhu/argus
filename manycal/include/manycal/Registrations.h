#pragma once

#include <ros/ros.h>

#include "manycal/ManycalCommon.h"

#include "graphopt/GraphOptimizer.h"

#include "graphopt/PoseGraph.hpp"
#include "graphopt/StaticPoseGraph.hpp"
#include "graphopt/OdometryGraph.hpp"
#include "graphopt/JumpPoseGraph.hpp"

#include "argus_msgs/ImageFiducialDetections.h"
#include "nav_msgs/Odometry.h"

#include "argus_utils/geometry/VelocityIntegrator.hpp"

#include <unordered_map>
#include <memory>

namespace argus
{

// Base class for all optimization target registration classes
class RegistrationBase
{
public:

	const std::string _name;
	GraphOptimizer& _graph;

	RegistrationBase( const std::string& n, GraphOptimizer& g );
};

class TargetRegistration;

// Base class for registrations with extrinsics properties
class ExtrinsicsRegistration : public RegistrationBase
{
public:

	TargetRegistration& parent;

	ExtrinsicsRegistration( TargetRegistration& parent, const std::string& n,
	                        GraphOptimizer& g, ros::NodeHandle& ph );

	void InitializeExtrinsics( const PoseSE3& pose,
	                           const PoseSE3::CovarianceMatrix& cov = 1E-3 * PoseSE3::CovarianceMatrix::Identity() );

	bool IsExtrinsicsInitialized() const;
	// Returns whether optimization is enabled
	bool IsExtrinsicsOptimizing() const;
	PoseSE3 GetExtrinsicsPose() const;
	isam::PoseSE3_Node* GetExtrinsicsNode();
	isam::PoseSE3_Prior* GetExtrinsicsPrior();

private:

	bool _extInitialized;
	bool _optimizeExtrinsics;
	std::shared_ptr<isam::PoseSE3_Node> _extrinsics;
	std::shared_ptr<isam::PoseSE3_Prior> _extrinsicsPrior;
};

class CameraRegistration : public ExtrinsicsRegistration
{
public:

	typedef std::shared_ptr<CameraRegistration> Ptr;

	CameraRegistration( TargetRegistration& parent, const std::string& name,
	                    GraphOptimizer& graph, ros::NodeHandle& ph );

	void InitializeIntrinsics( const CameraCalibration& intrinsics,
	                           const FixedMatrixType<4, 4>& cov = 1E-3 * FixedMatrixType<4, 4>::Identity() );

	bool IsIntrinsicsInitialized() const;
	bool IsIntrinsicsOptimizing() const;
	isam::MonocularIntrinsics_Node* GetIntrinsicsNode();
	isam::MonocularIntrinsics_Prior* GetIntrinsicsPrior();

private:

	bool _intInitialized;
	std::shared_ptr<isam::MonocularIntrinsics_Node> _intrinsics;
	std::shared_ptr<isam::MonocularIntrinsics_Prior> _intrinsicsPrior;

	bool _optimizeIntrinsics;
};

class FiducialRegistration : public ExtrinsicsRegistration
{
public:

	typedef std::shared_ptr<FiducialRegistration> Ptr;

	FiducialRegistration( TargetRegistration& parent, const std::string& name,
	                      GraphOptimizer& graph, ros::NodeHandle& ph );

	// TODO Think about how to do this right...
	void InitializeIntrinsics( const Fiducial& intrinsics, const MatrixType& cov = MatrixType() );

	bool IsIntrinsicsInitialized() const;
	bool IsIntrinsicsOptimizing() const;
	isam::FiducialIntrinsics_Node* GetIntrinsicsNode();
	isam::FiducialIntrinsics_Prior* GetIntrinsicsPrior();

private:

	bool _intInitialized;
	std::shared_ptr<isam::FiducialIntrinsics_Node> _intrinsics;
	std::shared_ptr<isam::FiducialIntrinsics_Prior> _intrinsicsPrior;
	bool _optimizeIntrinsics;
};

class TargetRegistration : public RegistrationBase
{
public:

	typedef std::shared_ptr<TargetRegistration> Ptr;

	TargetRegistration( const std::string& n,
	                    GraphOptimizer& g,
	                    ros::NodeHandle& nh,
	                    ros::NodeHandle& ph );

	// TODO Clean up interface
	void InitializePose( const ros::Time& time, const PoseSE3& pose );
	
	// Returns whether a pose node at the specified time would be initialized
	// from odometry, priors, etc.
	bool IsPoseInitialized( const ros::Time& time ) const;
	bool IsPoseOptimizing() const;

	// Retrieves or creates a pose node at the specified time
	// If dynamic mode, integrates odometry to initialize pose
	// If discontinuous mode, initializes to previous value
	isam::PoseSE3_Node* GetPoseNode( const ros::Time& t );

	const std::vector<CameraRegistration::Ptr>& GetCameras() const;
	const std::vector<FiducialRegistration::Ptr>& GetFiducials() const;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

	typedef PoseGraph<isam::PoseSE3, ros::Time> PoseGraphType;
	typedef StaticPoseGraph<isam::PoseSE3, ros::Time> StaticGraphType;
	typedef OdometryGraph<isam::PoseSE3, ros::Time> OdomGraphType;
	typedef JumpPoseGraph<isam::PoseSE3, ros::Time> JumpGraphType;

	TargetType _type;

	ros::Subscriber _detSub;
	ros::Subscriber _odomSub;

	ros::Time _lastTime;
	VelocityIntegratorSE3 _velocityIntegrator;
	PoseSE3::CovarianceMatrix _odomOffset;

	std::vector<CameraRegistration::Ptr> _cameras;
	std::vector<FiducialRegistration::Ptr> _fiducials;

	PoseGraphType::Ptr _poses;
	bool _initialized;
	bool _optimizePose;

	void OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg );
	// TODO Support more message types for velocity integration
};
}
