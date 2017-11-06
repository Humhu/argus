#include "manycal/Registrations.h"

#include "argus_utils/utils/ParamUtils.h"
#include "extrinsics_array/ExtrinsicsInterface.h"
#include "camplex/FiducialInfoManager.h"

namespace argus
{
RegistrationBase::RegistrationBase( const std::string& n, GraphOptimizer& g )
	: _name( n ), _graph( g ) {}

TargetRegistration::TargetRegistration( const std::string& n,
                                        GraphOptimizer& g,
                                        ros::NodeHandle& nh,
                                        ros::NodeHandle& ph )
	: RegistrationBase( n, g )
{
	std::string type;
	GetParamRequired( ph, "type", type );
	_type = string_to_target( type );
	ROS_INFO_STREAM( "Target: " << _name << " initializing as type: " << type );

	if( _type == TargetType::TARGET_STATIC )
	{
		_poses = std::make_shared<StaticGraphType>( _graph );
	}
	else if( _type == TargetType::TARGET_DYNAMIC )
	{
		double tLen;
		unsigned int buffLen;
		std::string topic;
		GetParam( ph, "integrator_buffer_len", tLen, 10.0 );
		_velocityIntegrator.SetMaxBuffLen( tLen );
		GetParam( ph, "odom_buffer_len", buffLen, (unsigned int) 10 );
		GetParamRequired( ph, "odom_topic", topic );
		_odomSub = nh.subscribe( topic,
								 buffLen,
								 &TargetRegistration::OdometryCallback,
								 this );
		ROS_INFO_STREAM( "Target: " << _name << " subscribing to odometry on "
									<< topic );

		_poses = std::make_shared<OdomGraphType>( _graph );
	}
	else if( _type == TargetType::TARGET_DISCONTINUOUS )
	{
		_poses = std::make_shared<JumpGraphType>( _graph );
	}
	else
	{
		throw std::invalid_argument( "Unknown graph type" );
	}

	_initialized = false;
	GetParam( ph, "optimize_pose", _optimizePose, false );
	PoseSE3 initPose;
	if( GetParam( ph, "initial_pose", initPose ) )
	{
		InitializePose( ros::Time::now(), initPose );
	}

	bool prefix;
	GetParam( ph, "prefix_members", prefix, false );

	// Initialize all member cameras
	YAML::Node camInfo;
	GetParam( ph, "cameras", camInfo );
	YAML::Node::const_iterator camIter;
	for( camIter = camInfo.begin(); camIter != camInfo.end(); ++camIter )
	{
		const std::string rawName = camIter->first.as<std::string>();
		std::string camName = prefix ? _name + "/" + rawName : rawName;
		ROS_INFO_STREAM( "Parsing camera " << rawName << " with name " << camName );
		ros::NodeHandle ch( ph.resolveName( "cameras/" + rawName ) );

		CameraRegistration::Ptr cam = std::make_shared<CameraRegistration>( *this,
		                                                                    camName,
		                                                                    _graph,
		                                                                    ch );
		_cameras.push_back( cam );
	}

	// Initialize all member fiducials
	YAML::Node fidsInfo;
	GetParam( ph, "fiducials", fidsInfo );
	YAML::Node::const_iterator fidIter;
	for( fidIter = fidsInfo.begin(); fidIter != fidsInfo.end(); ++fidIter )
	{
		const std::string rawName = fidIter->first.as<std::string>();
		std::string fidName = prefix ? _name + "/" + rawName : rawName;
		ROS_INFO_STREAM( "Parsing fiducial " << rawName << " with name " << fidName );
		ros::NodeHandle ch( ph.resolveName( "fiducials/" + rawName ) );

		FiducialRegistration::Ptr fid = std::make_shared<FiducialRegistration>( *this,
		                                                                        fidName,
		                                                                        _graph,
		                                                                        ch );
		_fiducials.push_back( fid );
	}
}

void TargetRegistration::InitializePose( const ros::Time& time,
                                         const PoseSE3& pose )
{
	isam::PoseSE3 p = PoseToIsam( pose );
	_poses->CreateNode( time, p );
	// TODO
	PoseSE3::CovarianceMatrix cov = PoseSE3::CovarianceMatrix::Identity();
	_poses->CreatePrior( time, p, isam::Covariance( cov ) );
}

bool TargetRegistration::IsPoseOptimizing() const { return _optimizePose; }

isam::PoseSE3_Node* TargetRegistration::GetPoseNode( const ros::Time& t )
{
	// If the node already exists, return it
	isam::PoseSE3_Node* node;
	node = _poses->RetrieveNode( t ).get();
	if( node ) { return node; }

	// Else we have to create the pose node
	ROS_INFO_STREAM( "Creating pose node for " << _name << " at " << t );
	if( _type == TargetType::TARGET_DYNAMIC )
	{
		PoseSE3 disp;
		PoseSE3::CovarianceMatrix cov;
		bool succ = !_velocityIntegrator.Integrate( _lastTime.toSec(),
		                                            t.toSec(),
		                                            disp,
		                                            cov );
		if( !succ )
		{
			ROS_ERROR_STREAM( "Could not integrate odometry" );
			return nullptr;
		}

		_poses->CreateEdge( _lastTime, t,
		                    PoseToIsam( disp ), isam::Covariance( cov ) );
	}
	else if( _type == TargetType::TARGET_DISCONTINUOUS )
	{
		// TODO
		_poses->CreateNode( t, PoseSE3() );
	}

	_lastTime = t;
	return _poses->RetrieveNode( t ).get();
}

const std::vector<CameraRegistration::Ptr>& TargetRegistration::GetCameras() const
{
	return _cameras;
}

const std::vector<FiducialRegistration::Ptr>& TargetRegistration::GetFiducials() const
{
	return _fiducials;
}

bool TargetRegistration::IsPoseInitialized( const ros::Time& time ) const
{
	if( _type == TargetType::TARGET_DYNAMIC )
	{
		OdomGraphType::Ptr poses = std::dynamic_pointer_cast<OdomGraphType>( _poses );

		// If time is within bounds of existing nodes
		if( poses->InRange( time ) ) { return poses->IsGrounded( time ); }

		// If time precedes bounds
		if( time < poses->EarliestIndex() ) { return false; }

		// Otherwise, see if we can integrate forward to required time
		// NOTE _lastTime should == poses->LatestIndex()
		PoseSE3 disp;
		PoseSE3::CovarianceMatrix cov;
		bool succ = !_velocityIntegrator.Integrate( _lastTime.toSec(),
		                                            time.toSec(),
		                                            disp,
		                                            cov );
		return succ && poses->IsGrounded( time );
	}
	// TODO Need a way to tell if JumpGraph is initialized or not, since we will never
	// actually be adding priors to it
	else
	{
		return _poses->IsGrounded( time );
	}
}

void TargetRegistration::OdometryCallback( const nav_msgs::Odometry::ConstPtr& msg )
{
	PoseSE3::TangentVector vel = MsgToTangent( msg->twist.twist );
	PoseSE3::CovarianceMatrix cov;
	ParseMatrix( msg->twist.covariance, cov ); // TODO Check return?
	_velocityIntegrator.BufferInfo( msg->header.stamp.toSec(), vel, cov );
}

ExtrinsicsRegistration::ExtrinsicsRegistration( TargetRegistration& p,
                                                const std::string& n,
                                                GraphOptimizer& g,
                                                ros::NodeHandle& ph )
	: RegistrationBase( n, g ), parent( p ), _extInitialized( false )
{
	GetParam( ph, "optimize_extrinsics", _optimizeExtrinsics, false );

	_extrinsics = std::make_shared<isam::PoseSE3_Node>();
	// NOTE We initialize it here so that if node->value() is called it won't segfault
	_extrinsics->init( PoseToIsam( PoseSE3() ) );

	PoseSE3 initExt;
	if( GetParam( ph, "initial_extrinsics", initExt ) )
	{
		if( _optimizeExtrinsics )
		{
			PoseSE3::CovarianceMatrix cov;
			GetParam( ph, "extrinsics_prior_cov", cov, 1E-3 * PoseSE3::CovarianceMatrix::Identity() );
			InitializeExtrinsics( initExt, cov );
		}
		else
		{
			InitializeExtrinsics( initExt );
		}
	}
}

void ExtrinsicsRegistration::InitializeExtrinsics( const PoseSE3& pose,
                                                   const PoseSE3::CovarianceMatrix& cov )
{
	isam::PoseSE3 p = PoseToIsam( pose );
	_extrinsics->init( p );
	if( _optimizeExtrinsics )
	{
		if( !_extInitialized )
		{
			_graph.AddNode( _extrinsics );
		}
		if( _extrinsicsPrior )
		{
			_graph.RemoveFactor( _extrinsicsPrior );
		}

		// TODO Covariance argument
		_extrinsicsPrior = std::make_shared<isam::PoseSE3_Prior>( _extrinsics.get(),
		                                                          isam::PoseSE3( p ),
		                                                          isam::Covariance( cov ) );
		
		_graph.AddFactor( _extrinsicsPrior );
	}
	_extInitialized = true;
}

bool ExtrinsicsRegistration::IsExtrinsicsInitialized() const
{
	return _extInitialized;
}

bool ExtrinsicsRegistration::IsExtrinsicsOptimizing() const
{
	return _optimizeExtrinsics;
}

PoseSE3 ExtrinsicsRegistration::GetExtrinsicsPose() const
{
	return IsamToPose( _extrinsics->value() );
}

isam::PoseSE3_Node* ExtrinsicsRegistration::GetExtrinsicsNode()
{
	return _extrinsics.get();
}

isam::PoseSE3_Prior* ExtrinsicsRegistration::GetExtrinsicsPrior()
{
	return _extrinsicsPrior.get();
}

CameraRegistration::CameraRegistration( TargetRegistration& p,
                                        const std::string& n,
                                        GraphOptimizer& g,
                                        ros::NodeHandle& ph )
	: ExtrinsicsRegistration( p, n, g, ph ), _intInitialized( false )
{
	GetParam( ph, "optimize_intrinsics", _optimizeIntrinsics, false );
	_intrinsics = std::make_shared<isam::MonocularIntrinsics_Node>();

	CameraCalibration cal; // TODO
	if( _optimizeIntrinsics )
	{
		FixedMatrixType<4, 4> cov;
		GetParam( ph, "intrinsics_prior_cov", cov, 1E-3 * FixedMatrixType<4, 4>::Identity() );
		InitializeIntrinsics( cal, cov );
	}
	else
	{
		InitializeIntrinsics( cal );
	}
}

void CameraRegistration::InitializeIntrinsics( const CameraCalibration& intrinsics,
                                               const FixedMatrixType<4, 4>& cov )
{
	isam::MonocularIntrinsics intr = CalibrationToIsam( intrinsics );
	_intrinsics->init( intr );
	if( _optimizeIntrinsics )
	{
		if( !_intInitialized )
		{
			_graph.AddNode( _intrinsics );
		}

		if( _intrinsicsPrior )
		{
			_graph.RemoveFactor( _intrinsicsPrior );
		}

		_intrinsicsPrior = std::make_shared<isam::MonocularIntrinsics_Prior>( _intrinsics.get(),
		                                                                      intr,
		                                                                      isam::Covariance( cov ) );
		_graph.AddFactor( _intrinsicsPrior );
	}
	_intInitialized = true;
}

bool CameraRegistration::IsIntrinsicsInitialized() const { return _intInitialized; }

bool CameraRegistration::IsIntrinsicsOptimizing() const { return _optimizeIntrinsics; }

isam::MonocularIntrinsics_Node* CameraRegistration::GetIntrinsicsNode()
{
	return _intrinsics.get();
}

isam::MonocularIntrinsics_Prior* CameraRegistration::GetIntrinsicsPrior()
{
	return _intrinsicsPrior.get();
}

FiducialRegistration::FiducialRegistration( TargetRegistration& p,
                                            const std::string& n,
                                            GraphOptimizer& g,
                                            ros::NodeHandle& ph )
	: ExtrinsicsRegistration( p, n, g, ph ), _intInitialized( false )
{
	GetParam( ph, "optimize_intrinsics", _optimizeIntrinsics, false );

	LookupInterface lookup;
	FiducialInfoManager fiducials( lookup );

	if( fiducials.CheckMemberInfo( n ) )
	{
		Fiducial intr = fiducials.GetInfo( n );
		if( _optimizeIntrinsics )
		{
			unsigned int N = 2 * intr.points.size();
			MatrixType cov( N, N );
			GetParam( ph, "intrinsics_prior_cov", cov, 1E-3 * MatrixType::Identity( N, N ) );
			InitializeIntrinsics( intr, cov );
		}
		else
		{
			InitializeIntrinsics( intr );
		}
	}
	else
	{
		ROS_WARN_STREAM( "Could not find fiducial intrinsics for " << n );
	}
}

void FiducialRegistration::InitializeIntrinsics( const Fiducial& intrinsics, const MatrixType& cov )
{
	isam::FiducialIntrinsics intr = FiducialToIsam( intrinsics );

	if( _intrinsics ) // _intrinsicsPrior should always also be valid
	{
		_graph.RemoveNode( _intrinsics );
		_graph.RemoveFactor( _intrinsicsPrior );
	}

	_intrinsics = std::make_shared<isam::FiducialIntrinsics_Node>( intr.dim() );
	_intrinsics->init( intr );
	if( _optimizeIntrinsics )
	{
		_graph.AddNode( _intrinsics );
		_intrinsicsPrior = std::make_shared<isam::FiducialIntrinsics_Prior>( _intrinsics.get(),
		                                                                     intr,
		                                                                     isam::Covariance( cov ) );
		_graph.AddFactor( _intrinsicsPrior );
	}
	_intInitialized = true;
}

bool FiducialRegistration::IsIntrinsicsInitialized() const { return _intInitialized; }

bool FiducialRegistration::IsIntrinsicsOptimizing() const { return _optimizeIntrinsics; }

isam::FiducialIntrinsics_Node* FiducialRegistration::GetIntrinsicsNode()
{
	return _intrinsics.get();
}

isam::FiducialIntrinsics_Prior* FiducialRegistration::GetIntrinsicsPrior()
{
	return _intrinsicsPrior.get();
}
}