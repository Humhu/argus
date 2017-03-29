#include "manycal/FiducialArrayCalibrator.h"
#include "argus_utils/utils/ParamUtils.h"

#include <boost/foreach.hpp>

namespace argus
{
	
FiducialArrayCalibrator::FiducialArrayCalibrator( ros::NodeHandle& nh,
                                                  ros::NodeHandle& ph )
: _fiducialManager( _lookupInterface ),
  _extrinsicsManager( nh, ph ), 
  _detCounter( 0 )
{
	GetParamRequired( ph, "reference_frame", _referenceFrame );
	GetParam( ph, "batch_period", _batchPeriod, (unsigned int) 10 );
	GetParam( ph, "min_detections_per_image", _minDetectionsPerImage, (unsigned int) 2 );
	GetParam( ph, "prior_covariance", _priorCovariance, PoseSE3::CovarianceMatrix::Identity() );
	
	std::string lookupNamespace;
	GetParam<std::string>( ph, "lookup_namespace", lookupNamespace, "/lookup" );
	_lookupInterface.SetLookupNamespace( lookupNamespace );

	// Create optimizer object
	ros::NodeHandle oh( ph.resolveName( "optimizer" ) );
	_optimizer = GraphOptimizer( oh );
	
	// Create camera intrinsics node
	_cameraIntrinsics = std::make_shared <isam::MonocularIntrinsics_Node>();
	_cameraIntrinsics->init( isam::MonocularIntrinsics( 1.0, 1.0, Eigen::Vector2d( 0, 0 ) ) );

	// Create fiducial reference pose node
	_fiducialReference = std::make_shared<isam::PoseSE3_Node>();
	_fiducialReference->init( isam::PoseSE3() );
	
	unsigned int buffSize;
	GetParam( ph, "buffer_size", buffSize, (unsigned int) 10 );
	_detSub = nh.subscribe( "detections", 
	                        buffSize, 
	                        &FiducialArrayCalibrator::DetectionCallback,
	                        this );
}

void FiducialArrayCalibrator::WriteResults() 
{
	BOOST_FOREACH( const FiducialRegistry::value_type& item, _fiducialRegistry )
	{
		const std::string& name = item.first;
		const FiducialRegistration& registration = item.second;
		PoseSE3 extrinsics = registration.extrinsics->value().pose;
		ROS_INFO_STREAM( "Fiducial " << name << " pose " << extrinsics );
		
		// TODO Write to a YAML file
		// ExtrinsicsInfo info;
		// info.referenceFrame = _referenceFrame;
		// info.extrinsics = extrinsics;
		// _extrinsicsManager.WriteMemberInfo( name, info, true );
	}
}

isam::PoseSE3_Node::Ptr
FiducialArrayCalibrator::InitializeCameraPose( const std::vector<FiducialDetection>& detections )
{
	isam::PoseSE3_Node::Ptr cameraNode;
	BOOST_FOREACH( const FiducialDetection& det, detections )
	{
		// Attempt to initialize unregistered fiducials from prior info
		// If we can't, continue to the next detection
		if( _fiducialRegistry.count( det.name ) == 0 &&
		    !InitializeFiducialFromPrior( det.name ) ) { continue; }

		// If we have a registered fiducial, use it to initialize
		if( _fiducialRegistry.count( det.name ) > 0 )
		{
			PoseSE3 relPose = EstimateArrayPose( det,
			                                     _fiducialManager.GetInfo( det.name ) );
			PoseSE3 fiducialPose = _fiducialRegistry[ det.name ].extrinsics->value().pose;
			PoseSE3 cameraPose = fiducialPose * relPose.Inverse();
			
			cameraNode = std::make_shared <isam::PoseSE3_Node>();
			cameraNode->init( isam::PoseSE3( cameraPose ) );
			break;
		}
	}
	return cameraNode;
}

void 
FiducialArrayCalibrator::DetectionCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
{
	if( msg->detections.size() < _minDetectionsPerImage )
	{
		ROS_WARN_STREAM( "Only " << msg->detections.size() << " detections, less than " <<
		                 _minDetectionsPerImage << " minimum. Skipping..." );
		return;
	}

	// Convert to C++
	std::vector<FiducialDetection> detections;
	detections.reserve( msg->detections.size() );
	BOOST_FOREACH( const argus_msgs::FiducialDetection& d, msg->detections )
	{
		detections.emplace_back( d );
	}

	isam::PoseSE3_Node::Ptr cameraNode = InitializeCameraPose( detections );
	if( !cameraNode )
	{
		ROS_WARN_STREAM( "Could not initialize camera pose." );
		return;
	}
	_cameraPoses.push_back( cameraNode );
	_optimizer.GetOptimizer().add_node( cameraNode.get() );
	
	// Create observation factors for each fiducial detection
	BOOST_FOREACH( const FiducialDetection& det, detections )
	{
		// if the fiducial is not registered and we can't get intrinsics, skip it
		if( _fiducialRegistry.count( det.name ) == 0 
		    && !_fiducialManager.CheckMemberInfo( det.name ) ) { continue; }
		
		const Fiducial& intrinsics = _fiducialManager.GetInfo( det.name );
		PoseSE3 relPose = EstimateArrayPose( det, intrinsics );
		PoseSE3 cameraPose = cameraNode->value().pose;
		PoseSE3 fiducialPose = cameraPose * relPose;
		RegisterFiducial( det.name, fiducialPose, false );
		
		const FiducialRegistration& reg = _fiducialRegistry[ det.name ];
		
		// TODO Formalize this error model?
		double imageCoordinateErr = std::pow( 0.03, 2 );
		isam::Noise cov = isam::Covariance( imageCoordinateErr * isam::eye( 2*det.points.size() ) );
		
		isam::FiducialFactor::Properties props;
		props.optCamReference = true;
		props.optCamIntrinsics = false;
		props.optCamExtrinsics = false;
		props.optFidReference = false;
		props.optFidIntrinsics = false;
		props.optFidExtrinsics = true;
		
		isam::FiducialFactor::Ptr factor = std::make_shared<isam::FiducialFactor>
		    ( cameraNode.get(), 
		      _cameraIntrinsics.get(), 
		      nullptr,
		      _fiducialReference.get(), 
		      reg.intrinsics.get(), 
		      reg.extrinsics.get(),
		      DetectionToIsam( det ), 
		      cov, 
		      props );
		_optimizer.GetOptimizer().add_factor( factor.get() );
		_observations.push_back( factor );
	}

	if( _detCounter % _batchPeriod == 0 )
	{
		_optimizer.GetOptimizer().batch_optimization();
		_optimizer.GetOptimizer().print_graph();
	}
	else
	{
		_optimizer.GetOptimizer().update();
	}
	_detCounter++;
}

bool FiducialArrayCalibrator::InitializeFiducialFromPrior( const std::string& name )
{
	if( _fiducialRegistry.count( name ) > 0 ) { return true; }

	// If not registered, attempt initialization
	// First check intrinsics
	if( !_fiducialManager.CheckMemberInfo( name ) ) { return false; }
	// Then check extrinsics
	try
	{
		PoseSE3 ext = _extrinsicsManager.GetExtrinsics( name,
		                                                _referenceFrame );
		RegisterFiducial( name, ext, true );
		return true;
	}
	catch( ExtrinsicsException& e ) { return false; }
}

void FiducialArrayCalibrator::RegisterFiducial( const std::string& name,
                                                const PoseSE3& pose,
                                                bool addPrior )
{
	if( _fiducialRegistry.count( name ) > 0 ) { return; }
	
	ROS_INFO_STREAM( "Registering fiducial " << name << " with pose " << pose );
	
	FiducialRegistration registration;
	isam::FiducialIntrinsics intrinsics = FiducialToIsam( _fiducialManager.GetInfo( name ) );
	registration.intrinsics = 
	    std::make_shared<isam::FiducialIntrinsics_Node>( intrinsics.name(), intrinsics.dim() );
	registration.intrinsics->init( intrinsics );
	// NOTE Not optimizing intrinsics, so we don't have to add it to the SLAM object
	
	registration.extrinsics = std::make_shared<isam::PoseSE3_Node>();
	registration.extrinsics->init( isam::PoseSE3( pose ) );
	_optimizer.GetOptimizer().add_node( registration.extrinsics.get() );
	
	if( addPrior )
	{
		isam::Noise priorCov = isam::Covariance( _priorCovariance );
		registration.extrinsicsPrior = 
			std::make_shared<isam::PoseSE3_Prior>( registration.extrinsics.get(),
			                                       isam::PoseSE3( pose ),
			                                       priorCov );
		_optimizer.GetOptimizer().add_factor( registration.extrinsicsPrior.get() );
	}

	_fiducialRegistry[ name ] = registration;
}

}
