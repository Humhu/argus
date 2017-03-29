#include "manycal/CameraArrayCalibrator.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

#include <boost/foreach.hpp>

namespace argus
{

CameraArrayCalibrator::CameraArrayCalibrator( const ros::NodeHandle& nh )
	: _fiducialManager( _lookupInterface ),
	_extrinsicsManager( nh )
{
	SetEnableSynchronization( false );
	SetReferenceFrame( "/world" );
	SetPriorCovariance( PoseSE3::CovarianceMatrix::Identity() );
}

void CameraArrayCalibrator::ReadParams( const ros::NodeHandle& ph )
{
	if( ph.hasParam( "sync" ) )
	{
		SetEnableSynchronization( true );
		double dt;
		unsigned int buffLen, minNum;
		GetParamRequired( ph, "sync/max_dt", dt );
		GetParamRequired( ph, "sync/buff_len", buffLen );
		GetParamRequired( ph, "sync/min_sync_num", minNum );
		_sync.SetMaxDt( dt );
		_sync.SetBufferLength( buffLen );
		_sync.SetMinSyncNum( minNum );
	}
	else
	{
		SetEnableSynchronization( false );
	}

	std::string frame;
	if( GetParam( ph, "reference_frame", frame ) )
	{
		SetReferenceFrame( frame );
	}

	PoseSE3::CovarianceMatrix cov;
	if( GetParam( ph, "prior_covariance", cov ) )
	{
		SetPriorCovariance( cov );
	}

	if( ph.hasParam( "lookup" ) )
	{
		ros::NodeHandle lh( ph.resolveName( "lookup" ) );
		std::string lookupNamespace;
		GetParam<std::string>( lh, "lookup_namespace", lookupNamespace, "/lookup" );
		_lookupInterface.SetLookupNamespace( lookupNamespace );
		_extrinsicsManager.ReadParams( lh );
	}

	if( ph.hasParam( "optimizer" ) )
	{
		 ros::NodeHandle oh( ph.resolveName( "optimizer" ) );
		_optimizer = GraphOptimizer( oh );
	}
}

void CameraArrayCalibrator::SetEnableSynchronization( bool e )
{
	// TODO Warn that synchronizer will not register old cameras?
	_useSynchronization = e;
}

void CameraArrayCalibrator::SetReferenceFrame( const std::string& f )
{
	_referenceFrame = f;
}

void CameraArrayCalibrator::SetPriorCovariance( const PoseSE3::CovarianceMatrix& mat )
{
	if( !TestPositiveDefinite( mat ) )
	{
		std::cout << "mat: " << std::endl << mat << std::endl;
		throw std::invalid_argument( "Prior must be positive-definite!" );
	}
	_priorCovariance = mat;
}

// bool CameraArrayCalibrator::WriteResults( manycal::WriteCalibration::Request& req,
//                                           manycal::WriteCalibration::Response& res )
// {
// 	YAML::Node yaml;
// 	BOOST_FOREACH( const CameraRegistry::value_type & item, _cameraRegistry )
// 	{
// 		const std::string& name = item.first;
// 		const CameraRegistration& registration = item.second;
// 		PoseSE3 extrinsics = registration.extrinsics->value().pose;
// 		// ROS_INFO_STREAM( "Camera " << name << " pose " << extrinsics );

// 		// ExtrinsicsInfo info;
// 		// info.extrinsics = extrinsics;
// 		// info._referenceFrame = _referenceFrame;
// 		// extrinsicsManager.WriteMemberInfo( name, info, true );

// 		// YAML::Node node;
// 		// PopulateExtrinsicsCalibration( extrinsicsManager.GetInfo( name ), node );
// 		// yaml[name] = node;
// 	}

// 	// std::ofstream resultsFile( req.calibrationPath );
// 	// if( !resultsFile.is_open() )
// 	// {
// 	//  ROS_ERROR_STREAM( "Could not open results file at: " << req.calibrationPath );
// 	//  return false;
// 	// }
// 	// ROS_INFO_STREAM( "Writing results to " << req.calibrationPath );
// 	// resultsFile << yaml;
// 	return true;
// }

std::vector<FiducialObjectCalibration> CameraArrayCalibrator::GetFiducials() const
{
	std::vector<FiducialObjectCalibration> fids;
	FiducialObjectCalibration cal;

	typedef FiducialRegistry::value_type Item;
	BOOST_FOREACH( const Item &item, _fiducialRegistry )
	{
		const std::string& name = item.first;
		const FiducialRegistration& reg = item.second;

		cal.intrinsics = IsamToFiducial( reg.intrinsics->value() );

		typedef std::map<ros::Time, isam::PoseSE3_Node::Ptr>::value_type Subitem;
		BOOST_FOREACH( const Subitem &sub, reg.poses )
		{
			const ros::Time& t = sub.first;
			const isam::PoseSE3_Node::Ptr& pose = sub.second;
			std::stringstream ss;
			ss << name << "_" << t;
			cal.name = ss.str();
			cal.extrinsics = pose->value().pose;
			fids.push_back( cal );
		}
	}
	return fids;
}

std::vector<CameraObjectCalibration> CameraArrayCalibrator::GetCameras() const
{
	std::vector<CameraObjectCalibration> cams;
	CameraObjectCalibration cal;

	typedef CameraRegistry::value_type Item;
	BOOST_FOREACH( const Item &item, _cameraRegistry )
	{
		const std::string& name = item.first;
		const CameraRegistration& reg = item.second;
		if( !reg.extrinsics ) { continue; }

		cal.name = name;
		cal.extrinsics = reg.extrinsics->value().pose;
		// cal.intrinsics = IsamToMonocular( reg.intrinsics->value() );
		cams.push_back( cal );
	}
	return cams;
}

void CameraArrayCalibrator::RegisterCamera( const std::string& name )
{
	WriteLock lock( _mutex );

	if( _cameraRegistry.count( name ) > 0 )
	{
		throw std::invalid_argument( "Camera " + name + " already registered!" );
	}

	_cameraRegistry[name];
	try
	{
		PoseSE3 extrinsics = _extrinsicsManager.GetExtrinsics( name, _referenceFrame );
		ROS_INFO_STREAM( "Found prior for camera: " << name );
		FillCameraRegistration( name, extrinsics, true, lock );
	}
	catch( ExtrinsicsException& e ) 
	{
		ROS_INFO_STREAM( "Could not find prior for camera: " << name );
	}

	if( _useSynchronization )
	{
		_sync.RegisterSource( name );
	}
}

void CameraArrayCalibrator::RegisterFiducial( const std::string& name )
{
	WriteLock lock( _mutex );
	
	if( _fiducialRegistry.count( name ) > 0 )
	{
		throw std::invalid_argument( "Fiducial " + name + " already registered!" );
	}

	// Force lookup of fiducial intrinsics
	if( !_fiducialManager.CheckMemberInfo( name, true ) )
	{
		throw std::runtime_error( "Could not retrieve intrinsics for " + name );
	}

	FiducialRegistration& registration = _fiducialRegistry[name];
	isam::FiducialIntrinsics intrinsics = FiducialToIsam( _fiducialManager.GetInfo( name ) );
	registration.intrinsics = std::make_shared<isam::FiducialIntrinsics_Node>
	                              ( intrinsics.name(), intrinsics.dim() );
	registration.intrinsics->init( intrinsics );
	// TODO Not optimizing intrinsics yet?
}

void CameraArrayCalibrator::BufferDetection( const ImageFiducialDetections& det )
{
	ReadLock lock( _mutex );

	if( _cameraRegistry.count( det.sourceName ) == 0 )
	{
		throw std::invalid_argument( "Camera " + det.sourceName + " not registered!" );
	}
	BOOST_FOREACH( const FiducialDetection &fd, det.detections )
	{
		if( _fiducialRegistry.count( fd.name ) == 0 )
		{
			throw std::invalid_argument( "Fiducial " + fd.name + " not registered!" );
		}
	}

	if( _useSynchronization )
	{
		_sync.BufferData( det.sourceName, det.timestamp.toSec(), det );
	}
	else
	{
		lock.release();
		WriteLock uplock( _mutex );
		_detBuffer.emplace_back( det );
	}
}

void CameraArrayCalibrator::Spin()
{
	WriteLock lock( _mutex );

	if( _useSynchronization )
	{
		std::vector<DetectionSynchronizer::KeyedStampedData> output;
		while( _sync.GetOutput( ros::Time::now().toSec(), output ) )
		{
			// Randomly pick the first timestamp TODO?
			ros::Time timestamp( std::get<1>( output[0] ) );
			BOOST_FOREACH( DetectionSynchronizer::KeyedStampedData & item, output )
			{
				ImageFiducialDetections& det = std::get<2>( item );
				det.timestamp = timestamp;
				_detBuffer.emplace_back( det );
			}
		}
	}
	
	// Process the cache
	std::vector<ImageFiducialDetections> recache;
	BOOST_FOREACH( const ImageFiducialDetections &dets, _detBuffer )
	{
		if( !ProcessDetection( dets, lock ) )
		{
			recache.push_back( dets );
		}
	}
	_detBuffer = recache;

	// Don't optimize when we don't have any observations!
	if( _observations.size() == 0 ) { return; }

	ROS_INFO_STREAM( "Optimizing..." );
	_optimizer.GetOptimizer().update();
}

bool CameraArrayCalibrator::ProcessDetection( const ImageFiducialDetections& dets,
                                              const WriteLock& lock )
{
	CheckLockOwnership( lock, &_mutex );

	// Attempt to initialize the camera if new
	CameraRegistration& camReg = _cameraRegistry.at( dets.sourceName );
	if( !camReg.extrinsics && !BootstrapInitializeCamera( dets, lock ) )
	{
		ROS_WARN_STREAM( "Could not bootstrap initialize camera: " << dets.sourceName );
		return false;
	}

	// Process detections
	std::string fidNames = "";
	BOOST_FOREACH( const FiducialDetection &det, dets.detections )
	{
		if( _fiducialRegistry.count( det.name ) == 0 ) { continue; }
		fidNames += det.name + " ";

		FiducialRegistration& fidReg = _fiducialRegistry[det.name];
		// Initialize fiducial if first detection at this time
		if( fidReg.poses.count( dets.timestamp ) == 0 )
		{
			PoseSE3 cameraPose = camReg.extrinsics->value().pose;
			PoseSE3 relPose = EstimateArrayPose( det,
			                                     IsamToFiducial( fidReg.intrinsics->value() ) );
			PoseSE3 fiducialPose = cameraPose * relPose;
			ROS_INFO_STREAM( "Initializing fiducial at pose: " << fiducialPose );
			isam::PoseSE3_Node::Ptr fidNode = std::make_shared<isam::PoseSE3_Node>();
			fidNode->init( isam::PoseSE3( fiducialPose ) );
			fidReg.poses[dets.timestamp] = fidNode;
			_optimizer.GetOptimizer().add_node( fidNode.get() );
		}

		isam::PoseSE3_Node::Ptr fidNode = fidReg.poses[dets.timestamp];

		double imageCoordinateErr = std::pow( 0.03, 2 );
		isam::Noise cov = isam::Covariance( imageCoordinateErr * isam::eye( 2 * det.points.size() ) );

		isam::FiducialFactor::Properties props;
		props.optCamReference = true; // Reference is actually extrinsics here
		props.optCamIntrinsics = false;
		props.optCamExtrinsics = false;
		props.optFidReference = true;
		props.optFidIntrinsics = false;
		props.optFidExtrinsics = false;

		isam::FiducialFactor::Ptr factor = std::make_shared<isam::FiducialFactor>
		                                       ( camReg.extrinsics.get(),
		                                       camReg.intrinsics.get(),
		                                       nullptr,
		                                       fidNode.get(),
		                                       fidReg.intrinsics.get(),
		                                       nullptr,
		                                       DetectionToIsam( det ),
		                                       cov,
		                                       props );
		ROS_INFO_STREAM( "Adding detection from " << dets.sourceName << " of " << fidNames );
		_optimizer.GetOptimizer().add_factor( factor.get() );
		_observations.push_back( factor );
	}

	return true;
}

// Attempt to initialize a camera from the detection
bool CameraArrayCalibrator::BootstrapInitializeCamera( const ImageFiducialDetections& dets,
                                                       const WriteLock& lock )
{
	CheckLockOwnership( lock, &_mutex );

	// If not, see if there if any observed fiducial have poses
	BOOST_FOREACH( const FiducialDetection &det, dets.detections )
	{
		FiducialRegistration& fidReg = _fiducialRegistry.at( det.name );

		// If fiducial pose at this time is not initialized yet
		if( fidReg.poses.count( dets.timestamp ) == 0 ) { continue; }

		PoseSE3 fiducialPose = fidReg.poses[dets.timestamp]->value().pose;
		PoseSE3 relPose = EstimateArrayPose( det,
		                                     IsamToFiducial( fidReg.intrinsics->value() ) );
		PoseSE3 cameraPose = fiducialPose * relPose.Inverse();
		FillCameraRegistration( dets.sourceName, cameraPose, false, lock );
		return true;
	}
	return false;
}

void CameraArrayCalibrator::FillCameraRegistration( const std::string& name,
                                                    const PoseSE3& pose,
                                                    bool addPrior,
													const WriteLock& lock )
{
	CheckLockOwnership( lock, &_mutex );

	ROS_INFO_STREAM( "Initializing camera " << name << " at pose " << pose );

	CameraRegistration& registration = _cameraRegistry.at( name );
	registration.extrinsics = std::make_shared<isam::PoseSE3_Node>();
	registration.extrinsics->init( isam::PoseSE3( pose ) );
	_optimizer.GetOptimizer().add_node( registration.extrinsics.get() );

	registration.intrinsics = std::make_shared<isam::MonocularIntrinsics_Node>();
	registration.intrinsics->init( isam::MonocularIntrinsics( 1, 1, Eigen::Vector2d( 0, 0 ) ) );
	// TODO: Not optimizing intrinsics yet?

	if( addPrior )
	{
		ROS_INFO_STREAM( "Adding a prior for " << name << " at " << pose <<
		                 " with covariance: " << std::endl << _priorCovariance );
		isam::Noise priorCov = isam::Covariance( _priorCovariance );
		registration.extrinsicsPrior =
		    std::make_shared<isam::PoseSE3_Prior>( registration.extrinsics.get(),
		                                           isam::PoseSE3( pose ),
		                                           priorCov );
		_optimizer.GetOptimizer().add_factor( registration.extrinsicsPrior.get() );
	}
}

}
