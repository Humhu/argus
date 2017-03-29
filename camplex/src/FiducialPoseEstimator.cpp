#include "camplex/FiducialPoseEstimator.h"

#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/MatrixUtils.h"

#include "geometry_msgs/TransformStamped.h"

#include "camplex/FiducialArray.h"

#include <boost/foreach.hpp>

#include <unordered_map>

namespace argus
{
	
FiducialPoseEstimator::FiducialPoseEstimator( ros::NodeHandle& nh, 
                                              ros::NodeHandle& ph )
: _fiducialManager( _lookupInterface ),
  _extrinsicsInterface( nh, ph )
{
	GetParam<std::string>( ph, "reference_frame", _refFrame, "" );

	unsigned int inBuffSize, outBuffSize;
	GetParam( ph, "input_buffer_size", inBuffSize, (unsigned int) 20 );
	GetParam( ph, "output_buffer_size", outBuffSize, (unsigned int) 20 );

	_detSub = nh.subscribe( "detections", 
	                        inBuffSize, 
	                        &FiducialPoseEstimator::DetectionsCallback, 
	                        this );
	_posePub = ph.advertise<geometry_msgs::TransformStamped>( "relative_pose", 
	                                                         outBuffSize );
}

bool FiducialPoseEstimator::GetFiducial( const std::string& name,
                                         const ros::Time& time,
                                         Fiducial& fid )
{
	// Force lookup of fiducials in case initialization is slow
	// NOTE This means "rogue" undocumented fiducials will slow the system down
	Fiducial raw;
	if( !_fiducialManager.HasMember( name ) )
	{
		if( !_fiducialManager.ReadMemberInfo( name, true ) ) 
		{
			ROS_INFO_STREAM( "Could not read intrinsics for " << name );
			return false;
		}
	}
	raw = _fiducialManager.GetInfo( name );
	
	PoseSE3 extrinsics;
	try
	{
		extrinsics = _extrinsicsInterface.GetExtrinsics( name, _refFrame, time );
	}
	catch( ExtrinsicsException& ex )
	{
		ROS_INFO_STREAM( "Could not get extrinsics for " << name << std::endl << ex.what() );
		return false;
	}
	fid = raw.Transform( extrinsics );
	return true;
}

void FiducialPoseEstimator::DetectionsCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
{
	const std::string& cameraName = msg->header.frame_id;
	const ros::Time& detTime = msg->header.stamp;
	
	std::vector<Fiducial> fids;
	std::vector<FiducialDetection> detections;
	// 1. Process all fiducials
	BOOST_FOREACH( const argus_msgs::FiducialDetection& det, msg->detections )
	{
		Fiducial fid;
		if( !GetFiducial( det.name, detTime, fid ) ) { continue; }
		fids.push_back( fid );
		detections.push_back( det );
	}
	if( detections.empty() )
	{
		return;
	}
	PoseSE3 relPose = EstimateArrayPose( detections, fids );

	geometry_msgs::TransformStamped poseMsg;
	poseMsg.header.stamp = msg->header.stamp;
	poseMsg.header.frame_id = _refFrame;
	poseMsg.child_frame_id = cameraName;
	poseMsg.transform = PoseToTransform( relPose );
	_posePub.publish( poseMsg );
}
	
} // end namespace fieldtrack
