#include "fieldtrack/FiducialPoseEstimator.h"
#include "argus_utils/YamlUtils.h"
#include "argus_msgs/RelativePose.h"

#include "extrinsics_array/ExtrinsicsArray.h"
#include "fiducial_array/FiducialArray.h"
#include "fiducial_array/PoseEstimation.h"

#include <boost/foreach.hpp>

#include <unordered_map>

using namespace argus_utils;
using namespace extrinsics_array;
using namespace fiducial_array;

namespace fieldtrack
{
	
FiducialPoseEstimator::FiducialPoseEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: fidManager( nh ), camManager( nh )
{
	std::string lookupNamespace;
	if( !ph.getParam( "lookup_namespace", lookupNamespace ) ) 
	{
		ROS_ERROR_STREAM( "Lookup namespace must be specified." );
	}
	
	if( lookupNamespace.back() != '/' ) { lookupNamespace += "/"; }
	fidManager.SetLookupNamespace( lookupNamespace + "fiducials" );
	camManager.SetLookupNamespace( lookupNamespace + "cameras" );
	
	detSub = nh.subscribe( "detections", 20, &FiducialPoseEstimator::DetectionsCallback, this );
	posePub = nh.advertise<argus_msgs::RelativePose>( "relative_poses", 20 );
}

void FiducialPoseEstimator::DetectionsCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
{

	// Map from reference frame names to fiducial detections
	typedef std::unordered_map< std::string, std::vector< argus_msgs::FiducialDetection > > 
	        DetectionMap;
	DetectionMap sorter;
	
	// 0. Retrieve the camera array if needed
	std::string cameraName = msg->header.frame_id;
	if( !camManager.HasMember( cameraName ) )
	{
		if( !camManager.ReadMemberInformation( cameraName ) )
		{
			ROS_WARN_STREAM( "Could not retrieve extrinsics info for camera " << cameraName );
			return;
		}
	}
	
	// Retrieve the camera extrinsics and reference frame
	const ExtrinsicsArray& cameraArray = camManager.GetParentArray( cameraName );
	std::string cameraFrameName = cameraArray.GetReferenceFrame();
	PoseSE3 cameraExtrinsics = cameraArray.GetPose( cameraName );
	
	// 1. Group detected fiducials into arrays
	for( unsigned int i = 0; i < msg->detections.size(); i++ )
	{
		const argus_msgs::FiducialDetection& det = msg->detections[i];
		if( !fidManager.HasMember( det.name ) )
		{
			// We allow cached lookup here so that we don't keep querying the master
			// for an untrackable object
			if( !fidManager.ReadMemberInformation( det.name, false ) )
			{
			  // 				ROS_WARN_STREAM( "Could not retrieve extrinsics info for fiducial " << det.name );
				continue;
			}
			
		}
		
		const ExtrinsicsArray& fiducialArray = fidManager.GetParentArray( det.name );
		std::string fiducialFrameName = fiducialArray.GetReferenceFrame();
		sorter[ fiducialFrameName ].push_back( det );
	}
	
	// 2. Convert detections into relative pose observations
	// TODO Spin off array trackers instead so they can initialize the PnP with state
	BOOST_FOREACH( const DetectionMap::value_type& item, sorter )
	{
		const std::string& fiducialFrameName = item.first;
		const std::vector< argus_msgs::FiducialDetection >& detections = item.second;
		
		// Collect all fiducial points from all detections in order
		std::vector<cv::Point3f> fiducialFramePoints;
		std::vector<cv::Point2f> imageFramePoints;
		BOOST_FOREACH( const argus_msgs::FiducialDetection& detection, detections )
		{
			if( !detection.normalized )
			{
				ROS_WARN_STREAM( "Received unnormalized detection of " << detection.name );
				return;
			}
			
			const FiducialArray& fiducialArray = fidManager.GetParentFiducialArray( detection.name );
			const Fiducial& fiducial = fiducialArray.GetFiducialTransformed( detection.name );
			const std::vector<cv::Point3f>& fidPoints = MsgToPoints( fiducial.points );
			fiducialFramePoints.insert( fiducialFramePoints.end(), fidPoints.begin(), fidPoints.end() );
			
			const std::vector<cv::Point2f>& imgPoints = MsgToPoints( detection.points );
			imageFramePoints.insert( imageFramePoints.end(), imgPoints.begin(), imgPoints.end() );
		}
		
		PoseSE3 cameraRelativePose = EstimateArrayPose( imageFramePoints, nullptr, fiducialFramePoints );
		PoseSE3 frameRelativePose = cameraExtrinsics * cameraRelativePose;

		//EulerAngles euler = QuaternionToEuler( cameraRelativePose.GetQuaternion() );
		//EulerAngles finalEuler = QuaternionToEuler( frameRelativePose.GetQuaternion() );
		//ROS_INFO_STREAM( "Relative orientation of target to camera: " << euler );
		//ROS_INFO_STREAM( "Relative orientation of target to robot: " << finalEuler );

		argus_msgs::RelativePose poseMsg;
		poseMsg.observer_name = cameraFrameName;
		poseMsg.observer_time = msg->header.stamp;
		poseMsg.target_name = fiducialFrameName;
		poseMsg.target_time = msg->header.stamp;
		poseMsg.relative_pose = PoseToMsg( frameRelativePose );
		posePub.publish( poseMsg );
	}
}
	
} // end namespace fieldtrack
