#include "fieldtrack/FiducialPoseEstimator.h"
#include "argus_utils/YamlUtils.h"
#include "argus_msgs/RelativePose.h"

#include "extrinsics_array/ExtrinsicsArray.h"
#include "fiducials/FiducialArray.h"
#include "fiducials/PoseEstimation.h"

#include <boost/foreach.hpp>

#include <unordered_map>

using namespace argus_utils;
using namespace extrinsics_array;
using namespace fiducials;

namespace fieldtrack
{
	
FiducialPoseEstimator::FiducialPoseEstimator( ros::NodeHandle& nh, ros::NodeHandle& ph )
: lookupInterface(), fiducialManager( lookupInterface ), extrinsicsManager( lookupInterface )
{
	std::string lookupNamespace;
	ph.param<std::string>( "lookup_namespace", lookupNamespace, "/lookup" );
	lookupInterface.SetLookupNamespace( lookupNamespace );
	
	detSub = nh.subscribe( "detections", 20, &FiducialPoseEstimator::DetectionsCallback, this );
	posePub = nh.advertise<argus_msgs::RelativePose>( "relative_poses", 20 );
}

bool FiducialPoseEstimator::RetrieveCameraInfo( const std::string& cameraName )
{
	if( !extrinsicsManager.HasMember( cameraName ) )
	{
		// Force lookup of cameras, since any cameras we receive data from should be onboard
		if( !extrinsicsManager.ReadMemberInformation( cameraName, true ) ) { return false; }
	}
	return true;
}

bool FiducialPoseEstimator::RetrieveFiducialInfo( const std::string& fidName )
{
	if( !extrinsicsManager.HasMember( fidName ) )
	{
		// Don't force lookup of fiducials since there may be "rogue" fiducials
		if( !extrinsicsManager.ReadMemberInformation( fidName, false ) ) { return false; }
	}
		
	if( !fiducialManager.HasFiducial( fidName ) )
	{
		if( !fiducialManager.ReadFiducialInformation( fidName ) ) { return false; }
		const PoseSE3& extrinsics = extrinsicsManager.GetExtrinsics( fidName );
		const Fiducial& intrinsics = fiducialManager.GetIntrinsics( fidName );
		transformedFiducials[ fidName ] = intrinsics.Transform( extrinsics );
	}
	return true;
}

void FiducialPoseEstimator::DetectionsCallback( const argus_msgs::ImageFiducialDetections::ConstPtr& msg )
{

	// Map from reference frame names to fiducial detections
	typedef std::unordered_map< std::string, std::vector< argus_msgs::FiducialDetection > > 
	        DetectionMap;
	DetectionMap sorter;
	
	// 0. Retrieve the camera information
	std::string cameraName = msg->header.frame_id;
	if( !RetrieveCameraInfo( cameraName ) )
	{
		ROS_WARN_STREAM( "Could not retrieve information for camera: " << cameraName );
		return;
	}
	const std::string& cameraFrameName = extrinsicsManager.GetReferenceFrame( cameraName );
	const PoseSE3& cameraExtrinsics = extrinsicsManager.GetExtrinsics( cameraName );
	
	// 1. Group detected fiducials into arrays
	for( unsigned int i = 0; i < msg->detections.size(); i++ )
	{
		const argus_msgs::FiducialDetection& det = msg->detections[i];
		const std::string& fidName = det.name;
		if( !RetrieveFiducialInfo( fidName ) )
		{
			ROS_WARN_STREAM( "Could not retrieve information for fiducial: " << fidName );
			return;
		}
		const std::string& fiducialFrameName = extrinsicsManager.GetReferenceFrame( fidName );
		sorter[ fiducialFrameName ].push_back( det );
	}
	
	// 2. Convert detections into relative pose observations
	// TODO Spin off array trackers instead so they can initialize the PnP with state
	BOOST_FOREACH( const DetectionMap::value_type& item, sorter )
	{
		const std::string& fiducialFrameName = item.first;
		const std::vector<argus_msgs::FiducialDetection>& detections = item.second;
		
		// Collect all fiducial points from all detections in order
		std::vector<cv::Point3f> fiducialFramePoints;
		std::vector<cv::Point2f> imageFramePoints;
		BOOST_FOREACH( const argus_msgs::FiducialDetection& detection, detections )
		{
			const std::string& fiducialName = detection.name;
			if( !detection.normalized )
			{
				ROS_WARN_STREAM( "Received unnormalized detection of: " << fiducialName
				    << " from camera: " << cameraName );
				return;
			}
			
			const Fiducial& fiducial = transformedFiducials[ fiducialName ];
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
