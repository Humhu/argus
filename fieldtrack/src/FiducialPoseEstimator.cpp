#include "fieldtrack/FiducialPoseEstimator.h"
#include "argus_utils/YamlUtils.h"
#include "argus_utils/MatrixUtils.h"
#include "argus_msgs/RelativePoseWithCovariance.h"

#include "fiducials/FiducialArray.h"
#include "fiducials/PoseEstimation.h"

#include <boost/foreach.hpp>

#include <unordered_map>

using namespace argus;
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
	
	std::vector<double> covData;
	if( !ph.getParam( "pose_covariance", covData ) )
	{
		covariance = PoseSE3::CovarianceMatrix::Identity();
	}
	else
	{
		if( !ParseMatrix( covData, covariance ) )
		{
			ROS_ERROR_STREAM( "Could not parse covariance matrix." );
			exit( -1 );
		}
	}
	
	detSub = nh.subscribe( "detections", 20, &FiducialPoseEstimator::DetectionsCallback, this );
	posePub = nh.advertise<argus_msgs::RelativePoseWithCovariance>( "relative_pose", 20 );
}

bool FiducialPoseEstimator::RetrieveCameraInfo( const std::string& cameraName )
{
	if( !extrinsicsManager.HasMember( cameraName ) )
	{
		// Force lookup of cameras, since any cameras we receive data from should be onboard
		if( !extrinsicsManager.ReadMemberInfo( cameraName, true ) ) { return false; }
	}
	return true;
}

bool FiducialPoseEstimator::RetrieveFiducialInfo( const std::string& fidName )
{
	if( !extrinsicsManager.HasMember( fidName ) )
	{
		// Don't force lookup of fiducials since there may be "rogue" fiducials
		if( !extrinsicsManager.ReadMemberInfo( fidName, false ) ) { return false; }
	}
		
	if( !fiducialManager.HasMember( fidName ) )
	{
		if( !fiducialManager.ReadMemberInfo( fidName ) ) { return false; }
		const PoseSE3& extrinsics = extrinsicsManager.GetInfo( fidName ).extrinsics;
		const Fiducial& intrinsics = fiducialManager.GetInfo( fidName );
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
	
	const extrinsics_array::ExtrinsicsInfo& cameraInfo = 
		extrinsicsManager.GetInfo( cameraName );
	
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
		const std::string& fiducialFrameName = extrinsicsManager.GetInfo( fidName ).referenceFrame;
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
		double sumMinDists = 0;
		BOOST_FOREACH( const argus_msgs::FiducialDetection& detection, detections )
		{
			const std::string& fiducialName = detection.name;
			if( !detection.normalized )
			{
				ROS_WARN_STREAM( "Received unnormalized detection of: " << fiducialName
				    << " from camera: " << cameraName );
				return;
			}
			sumMinDists += FindMinDistance( detection.points );
			const Fiducial& fiducial = transformedFiducials[ fiducialName ];
			const std::vector<cv::Point3f>& fidPoints = MsgToPoints( fiducial.points );
			fiducialFramePoints.insert( fiducialFramePoints.end(), fidPoints.begin(), fidPoints.end() );
			
			const std::vector<cv::Point2f>& imgPoints = MsgToPoints( detection.points );
			imageFramePoints.insert( imageFramePoints.end(), imgPoints.begin(), imgPoints.end() );
		}
		
		PoseSE3 cameraRelativePose = EstimateArrayPose( imageFramePoints, nullptr, fiducialFramePoints );
		
		PoseSE3 frameRelativePose = cameraInfo.extrinsics * cameraRelativePose;

		//EulerAngles euler = QuaternionToEuler( cameraRelativePose.GetQuaternion() );
		//EulerAngles finalEuler = QuaternionToEuler( frameRelativePose.GetQuaternion() );
		//ROS_INFO_STREAM( "Relative orientation of target to camera: " << euler );
		//ROS_INFO_STREAM( "Relative orientation of target to robot: " << finalEuler );

		argus_msgs::RelativePoseWithCovariance poseMsg;
		poseMsg.header.stamp = msg->header.stamp;
		poseMsg.header.frame_id = msg->header.frame_id;
		poseMsg.relative_pose.observer_name = cameraInfo.referenceFrame;
		poseMsg.relative_pose.observer_time = msg->header.stamp;
		poseMsg.relative_pose.target_name = fiducialFrameName;
		poseMsg.relative_pose.target_time = msg->header.stamp;
		poseMsg.relative_pose.relative_pose = PoseToMsg( frameRelativePose );
		double quality = sumMinDists / detections.size();
		SerializeSymmetricMatrix( covariance / quality, poseMsg.covariance );
		posePub.publish( poseMsg );
	}
}
	
} // end namespace fieldtrack
