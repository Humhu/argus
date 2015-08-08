#include "atags/AtagCommon.h"
#include "argus_utils/GeometryUtils.h"

using namespace argus_utils;

namespace atags 
{
	
	AprilTags::TagDetection msg_to_detection( const argus_msgs::TagDetection& msg )
	{
		AprilTags::TagDetection det;
		det.good = true;
		det.id = msg.id;
		det.hammingDistance = msg.hammingDistance;
		for( unsigned int i = 0; i < 4; i++ )
		{
			det.p[i].first = msg.corners[i].x;
			det.p[i].second = msg.corners[i].y;
		}
		det.cxy.first = msg.center.x;
		det.cxy.second = msg.center.y;
		
		return det;
	}
	
	/*! \brief Return the pose of a rectified tag detection. */
	argus_utils::PoseSE3 get_tag_pose( const argus_msgs::TagDetection& msg,
										double tagSize )
	{
		if( !msg.isNormalized )
		{
			throw std::runtime_error( "Called wrong get_tag_pose for unnormalized detection." );
		}
		return get_tag_pose( msg, tagSize, 1.0, 1.0, 0.0, 0.0 );
	}

	argus_utils::PoseSE3 get_tag_pose( const argus_msgs::TagDetection& msg, double tagSize,
										double fx, double fy, double px, double py )
	{
		PoseSE3 pose;
		Eigen::Vector3d translation;
		Eigen::Matrix3d rotation;
		
		AprilTags::TagDetection detection = msg_to_detection( msg );
		detection.getRelativeTranslationRotation( tagSize, fx, fy, px, py,
												  translation, rotation );
// 		static PoseSE3 postrotation( 0, 0, 0, M_PI/2, M_PI/2, 0 );		
		static PoseSE3 postrotation( PoseSE3::Translation( 0, 0, 0 ), 
									 EulerToQuaternion( EulerAngles( -M_PI/2, -M_PI/2, 0 ) ) );
		
		PoseSE3::Translation t( translation );
		PoseSE3::Quaternion q( rotation );
		PoseSE3 H_tag_cam( t, q );
		return H_tag_cam * postrotation;
		
	}
	
}
