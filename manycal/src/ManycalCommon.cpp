#include "manycal/ManycalCommon.h"

namespace manycal
{
	
isam::Pose3d PoseToIsam( const argus_utils::PoseSE3& pose )
{
	return isam::Pose3d( pose.ToTransform() );
}

argus_utils::PoseSE3 IsamToPose( const isam::Pose3d& is )
{
	return argus_utils::PoseSE3( is.wTo() );
}

isam::Point3d MsgToIsam( const geometry_msgs::Point& msg )
{
	return isam::Point3d( msg.x, msg.y, msg.z );
}

geometry_msgs::Point IsamToMsg( const isam::Point3d& is )
{
	geometry_msgs::Point msg;
	msg.x = is.x();
	msg.y = is.y();
	msg.z = is.z();
	return msg;
}
	
}
