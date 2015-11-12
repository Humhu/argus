#pragma once

#include <isam/Pose3d.h>
#include <isam/Point3d.h>

#include "argus_utils/PoseSE3.h"

#include <geometry_msgs/Point.h>

namespace manycal
{

/*! \brief Convert to and from the ISAM pose representation. */
isam::Pose3d PoseToIsam( const argus_utils::PoseSE3& pose );
argus_utils::PoseSE3 IsamToPose( const isam::Pose3d& is );

/*! \brief Convert to and from the ISAM point representation. */
isam::Point3d MsgToIsam( const geometry_msgs::Point& msg );
geometry_msgs::Point IsamToMsg( const isam::Point3d& is );

}
