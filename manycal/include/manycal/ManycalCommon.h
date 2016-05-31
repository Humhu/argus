#pragma once

#include <isam/Pose3d.h>
#include <isam/Point3d.h>

#include <ros/time.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "fiducials/Fiducial.h"
#include "manycal/sclam_fiducial.h"
#include "argus_utils/PoseSE3.h"
#include "argus_msgs/FiducialDetection.h"

#include <geometry_msgs/Point.h>

namespace manycal
{

/*! \brief Convert to and from the ISAM pose representation. */
isam::Pose3d PoseToIsam( const argus::PoseSE3& pose );
argus::PoseSE3 IsamToPose( const isam::Pose3d& is );

/*! \brief Convert to and from the ISAM point representation. */
isam::Point3d MsgToIsam( const geometry_msgs::Point& msg );
geometry_msgs::Point IsamToMsg( const isam::Point3d& is );

/*! \brief Convert to and from the ISAM detection representation. */
isam::FiducialDetection DetectionToIsam( const argus_msgs::FiducialDetection& detection );
argus_msgs::FiducialDetection IsamToDetection( const isam::FiducialDetection& detection );

/*! \brief Convert to and from the ISAM fiducial representation. */
isam::FiducialIntrinsics FiducialToIsam( const fiducials::Fiducial& fid );
fiducials::Fiducial IsamToFiducial( const isam::FiducialIntrinsics& fid );

}
