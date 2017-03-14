#pragma once

#include <isam/Pose3d.h>
#include <isam/Point3d.h>

#include <ros/time.h>
#include <boost/date_time/posix_time/posix_time_types.hpp>

#include "fiducials/FiducialCommon.h"

#include "manycal/sclam_fiducial.h"
#include "argus_utils/geometry/PoseSE3.h"

#include <geometry_msgs/Point.h>

namespace argus
{

struct ObjectCalibration
{
    std::string name;
    PoseSE3 extrinsics;
};

struct FiducialCalibration : public ObjectCalibration
{
	Fiducial intrinsics;
};

struct CameraCalibration : public ObjectCalibration
{
	// camplex::CameraCalibration intrinsics;
};
// TODO Message representations?

/*! \brief Convert to and from the ISAM pose representation. */
isam::Pose3d PoseToIsam( const PoseSE3& pose );
PoseSE3 IsamToPose( const isam::Pose3d& is );

/*! \brief Convert to and from the ISAM point representation. */
isam::Point3d PointToIsam( const Translation3Type& msg );
Translation3Type IsamToPoint( const isam::Point3d& is );

/*! \brief Convert to and from the ISAM detection representation. */
isam::FiducialDetection DetectionToIsam( const FiducialDetection& detection );
FiducialDetection IsamToDetection( const isam::FiducialDetection& detection );

/*! \brief Convert to and from the ISAM fiducial representation. */
isam::FiducialIntrinsics FiducialToIsam( const Fiducial& fid );
Fiducial IsamToFiducial( const isam::FiducialIntrinsics& fid );

}
