#pragma once

#include "manycal/Registrations.h"
#include "graphopt/sclam_fiducial.h"
#include "camplex/FiducialCommon.h"

namespace argus
{
isam::FiducialFactor::Ptr create_fiducial_factor( const ros::Time& time,
                                                  const FiducialDetection& detection,
                                                  const MatrixType& cov,
                                                  CameraRegistration& camera,
                                                  FiducialRegistration& fiducial );
}