#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include "extrinsics_array/ExtrinsicsCommon.h"
#include <argus_utils/geometry/PoseSE3.h>
#include <memory>

namespace argus
{

/*! \brief Wrapper around the tf2 transform lookup system. */
class ExtrinsicsInterface
{
public:

	typedef std::shared_ptr<ExtrinsicsInterface> Ptr;

	ExtrinsicsInterface( ros::NodeHandle& nh, ros::NodeHandle& ph );

	/*! \brief Attempt to convert a relative pose into the specified from/to
	 * frames. */
	PoseSE3 Convert( const std::string& fromIn,
	                 const std::string& toIn,
	                 const ros::Time& timeIn,
	                 const PoseSE3& poseIn,
	                 const std::string& fromOut, 
	                 const std::string& toOut );

	/*! \brief Look up the pose of 'from' relative to 'to' at an optional time. 
	 * Throws an ExtrinsicsException if the lookup fails. */
	PoseSE3 GetExtrinsics( const std::string& from, 
	                       const std::string& to,
	                       const ros::Time& time = ros::Time( 0 ) );

	/*! \brief Look up the displacement of 'from' from start to stop. Throws an
	 * ExtrinsicsException if the lookup fails. */
	PoseSE3 GetDisplacement( const std::string& frame,
	                         const ros::Time& start,
	                         const ros::Time& stop );

	PoseSE3 GetExtrinsics( const std::string& from,
	                       const ros::Time& fromTime,
	                       const std::string& to,
	                       const ros::Time& toTime );

private:

	std::shared_ptr<tf2_ros::Buffer> _tfBuffer;
	std::shared_ptr<tf2_ros::TransformListener> _tfListener;

};

}