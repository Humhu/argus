#pragma once

#include "argus_utils/PoseSE3.h"
#include "fiducials/FiducialCommon.h"
#include "fiducials/FiducialInfo.h"
#include "geometry_msgs/Point.h"

namespace fiducials
{

/*! \brief Stores point representations for a fiducial. */
class Fiducial 
{
public:
	
	/*! \brief This fiducials ordered points. */
	std::vector <geometry_msgs::Point> points;
	
	/*! \brief Constructs an empty fiducial. */
	Fiducial() {}
	
	/*! \brief Constructs from a fiducial info message. */
	Fiducial( const FiducialInfo& info )
	: points( info.points ) {}
	
	/*! \brief Returns a fiducial with transformation applied to the points. */
	Fiducial Transform( const argus_utils::PoseSE3& pose ) const
	{
		Eigen::Matrix <double, 3, Eigen::Dynamic> p = MsgToMatrix( points );
		p = pose.ToTransform() * p;
		FiducialInfo info;
		info.points = MatrixToMsg( p );	
		return Fiducial( info );
	}
	
	/*! \brief Returns a corresponding info message. */
	FiducialInfo ToInfo() const
	{
		FiducialInfo info;
		info.points = points;
		return info;
	}
	
};
	
} // end namespace fiducials
