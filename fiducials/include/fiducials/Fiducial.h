#pragma once

#include "argus_utils/PoseSE3.h"
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
	Fiducial();
	
	/*! \brief Constructs from a fiducial info message. */
	Fiducial( const FiducialInfo& info );
	
	/*! \brief Returns a fiducial with transformation applied to the points. */
	Fiducial Transform( const argus::PoseSE3& pose ) const;
		
	/*! \brief Returns a corresponding info message. */
	FiducialInfo ToInfo() const;
	
};
	
} // end namespace fiducials
