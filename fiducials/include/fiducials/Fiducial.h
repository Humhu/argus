#pragma once

#include "argus_utils/geometry/PoseSE3.h"
#include "fiducials/FiducialInfo.h"
#include "geometry_msgs/Point.h"

namespace argus
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
	Fiducial( const fiducials::FiducialInfo& info );
	
	/*! \brief Returns a fiducial with transformation applied to the points. */
	Fiducial Transform( const PoseSE3& pose ) const;
		
	/*! \brief Returns a corresponding info message. */
	fiducials::FiducialInfo ToInfo() const;
	
};
	
} // end namespace argus
