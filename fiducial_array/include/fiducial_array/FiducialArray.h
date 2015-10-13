#pragma once

#include "fiducial_array/FiducialArrayInfo.h"

#include <opencv2/core.hpp>

#include <unordered_map>

namespace fiducial_array
{

/*! \brief Stores point representations for a fiducial array. */
class FiducialArray
{
public:
	
	FiducialArray();
	
	/*! \brief Populates the array using a fiducial array calibration message. */
	void FromInfo( const FiducialArrayInfo& info );
	
	/*! \brief Returns the fiducial points in the array frame. */
	const std::vector< cv::Point3f >& GetFiducialPoints( const std::string& name ) const;
	
private:
	
	/*! \brief Fiducial points in array coordinates */
	std::unordered_map< std::string, std::vector< cv::Point3f > > fiducialPoints;
};

}
