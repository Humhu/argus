#pragma once

#include "extrinsics_array/ExtrinsicsArray.h"
#include "fiducial_array/FiducialInfo.h"
#include "fiducial_array/FiducialArrayInfo.h"

#include <opencv2/core.hpp>

#include <unordered_map>

namespace fiducial_array
{

/*! \brief Stores point representations for a fiducial array. */
class FiducialArray 
: public extrinsics_array::ExtrinsicsArray
{
public:
	
	typedef std::shared_ptr< FiducialArray > Ptr;
	
	/*! \brief Constructs an empty array. */
	FiducialArray();
	
	virtual ~FiducialArray();
	
	/*! \brief Constructs an array from a calibration message. */
	FiducialArray( const FiducialArrayInfo& info );
	
	/*! \brief Constructs an array from names, poses, and fiducial-frame points. */
	FiducialArray( const std::string& refName,
	               const std::vector< std::string >& names,
	               const std::vector< argus_utils::PoseSE3 >& poses,
	               const std::vector< std::vector< cv::Point3f > >& points );
	
	/*! \brief Returns the fiducial points in the array frame. */
	const std::vector< cv::Point3f >& GetFiducialPoints( const std::string& name ) const;
	
protected:
	
	/*! \brief Fiducial points in array coordinates */
	std::unordered_map< std::string, std::vector< cv::Point3f > > fiducialPoints;
	
	void Populate( const std::vector< std::string >& names,
	               const std::vector< argus_utils::PoseSE3 >& poses,
	               const std::vector< std::vector< cv::Point3f > >& points );
	
};

}
