#pragma once

#include "extrinsics_array/ExtrinsicsArray.h"
#include "fiducial_array/Fiducial.h"
#include "fiducial_array/FiducialInfo.h"
#include "fiducial_array/FiducialArrayInfo.h"

#include <opencv2/core.hpp>

#include <unordered_map>

namespace fiducial_array
{

/*! \brief Stores point representations for a fiducial array.*/
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
	
	/*! \brief Returns the fiducial points in the array frame. */
	const Fiducial& GetFiducialTransformed( const std::string& name ) const;
	
	/*! \brief Returns the fiducial points in their local frame. */
	const Fiducial& GetFiducial( const std::string& name ) const;
	
protected:
	
	/*! \brief Fiducials in local coordinates */
	std::unordered_map <std::string, Fiducial > fiducials;
	
	/*! \brief Fiducials in array coordinates */
	std::unordered_map <std::string, Fiducial > arrayFiducials;
	
};

}
