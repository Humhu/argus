#pragma once

#include "fiducials/Fiducial.h"
#include "fiducials/FiducialInfo.h"

#include <opencv2/core.hpp>

#include <unordered_map>

namespace argus
{

/*! \brief Stores point representations for a fiducial array.*/
class FiducialArray 
{
public:
	
	typedef std::shared_ptr <FiducialArray> Ptr;
	
	/*! \brief Constructs an empty array for a specified reference frame ID */
	FiducialArray( const std::string& refName );
	
	/*! \brief Adds a member. Overwrites existing members. */
	void AddFiducial( const std::string& fidName, const Fiducial& fid,
	                  const PoseSE3& extrinsics );
	
	/*! \brief Return the reference frame ID. */
	const std::string& GetReferenceFrame() const;
	
	/*! \brief Returns the fiducial points in the array frame. */
	const Fiducial& GetFiducialTransformed( const std::string& fidName ) const;
	
	/*! \brief Returns the fiducial points in their local frame. */
	const Fiducial& GetFiducial( const std::string& fidName ) const;
	
protected:
	
	std::string referenceFrame;
	
	struct FiducialRegistration
	{
		Fiducial fiducial;
		Fiducial transformedFiducial;
	};
	
	std::unordered_map <std::string, FiducialRegistration> fiducialRegistry;
	
};

}
