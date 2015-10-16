#pragma once

#include "argus_utils/PoseSE3.h"
#include "extrinsics_array/ExtrinsicsArrayInfo.h"
#include <unordered_map>
#include <memory>

namespace extrinsics_array
{

/*! \brief Stores pose representations for a array of objects. */
class ExtrinsicsArray
{
public:
	
	typedef std::shared_ptr< ExtrinsicsArray > Ptr;
	
	/*! \brief Constructs an empty array. */
	ExtrinsicsArray();
	
	virtual ~ExtrinsicsArray();
	
	/*! \brief Constructs an array from an info message. */
	ExtrinsicsArray( const ExtrinsicsArrayInfo& info );
	
	/*! \brief Constructs an array from name pose pairs. */
	ExtrinsicsArray( const std::string& refName,
	                 const std::vector< std::string >& names,
	                 const std::vector< argus_utils::PoseSE3 >& poses );
	
	/*! \brief Return the reference frame name. */
	const std::string& GetReferenceFrame() const;
	
	/*! \brief Returns whether or not the array contains a member. */
	bool HasMember( const std::string& memberName ) const;
	
	/*! \brief Returns the pose in the array frame. */
	const argus_utils::PoseSE3& GetPose( const std::string& name ) const;
	
protected:
	
	/*! \brief The array reference frame name. */
	std::string referenceFrame;
	
	/*! \brief Fiducial points in array coordinates */
	std::unordered_map< std::string, argus_utils::PoseSE3 > extrinsics;
	
	/*! \brief Populates the array using name-pose pairs. */
	void Populate( const std::vector< std::string >& names,
	               const std::vector< argus_utils::PoseSE3 >& poses );
};

} // end namespace extrinsics_array
