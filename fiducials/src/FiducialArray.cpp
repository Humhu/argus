#include "fiducials/FiducialArray.h"
#include "fiducials/FiducialCommon.h"
#include "argus_utils/geometry/GeometryUtils.h"

namespace argus
{
	
FiducialArray::FiducialArray( const std::string& refName ) 
: referenceFrame( refName ) {}

void FiducialArray::AddFiducial( const std::string& fidName, const Fiducial& fid,
                                 const PoseSE3& extrinsics )
{
	FiducialRegistration registration;
	registration.fiducial = fid;
	registration.transformedFiducial = fid.Transform( extrinsics );
	fiducialRegistry[ fidName ] = registration;
}

const std::string& FiducialArray::GetReferenceFrame() const
{
	return referenceFrame;
}

const Fiducial& FiducialArray::GetFiducialTransformed( const std::string& fidName ) const
{
	return fiducialRegistry.at( fidName ).transformedFiducial;
}

const Fiducial& FiducialArray::GetFiducial( const std::string& fidName ) const
{
	return fiducialRegistry.at( fidName ).fiducial;
}
	
}
