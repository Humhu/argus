#include "fiducial_array/FiducialArray.h"
#include "fiducial_array/FiducialCommon.h"
#include "argus_utils/GeometryUtils.h"

namespace fiducial_array
{
	
FiducialArray::FiducialArray() {}

FiducialArray::~FiducialArray() {}

FiducialArray::FiducialArray( const FiducialArrayInfo& info )
: ExtrinsicsArray( info.extrinsics )
{
	for( unsigned int i = 0; i < info.extrinsics.memberNames.size(); i++ )
	{
		const std::string& fidName = info.extrinsics.memberNames[i];
		if( fiducials.count( fidName ) > 0 )
		{
			throw std::runtime_error( "Repeated fiducial: " + fidName );
		}
		
		fiducials[ fidName ] = Fiducial( info.fiducials[i] );
		argus_utils::PoseSE3 fidPose = argus_utils::MsgToPose( info.extrinsics.extrinsics[i] );
		arrayFiducials[ fidName ] = fiducials[ fidName ].Transform( fidPose );
	}
}

const Fiducial& FiducialArray::GetFiducialTransformed( const std::string& name ) const
{
	return arrayFiducials.at( name );
}

const Fiducial& FiducialArray::GetFiducial( const std::string& name ) const
{
	return fiducials.at( name );
}
	
}
