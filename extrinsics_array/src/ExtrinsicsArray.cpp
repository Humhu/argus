#include "extrinsics_array/ExtrinsicsArray.h"
#include "argus_utils/GeometryUtils.h"
#include <boost/foreach.hpp>

using namespace argus_utils;

namespace extrinsics_array
{
	
ExtrinsicsArray::ExtrinsicsArray( const std::string& refName )
: referenceFrame( refName ) {}

ExtrinsicsArray::ExtrinsicsArray( const ExtrinsicsArrayInfo& info )
{
	for( unsigned int i = 0; i < info.extrinsics.size(); i++ )
	{
		AddMember( info.memberNames[i], MsgToPose( info.extrinsics[i] ) );
	}
	referenceFrame = info.frame_id;
}

void ExtrinsicsArray::AddMember( const std::string& memberName,
                                 const PoseSE3& pose )
{
	extrinsics[ memberName ] = pose;
}

const std::string& ExtrinsicsArray::GetReferenceFrame() const { return referenceFrame; }

bool ExtrinsicsArray::HasMember( const std::string& memberName ) const
{
	return extrinsics.count( memberName ) != 0;
}

const PoseSE3& ExtrinsicsArray::GetPose( const std::string& name ) const
{
	return extrinsics.at( name );
}
	
} // end namespace extrinsics_array
