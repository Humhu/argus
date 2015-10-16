#include "extrinsics_array/ExtrinsicsArray.h"
#include "argus_utils/GeometryUtils.h"
#include <boost/foreach.hpp>

using namespace argus_utils;

namespace extrinsics_array
{
	
ExtrinsicsArray::ExtrinsicsArray() {}

ExtrinsicsArray::~ExtrinsicsArray() {}

ExtrinsicsArray::ExtrinsicsArray( const ExtrinsicsArrayInfo& info )
{
	std::vector< PoseSE3 > poses;
	poses.reserve( info.extrinsics.size() );
	for( unsigned int i = 0; i < info.extrinsics.size(); i++ )
	{
		poses.push_back( MsgToPose( info.extrinsics[i] ) );
	}
	referenceFrame = info.frame_id;
	Populate( info.memberNames, poses );
}

ExtrinsicsArray::ExtrinsicsArray( const std::string& refName,
                                  const std::vector< std::string >& names,
                                  const std::vector< argus_utils::PoseSE3 >& poses )
{
	referenceFrame = refName;
	Populate( names, poses );
}

void ExtrinsicsArray::Populate( const std::vector< std::string >& names,
                                const std::vector< argus_utils::PoseSE3 >& poses )
{
	if( names.size() != poses.size() )
	{
		throw std::runtime_error( "Need same number of names and poses." );
	}
	
	for( unsigned int i = 0; i < names.size(); i++ )
	{
		if( HasMember( names[i] ) )
		{
			throw std::runtime_error( "Duplicate member name " + names[i] );
		}
		extrinsics[ names[i] ] = poses[i];
	}
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
