#include "fiducial_array/Fiducial.h"
#include "fiducial_array/FiducialCommon.h"

namespace fiducial_array
{
	
Fiducial::Fiducial() {}

Fiducial::Fiducial( const FiducialInfo& info )
: points ( info.points ) {}
	
Fiducial Fiducial::Transform( const argus_utils::PoseSE3& pose ) const
{
	Eigen::Matrix <double, 3, Eigen::Dynamic> p = MsgToMatrix( points );
	p = pose.ToTransform() * p;
	FiducialInfo info;
	info.points = MatrixToMsg( p );	
	return Fiducial( info );
}
	
} // end namespace fiducial_array
