#include "fiducials/Fiducial.h"
#include "fiducials/FiducialCommon.h"

namespace fiducials
{
	
Fiducial::Fiducial() {}

Fiducial::Fiducial( const FiducialInfo& info )
: points( info.points ) {}

Fiducial Fiducial::Transform( const argus::PoseSE3& pose ) const
{
	Eigen::Matrix <double, 3, Eigen::Dynamic> p = MsgToMatrix( points );
	p = pose.ToTransform() * p;
	FiducialInfo info;
	info.points = MatrixToMsg( p );	
	return Fiducial( info );
}

FiducialInfo Fiducial::ToInfo() const
{
	FiducialInfo info;
	info.points = points;
	return info;
}

}
