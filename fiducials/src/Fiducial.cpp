#include "fiducials/Fiducial.h"
#include "fiducials/FiducialCommon.h"

namespace argus
{
	
Fiducial::Fiducial() {}

Fiducial::Fiducial( const fiducials::FiducialInfo& info )
: points( info.points ) {}

Fiducial Fiducial::Transform( const PoseSE3& pose ) const
{
	Eigen::Matrix <double, 3, Eigen::Dynamic> p = MsgToMatrix( points );
	p = pose.ToTransform() * p;
	fiducials::FiducialInfo info;
	info.points = MatrixToMsg( p );	
	return Fiducial( info );
}

fiducials::FiducialInfo Fiducial::ToInfo() const
{
	fiducials::FiducialInfo info;
	info.points = points;
	return info;
}

}
