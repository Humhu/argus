#include "manycal/slamse3.h"

namespace isam
{

PoseSE3::PoseSE3() {}
	
PoseSE3::PoseSE3( const PoseType& p )
: pose( p ) {}

PoseSE3::PoseSE3( double x, double y, double z, double qw, double qx, double qy, double qz )
: pose( x, y, z, qw, qx, qy, qz ) {}

Vector7d PoseSE3::vector() const
{
	argus::Translation3Type position = pose.GetTranslation();
	Vector7d ret;
	argus::QuaternionType quaternion = pose.GetQuaternion();
	ret << position.x(), position.y(), position.z(), quaternion.w(), quaternion.x(),
	    quaternion.y(), quaternion.z();
	return ret;
}

Eigen::VectorXb PoseSE3::is_angle() const
{
	return Eigen::VectorXb::Constant(7, 1, false);
}

PoseSE3 PoseSE3::exmap( const Vector6d& delta ) const
{
	return PoseSE3( pose * PoseType::Exp( delta ) );
}

void PoseSE3::write( std::ostream& out ) const
{
	out << "( " << vector().transpose() << " )";
}

void PoseSE3::set( const Vector7d& v )
{
	pose = PoseType( v(0), v(1), v(2), v(3), v(4), v(5), v(6) );
}

std::ostream& operator<<( std::ostream& out, const PoseSE3& pose )
{
	pose.write( out );
	return out;
}

}
