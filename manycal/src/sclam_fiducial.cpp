#include "manycal/sclam_fiducial.h"

namespace isam
{
	
const char* FiducialIntrinsics::name() { return "FiducialIntrinsics"; }

FiducialIntrinsics::FiducialIntrinsics( const std::vector <isam::Point3d>& pts )
{
	if( pts.size() == 0 )
	{
		throw std::runtime_error( "Cannot create fiducial with zero points." );
	}
	
	points = VectorType( 3 * pts.size() );
	for( unsigned int i = 0; i < pts.size(); i++ )
	{
		points.block<3,1>( 3*i, 0 ) = PointType( pts[i].x(), pts[i].y(), pts[i].z() );
	}
}

FiducialIntrinsics::FiducialIntrinsics( const VectorType& v )
: points( v ) {}

FiducialIntrinsics FiducialIntrinsics::exmap( const Eigen::VectorXd& delta )
{
	if( delta.rows() != points.rows() )
	{
		throw std::runtime_error( "Cannot apply delta of size " 
		    + std::to_string( delta.rows() ) + " to fiducial of size " 
		    + std::to_string( points.rows() ) );
	}
	return FiducialIntrinsics( points + delta );
}

int FiducialIntrinsics::dim() const
{
	return points.rows();
}

void FiducialIntrinsics::set( const Eigen::VectorXd& v )
{
	if( v.rows() != points.rows() )
	{
		throw std::runtime_error( "Cannot set arg of size " 
		    + std::to_string( v.rows() ) + " to fiducial of size " 
		    + std::to_string( points.rows() ) );
	}
	points = v;
}

Eigen::VectorXb FiducialIntrinsics::is_angle() const
{
	return Eigen::VectorXb::Constant( points.rows()/3, 1, false );
}

FiducialIntrinsics::MatrixType FiducialIntrinsics::matrix() const
{
	Eigen::Map <const MatrixType> mMap( points.data(), 3, points.rows()/3 );
	return MatrixType( mMap );
}

Eigen::VectorXd FiducialIntrinsics::vector() const
{
	return points;
}

void FiducialIntrinsics::write( std::ostream& out ) const
{
	out << "(points: " << points.transpose() << ")";
}

std::ostream& operator<<( std::ostream& out, const FiducialIntrinsics& in )
{
	in.write( out );
	return out;
}

FiducialDetection::FiducialDetection( const Eigen::VectorXd& p )
{
	if( p.rows() % 2 != 0 )
	{
		throw std::runtime_error( "FiducialDetection: Uneven vector size." );
	}
	points = p;
}

int FiducialDetection::dim() const
{
	return points.rows();
}

Eigen::VectorXd FiducialDetection::vector() const
{
	return points;
}

void FiducialDetection::write( std::ostream& out ) const
{
	out << "(points: " << points.transpose() << ")";
}

std::ostream& operator<<( std::ostream& out, const FiducialDetection& det )
{
	det.write( out );
	return out;
}

FiducialDetection Predict( const FiducialIntrinsics& fiducial,
                           const MonocularIntrinsics& camera,
                           const PoseSE3& fiducialToCamera )
{
	
 	static PoseSE3::PoseType camToObj( 0, 0, 0, -0.5, 0.5, -0.5, 0.5 );
	
	Eigen::Transform <double, 2, Eigen::Affine> cameraMatrix( camera.K().block<2,2>(0,0) );
	PoseSE3::PoseType fidToCam = camToObj.Inverse() * fiducialToCamera.pose;
	PoseSE3::PoseType::Transform relPose( fidToCam.ToTransform() );
	
	Eigen::Matrix <double, 3, Eigen::Dynamic> relPoints = relPose * fiducial.matrix();
	Eigen::Matrix <double, 2, Eigen::Dynamic> imgPoints = (cameraMatrix * relPoints).colwise().hnormalized();
	
	return FiducialDetection( Eigen::Map <Eigen::VectorXd> ( imgPoints.data(), 2 * imgPoints.cols() ) );
}


}
