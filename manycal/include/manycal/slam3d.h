#pragma once

#include <argus_utils/PoseSE3.h>
#include <argus_utils/GeometryUtils.h>

#include "isam/Node.h"
#include "isam/Factor.h"
#include "isam/slam3d.h"

namespace isam
{

typedef Eigen::Matrix <double,7,1> Vector7d;
	
/*! \brief 3D pose. */
class PoseSE3
{
public:
	typedef argus_utils::PoseSE3 PoseType;
	
	static const int dim = 6;
	static const char* name() { return "PoseSE3"; }
	
	PoseType pose;
	
	PoseSE3() {}
	
	PoseSE3( const PoseType& p )
	: pose( p ) {}
	
	PoseSE3( double x, double y, double z, double qw, double qx, double qy, double qz )
	: pose( x, y, z, qw, qx, qy, qz ) {}
	
	Vector7d vector() const
	{
		PoseType::Translation position = pose.GetTranslation();
		Vector7d ret;
		PoseType::Quaternion quaternion = pose.GetQuaternion();
		ret << position.x(), position.y(), position.z(), quaternion.w(), quaternion.x(),
		    quaternion.y(), quaternion.z();
		return ret;
	}
	
	Eigen::VectorXb is_angle() const
	{
		return Eigen::VectorXb::Constant(7, 1, false);
	}
	
	PoseSE3 exmap( const Vector6d& delta ) const
	{
		return PoseSE3( pose * PoseType::Exp( delta ) );
	}
	
	void write( std::ostream& out ) const
	{
		out << "( " << vector().transpose() << " )";
	}
	
	void set( const Vector7d& v )
	{
		pose = PoseType( v(0), v(1), v(2), v(3), v(4), v(5), v(6) );
	}
	
};

std::ostream& operator<<( std::ostream& out, const PoseSE3& pose )
{
	pose.write( out );
	return out;
}

typedef NodeT<PoseSE3> PoseSE3_Node;

class PoseSE3_Prior : public FactorT<PoseSE3>
{
public:
	
	PoseSE3_Prior( PoseSE3_Node* pose, const PoseSE3& measure, const Noise& noise )
	: FactorT<PoseSE3>( "PoseSE3_Prior", 6, noise, measure ),
	_pose( pose )
	{
		_nodes.resize( 1 );
		_nodes[0] = _pose;
	}
	
	void initialize()
	{
		if( !_pose->initialized() )
		{
			_pose->init( _measure );
		}
	}
	
	Eigen::VectorXd basic_error( Selector s = ESTIMATE ) const
	{
		PoseSE3::PoseType delta = _pose->value(s).pose.Inverse() * _measure.pose;
		PoseSE3::PoseType::TangentVector err = 
		    PoseSE3::PoseType::Log( delta );
		return err;
	}
	
	// TODO Work out analytical jacobian
// 	virtual Jacobian jacobian()
// 	{
// 		Eigen::VectorXd r = error( LINPOINT );
// 		Jacobian jac( r );
// 		
// 		PoseSE3::PoseType err = _pose->value( LINPOINT ).pose.Inverse() * _measure.pose;
// 		
// 		Eigen::MatrixXd Hadj = err.GetAdjoint();
// 		Eigen::MatrixXd Hnum = numerical_jacobian();
// 		
// 		std::cout << "Hadj: " << std::endl << Hadj << std::endl;
// 		std::cout << "Hnum: " << std::endl << Hnum << std::endl;
// 		
// 		jac.add_term( _nodes[0], Hnum );
// 		return jac;
// 	}
	
private:
	
	PoseSE3_Node* _pose;
	
};

// TODO Figure out Jacobian
/*! \brief Factor that connects two poses by a displacement. */
class PoseSE3_Factor : public FactorT<PoseSE3>
{
public:

	PoseSE3_Factor( PoseSE3_Node* pose1, PoseSE3_Node* pose2, const PoseSE3& delta,
	                const Noise& noise )
	: FactorT<PoseSE3>( "PoseSE3_Factor", 6, noise, delta ),
	_pose1( pose1 ), _pose2( pose2 )
	{
		_nodes.resize(2);
		_nodes[0] = _pose1;
		_nodes[1] = _pose2;
	}
	
	void initialize()
	{
		require( _pose1->initialized() || _pose2->initialized(),
				 "PoseSE3_Factor requires at least one pose initialized." );
		if( !_pose1->initialized() )
		{
			PoseSE3::PoseType prevPose = _pose2->value().pose * _measure.pose.Inverse();
			_pose1->init( PoseSE3( prevPose ) );
		}
		else if( !_pose2->initialized() )
		{
			PoseSE3::PoseType nextPose = _pose1->value().pose * _measure.pose;
			_pose2->init( PoseSE3( nextPose ) );
		}
	}
	
	Eigen::VectorXd basic_error( Selector s = ESTIMATE ) const
	{
		PoseSE3::PoseType delta = _pose1->value(s).pose.Inverse() * _pose2->value(s).pose;
		return PoseSE3::PoseType::Log( delta );
	}
	
private:
	
	PoseSE3_Node* _pose1;
	PoseSE3_Node* _pose2;
	
};

template <typename P>
class Slam_Traits {};

/*! \brief Traits template that describes types for PoseSE3. */
template <>
class Slam_Traits<PoseSE3>
{
public:
	
	typedef PoseSE3 PoseType;
	typedef PoseSE3_Node NodeType;
	typedef PoseSE3_Prior PriorType;
	typedef PoseSE3_Factor EdgeType;
	
	/*! \brief Scales a displacement. */
	static PoseSE3 ScaleDisplacement( const PoseSE3& displacement, double scale )
	{
		PoseSE3::PoseType::TangentVector w = PoseSE3::PoseType::Log( displacement.pose );
		return PoseSE3( PoseSE3::PoseType::Exp( w * scale ) );
	}
	
};

}
