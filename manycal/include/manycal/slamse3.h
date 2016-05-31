#pragma once

#include <isam/Node.h>
#include <isam/Factor.h>
#include <Eigen/Dense>

#include <argus_utils/PoseSE3.h>
#include <argus_utils/GeometryUtils.h>

#include "manycal/slam_traits.h"

namespace isam
{

typedef Eigen::Matrix <double,6,1> Vector6d;
typedef Eigen::Matrix <double,7,1> Vector7d;

/*! \brief 3D pose. */
class PoseSE3
{
public:
	typedef argus::PoseSE3 PoseType;
	
	static const int dim = 6;
	static const char* name() { return "PoseSE3"; }
	
	PoseType pose;
	
	PoseSE3();
	
	PoseSE3( const PoseType& p );
	
	PoseSE3( double x, double y, double z, double qw, double qx, double qy, double qz );
	
	Vector7d vector() const;
	
	Eigen::VectorXb is_angle() const;
	
	PoseSE3 exmap( const Vector6d& delta ) const;
	
	void write( std::ostream& out ) const;
	
	void set( const Vector7d& v );
};

std::ostream& operator<<( std::ostream& out, const PoseSE3& pose );

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

/*! \brief Traits template that describes types for PoseSE3. */
template <>
class Slam_Traits<PoseSE3>
{
public:
	
	typedef PoseSE3 PoseType;
	typedef PoseSE3_Node NodeType;
	typedef PoseSE3_Prior PriorType;
	typedef PoseSE3_Factor EdgeType;
	typedef PoseType::PoseType::CovarianceMatrix CovarianceType;
	
	// TODO Use manifold Noise type from ManifoldKalmanFilter?
	/*! \brief Combines two displacements together. Currently assumes post-appended noise,
	 * i.e. disp = mean * noise. */
	static void ComposeDisplacement( const PoseSE3& a, const PoseSE3& b,
	                                 const CovarianceType& A, const CovarianceType& B,
	                                 PoseSE3& c, CovarianceType& C )
	{
		c = PoseSE3( a.pose * b.pose );
		C = PoseSE3::PoseType::Adjoint( b.pose ) * A + B;
	}

	/*! \brief Scales a displacement. */
	static PoseSE3 ScaleDisplacement( const PoseSE3& displacement, double scale )
	{
		PoseSE3::PoseType::TangentVector w = PoseSE3::PoseType::Log( displacement.pose );
		return PoseSE3( PoseSE3::PoseType::Exp( w * scale ) );
	}
	
};

}
