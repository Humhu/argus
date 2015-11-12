#pragma once

#include "manycal/slam3d.h"

namespace isam
{
	
/*! \brief Velocity for a 3D pose. */
class VelocitySE3
{
public:
	typedef argus_utils::PoseSE3::TangentVector VelocityType;

	static const int dim = 6;
	static const char* name() { return "VelocitySE3"; }
	
	VelocityType velocity;
	
	VelocitySE3()
	: velocity( VelocityType::Zero() ) {}
	
	VelocitySE3( const VelocityType& v )
	: velocity( v ) {}
	
	Vector6d vector() const
	{
		return Vector6d( velocity );
	}
	
	VelocitySE3 exmap( const Vector6d& delta ) const
	{
		return VelocitySE3( velocity + delta );
	}
	
	Eigen::VectorXb is_angle() const
	{
		return Eigen::VectorXb::Constant(6, 1, false );
	}
	
	void write( std::ostream& out ) const
	{
		out << "(" << velocity.transpose() << ")";
	}
	
	void set( const Vector6d& v )
	{
		velocity = v;
	}
	
};

std::ostream& operator<<( std::ostream& out, const VelocitySE3& vel )
{
	vel.write( out );
	return out;
}

typedef NodeT<VelocitySE3> VelocitySE3_Node;

class VelocitySE3_Prior : public FactorT<VelocitySE3>
{
public:
	
	VelocitySE3_Prior( VelocitySE3_Node* vel, const VelocitySE3& measure,
	                   const Noise& noise )
	: FactorT<VelocitySE3>( "VelocitySE3_Prior", 6, noise, measure ),
	_vel( vel )
	{
		_nodes.resize( 1 );
		_nodes[0] = _vel;
	}
	
	void initialize()
	{
		if( !_vel->initialized() )
		{
			_vel->init( _measure );
		}
	}
	
	Eigen::VectorXd basic_error( Selector s = ESTIMATE ) const
	{
		return _vel->value( s ).vector() - _measure.vector();
	}
	
	virtual Jacobian jacobian()
	{
		Eigen::VectorXd r = error( LINPOINT );
		Jacobian jac( r );
		jac.add_term( _nodes[0], Eigen::MatrixXd::Identity( 6, 6 ) );
		return jac;
	}
	
private:
	
	VelocitySE3_Node* _vel;
};

/*! \brief Factor that connects two velocities by a difference. */
class VelocitySE3_Factor : public FactorT<VelocitySE3>
{
public:

	VelocitySE3_Factor( VelocitySE3_Node* vel1, VelocitySE3_Node* vel2,
	                    const VelocitySE3& delta, const Noise& noise )
	: FactorT<VelocitySE3>( "VelocitySE3_Factor", 6, noise, delta ),
	_velocity1( vel1 ), _velocity2( vel2 )
	{
		_nodes.resize(2);
		_nodes[0] = _velocity1;
		_nodes[1] = _velocity2;
	}
	
	void initialize()
	{
		require( _velocity1->initialized() || _velocity2->initialized(),
		         "VelocitySE3_Factor requires at least one velocity be initialized." );
		if( !_velocity1->initialized() )
		{
			VelocitySE3 v1( _velocity2->value().velocity - _measure.velocity );
			_velocity1->init( v1 );
		}
		else if( !_velocity2->initialized() )
		{
			VelocitySE3 v2( _velocity1->value().velocity + _measure.velocity );
		}
	}
	
	Eigen::VectorXd basic_error( Selector s = ESTIMATE ) const
	{
		return _velocity2->value(s).velocity - _velocity1->value(s).velocity 
		    - _measure.velocity;
	}
	
	virtual Jacobian jacobian()
	{
		Eigen::VectorXd r = error( LINPOINT );
		Jacobian jac( r );
		jac.add_term( _nodes[0], -Eigen::MatrixXd::Identity( 6, 6 ) );
		jac.add_term( _nodes[1], Eigen::MatrixXd::Identity( 6, 6 ) );
		return jac;
	}

private:
	
	VelocitySE3_Node* _velocity1;
	VelocitySE3_Node* _velocity2;
	
};

/*! \brief Factor that connects two poses and a velocity with a timestep measurement. 
 * Here the error is between the observed velocity and the velocity node's value. */
class PoseSE3Vel_Factor : public FactorT<double>
{
public:

	PoseSE3Vel_Factor( PoseSE3_Node* pose1, PoseSE3_Node* pose2, VelocitySE3_Node* vel,
	                  double dt, const Noise& noise )
	: FactorT<double>( "PoseSE3Vel_Factor", 6, noise, dt),
	_pose1( pose1 ), _pose2( pose2 ), _velocity( vel )
	{
		if( dt <= 0 )
		{
			throw std::runtime_error( "Delta t must be positive." );
		}
		_nodes.resize( 3 );
		_nodes[0] = _pose1;
		_nodes[1] = _pose2;
		_nodes[2] = _velocity;
	}
	
	void initialize()
	{
		require( ((_pose1->initialized() || _pose2->initialized()) && _velocity->initialized() )
		         || (_pose1->initialized() && _pose2->initialized() ),
		         " PoseSE3Vel_Factor requires one pose and the velocity or both poses be intialized." );
		if( !_velocity->initialized() )
		{
			PoseSE3::PoseType delta = _pose1->value().pose.Inverse() * _pose2->value().pose;
			VelocitySE3::VelocityType vel = PoseSE3::PoseType::Log( delta ) / _measure;
			_velocity->init( VelocitySE3( vel ) );
		}
		else if( !_pose2->initialized() )
		{
			PoseSE3::PoseType delta = PoseSE3::PoseType::Exp( _velocity->value().velocity * _measure );
			PoseSE3::PoseType forward = _pose1->value().pose * delta;
			_pose2->init( PoseSE3( forward ) );
		}
		else if( !_pose1->initialized() )
		{
			PoseSE3::PoseType invDelta = PoseSE3::PoseType::Exp( -_velocity->value().velocity * _measure );
			PoseSE3::PoseType backward = _pose2->value().pose * invDelta;
			_pose1->init( PoseSE3( backward ) );
		}
	}
	
	Eigen::VectorXd basic_error( Selector s = ESTIMATE ) const
	{
		PoseSE3::PoseType forward = _pose1->value(s).pose * PoseSE3::PoseType::Exp( _velocity->value(s).velocity );
		PoseSE3::PoseType err = forward.Inverse() * _pose2->value(s).pose;
		return PoseSE3::PoseType::Log( err );
	}
	
private:
	
	PoseSE3_Node* _pose1;
	PoseSE3_Node* _pose2;
	VelocitySE3_Node* _velocity;
	
};

} // end namespace isam
