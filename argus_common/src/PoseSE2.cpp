#include "argus_common/PoseSE2.h"
#include "argus_common/PoseSE3.h"
#include <cmath>

namespace argus_common
{

	PoseSE2::PoseSE2() 
		: trans(0, 0), rot(0) 
	{}

	PoseSE2::PoseSE2(const double x, const double y, const double theta) 
		: trans(x, y), rot(theta) 
	{}

	PoseSE2::PoseSE2(const Vector &vec) 
		: trans(vec(0), vec(1)), rot(vec(2)) 
	{}

	PoseSE2::PoseSE2( const Matrix& m ) 
		: trans( m.block<2,1>(0,2) ), rot(0.0) 
	{
		rot.fromRotationMatrix( m.block<2,2>(0,0) );
	}
		
	// TODO Remove this because of inaccuracy!
	PoseSE2::PoseSE2(const Transform &t) 
		: trans(t.translation()), rot(0.0) 
	{
		rot.fromRotationMatrix(t.linear());
	}

	PoseSE2::PoseSE2(const Rotation& r, const Translation& t) 
		: trans(t), rot(r) 
	{}

	PoseSE2::PoseSE2(const PoseSE3& se3) 
		: trans(se3.GetTranslation().x(), se3.GetTranslation().y()), rot(0.0) 
	{
		// Approximate projection by taking angular norm * z component
		PoseSE3::Quaternion quat = se3.GetQuaternion();
		double alpha_2 = acos(quat.w());
		if(sin(alpha_2) != 0.0) 
		{
			rot = Rotation( 2.0*alpha_2*quat.z()/sin(alpha_2) );
		}
	}

	PoseSE2::TangentVector PoseSE2::Log( const PoseSE2& other ) const 
	{
		PoseSE2 diff = other/(*this);
		TranslationVector t = diff.ToMatrix().block<2,1>(0,2);
		
		double theta = diff.GetRotation().angle();
		SECoefficients coeffs( theta );

		double f;
		if( theta*theta < 1E-4 ) 
		{
			f = 1 - theta*theta/12 - theta*theta/720;
		} 
		else 
		{
			f = 0.5*coeffs.a/coeffs.b;
		}
		
		TangentVector v;
		v << f*t(0) + 0.5*theta*t(1),
				f*t(1) - 0.5*theta*t(0),
				theta;
		return v;
	}

	PoseSE2 PoseSE2::Exp( const PoseSE2::TangentVector& other ) const 
	{
		double theta = other(2);
		SECoefficients coeffs( theta );
		Rotation::Matrix2 V;
		V << coeffs.a, -coeffs.b*theta,
				coeffs.b*theta, coeffs.a;
		TranslationVector t = V*other.block<2,1>(0,0);

		PoseSE2 delta( t(0), t(1), other(2) );
		return delta*(*this);
			
	}

	PoseSE2::AdjointMatrix PoseSE2::GetAdjoint() const 
	{
		AdjointMatrix m = AdjointMatrix::Zero();
		m.block<2,2>(0,0) = rot.matrix();
		m(0,2) = trans.y();
		m(1,2) = -trans.x();
		m(2,2) = 1;
		return m;
	}

	PoseSE2::TangentVector PoseSE2::Adjoint( const TangentVector& other ) const 
	{
		AdjointMatrix m = GetAdjoint();
		return m*other;
	}
	
	PoseSE2::Translation PoseSE2::GetTranslation() const 
	{
		return trans;
	}

	PoseSE2::Rotation PoseSE2::GetRotation() const 
	{
		return rot;
	}
	
	PoseSE2::Matrix PoseSE2::ToMatrix() const 
	{
		return ToTransform().matrix();
	}
	
	PoseSE2::Transform PoseSE2::ToTransform() const 
	{
		Transform ret;
		ret = rot*trans;
		return ret;
	}

	PoseSE2::Vector PoseSE2::ToVector() const 
	{
		Vector ret;
		ret << trans.x(), trans.y(), rot.angle();
		return ret;
	}

	PoseSE2 PoseSE2::Inverse() const 
	{
		Translation tInv( rot.inverse()*(-trans.translation()) );
		Rotation rInv = rot.inverse();

		PoseSE2 ret(rInv, tInv);
		return ret;
	}

	PoseSE2 PoseSE2::operator+() const 
	{
		return *this;
	}
	
	PoseSE2 PoseSE2::operator-() const 
	{
		return Inverse();
	}
	
	PoseSE2 PoseSE2::operator*(const PoseSE2& other) const 
	{
		Rotation rRes = rot*other.rot;
		Translation tRes( rot*(other.trans.translation()) + trans.translation() );
		PoseSE2 ret( rRes, tRes );
		return ret;
	}

	PoseSE2 PoseSE2::operator/(const PoseSE2& other) const 
	{
		Transform oTrans = other.ToTransform().inverse();
		Transform tTrans = ToTransform();
		return PoseSE2( tTrans*oTrans );
	}

	void PoseSE2::Print(std::ostream& os) const 
	{
		Vector vec = ToVector();
		os << vec(0) << " " << vec(1) << " " << vec(2);
	}

	PoseSE2 operator+( const PoseSE2& se2, PoseSE2::Vector& vec ) 
	{
		PoseSE2::Vector se2Vec = se2.ToVector();
		PoseSE2::Vector u = se2Vec + vec;
		return PoseSE2( u );
	}
	
	PoseSE2 operator+( PoseSE2::Vector& vec, const PoseSE2& se2 ) 
	{
		PoseSE2::Vector se2Vec = se2.ToVector();
		PoseSE2::Vector u = vec + se2Vec;
		return PoseSE2( u );
	}

	PoseSE2 operator-( const PoseSE2& se2, PoseSE2::Vector& vec ) 
	{
		PoseSE2::Vector se2Vec = se2.ToVector();
		PoseSE2::Vector u = se2Vec - vec;
		return PoseSE2( u );
	}
	
	PoseSE2 operator-( PoseSE2::Vector& vec, const PoseSE2& se2 ) 
	{
		PoseSE2::Vector se2Vec = se2.ToVector();
		PoseSE2::Vector u = vec - se2Vec;
		return PoseSE2( u );
	}
	
	std::ostream& operator<<(std::ostream& os, const PoseSE2& se2) 
	{
		se2.Print(os);
		return os;
	}

	PoseSE2::CovarianceVector unroll_covariance( const PoseSE2::CovarianceMatrix& cov ) 
	{
		PoseSE2::CovarianceVector vec;
		vec << cov(0,0), cov(0,1), cov(0,2),
				cov(1,1), cov(1,2),
				cov(2,2);
		return vec;
	}

	PoseSE2::CovarianceMatrix rollup_covariance( const PoseSE2::CovarianceVector& vec ) 
	{
		PoseSE2::CovarianceMatrix cov;
		cov << vec(0), vec(1), vec(2),
				vec(1), vec(3), vec(4),
				vec(2), vec(4), vec(5);
		return cov;
	}
	
}
