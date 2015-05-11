#ifndef _POSE_SE2_H_
#define _POSE_SE2_H_

#include <Eigen/Geometry>
#include <iostream>

#define PSE2_EXPONENT_DIM (3)
#define PSE2_TANGENT_DIM (3)
#define PSE2_COVARIANCE_DOF (6)

// TODO: Templatize to choose between float and double
namespace argus_common 
{

	class PoseSE3;
	
	class PoseSE2 
	{
	friend class PoseSE3;
	public:

		typedef double ScalarType;
		typedef Eigen::Vector3d Vector;
		typedef Eigen::Transform<double, 2, Eigen::Isometry> Transform;
		typedef Eigen::Matrix<double, PSE2_EXPONENT_DIM, PSE2_EXPONENT_DIM> Matrix;
		typedef Eigen::Rotation2D<double> Rotation;
		typedef Eigen::Translation<double, 2> Translation;
		typedef Translation::VectorType TranslationVector;

		// Probability definitions
		typedef Eigen::Matrix<double, PSE2_TANGENT_DIM, PSE2_TANGENT_DIM> CovarianceMatrix;
		typedef Eigen::Matrix<double, PSE2_COVARIANCE_DOF, 1> CovarianceVector;
		
		// Lie group definitions
		typedef Eigen::Matrix<double, PSE2_TANGENT_DIM, 1> TangentVector;
		typedef Eigen::Matrix<double, PSE2_TANGENT_DIM, PSE2_TANGENT_DIM> AdjointMatrix;
		
		PoseSE2();
		explicit PoseSE2( double x, double y, double theta );
		explicit PoseSE2( const Vector& vec );
		explicit PoseSE2( const Transform& trans );
		explicit PoseSE2( const Matrix& m );
		explicit PoseSE2( const Rotation& r, const Translation& t );
		explicit PoseSE2( const PoseSE3& se3 );

		Matrix ToMatrix() const;
		Transform ToTransform() const;
		Vector ToVector() const;
		PoseSE2 Inverse() const;

		PoseSE2::Translation GetTranslation() const;
		PoseSE2::Rotation GetRotation() const;
		
		PoseSE2::TangentVector Log( const PoseSE2& other ) const;
		PoseSE2 Exp( const PoseSE2::TangentVector& other ) const;
		PoseSE2::AdjointMatrix GetAdjoint() const;
		PoseSE2::TangentVector Adjoint( const TangentVector& other ) const;
		
		PoseSE2 operator+() const;
		PoseSE2 operator-() const;
		PoseSE2 operator*( const PoseSE2& other ) const;
		PoseSE2 operator/( const PoseSE2& other ) const;

		friend std::ostream& operator<<( std::ostream& os, const PoseSE2& se2 );

	protected:

		Translation trans;
		Rotation rot;

		virtual void Print( std::ostream& os ) const;

	};

	PoseSE2 operator+( const PoseSE2& se2, PoseSE2::Vector& vec );
	PoseSE2 operator+( PoseSE2::Vector& vec, const PoseSE2& se2 );
	PoseSE2 operator-( const PoseSE2& se2, PoseSE2::Vector& vec );
	PoseSE2 operator-( PoseSE2::Vector& vec, const PoseSE2& se2 );
	std::ostream& operator<<(std::ostream& os, const PoseSE2& se2);

	PoseSE2::CovarianceVector unroll_covariance( const PoseSE2::CovarianceMatrix& cov );
	PoseSE2::CovarianceMatrix rollup_covariance( const PoseSE2::CovarianceVector& vec );

}

#endif

