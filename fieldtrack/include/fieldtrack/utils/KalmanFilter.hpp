#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>

namespace argus 
{

/*! \class KalmanFilter KalmanFilter.h
 * \brief A basic discrete-time Kalman filter. */
template < int StateDim = Eigen::Dynamic,
           int ControlDim = Eigen::Dynamic,
           int ObsDim = Eigen::Dynamic >
class KalmanFilter 
{
public:

	typedef Eigen::Matrix<double, StateDim, 1>          StateVector;
	typedef Eigen::Matrix<double, StateDim, StateDim>   StateCovariance;
	typedef Eigen::Matrix<double, StateDim, StateDim>   StateTransition;
	
	typedef Eigen::Matrix<double, ControlDim, 1>        ControlVector;
	typedef Eigen::Matrix<double, StateDim, ControlDim> ControlTransition;

	typedef Eigen::Matrix<double, ObsDim, 1>            ObservationVector;
	typedef Eigen::Matrix<double, ObsDim, StateDim>     ObservationMatrix;
	typedef Eigen::Matrix<double, ObsDim, ObsDim>       ObservationCovariance;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	KalmanFilter() {}

	StateTransition& TransMatrix() { return A; }
	const StateTransition& TransMatrix() const { return A; }
	StateCovariance& TransCovariance() { return Q; }
	const StateCovariance& TransCovariance() const { return Q; }
	
	ControlTransition& ControlMatrix() { return B; }
	const ControlTransition& ControlMatrix() const { return B; }
	
	ObservationMatrix& ObsMatrix() { return C; }
	const ObservationMatrix& ObsMatrix() const { return C; }
	ObservationCovariance& ObsCovariance() { return R; }
	const ObservationCovariance& ObsCovariance() const { return R; }
	
	StateVector& EstimateMean() { return x; }
	const StateVector& EstimateMean() const { return x; }
	StateCovariance& EstimateCovariance() { return S; }
	const StateCovariance& EstimateCovariance() const { return S; }
	
	/*! \brief Execute a predict step with no controls. */
	void Predict() { PredictControl( ControlVector(), Q ); }

	/*! \brief Execute a predict step with no controls and prescribed covariance. */
	void Predict( const StateCovariance& q ) { PredictControl( ControlVector(), q ); }

	/*! \brief Execute a predict step with controls. */
	void PredictControl( const ControlVector& u ) { PredictControl( u, Q ); }
	
	/*! \brief Execute a predict step with controls and prescribed covariance. */
	void PredictControl( const ControlVector& u, const StateCovariance& q )
	{
		if( A.size() == 0 )
		{
			throw std::runtime_error( "KalmanFilter: A is empty." );
		}

		if( u.size() == 0 || B.size() == 0 ) { x = A*x; }
		else { x = A*x + B*u; }

		if( q.size() == 0 ) 
		{ 
			if( Q.size() == 0 )
			{
				throw std::runtime_error( "KalmanFilter: Q is empty." );
			}
			S = A*S*A.transpose() + Q; 
		}
		else { S = A*S*A.transpose() + q; }
	}
	
	/*! \brief Execute a measurement update step. */
	void Update( const ObservationVector& y ) { Update( y, R ); }

	/*! \brief Execute a measurement update with a prescribed covariance. */
	void Update( const ObservationVector& y, const ObservationCovariance& r )
	{
		if( y.size() == 0 || C.size() == 0 )
		{
			throw std::runtime_error( "Kalman Filter: y or C are empty." );
		}

		ObservationVector v = y - C*x;
		ObservationCovariance V = C*S*C.transpose() + r;
		Eigen::LLT<ObservationCovariance> dec( V );
		ObservationMatrix K = dec.solve( C*S ).transpose();
		x = x + K*v;
		S = ( StateCovariance::Identity() - K*C )*S;
	}

protected:

	/*! \brief The current state estimate mean and covariance. */
	StateVector x;
	StateCovariance S;

	/*! \brief The state transition matrices. x evolves as x' = Ax + Bu */
	StateTransition A;
	ControlTransition B;

	/*! \brief The state transition covariance matrix. */
	StateCovariance Q;

	/*! \brief The state observation matrix. */
	ObservationMatrix C;

	/*! \brief The observation covariance matrix. */
	ObservationCovariance R;

};

} // end namespace argus
