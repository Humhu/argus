#pragma once

#include <Eigen/Dense>
#include <boost/function.hpp>

namespace argus
{

// TODO Perhaps adopt std::function instead of boost
// NOTE Setting certain template args to Dynamic does not work, ie. StateDim
/*! \class ExtendedKalmanFilter ExtendedKalmanFilter.h
* \brief A basic discrete-time Kalman filter with compile-time sizes. */
template < int StateDim = Eigen::Dynamic,
           int ControlDim = Eigen::Dynamic,
           int ObsDim = Eigen::Dynamic >
class ExtendedKalmanFilter 
{
public:

	typedef Eigen::Matrix<double, StateDim, 1>          StateVector;
	typedef Eigen::Matrix<double, StateDim, StateDim>   StateCovariance;	
	typedef Eigen::Matrix<double, ControlDim, 1>        ControlVector;
	typedef Eigen::Matrix<double, StateDim, StateDim>   TransitionJacobian;

	typedef boost::function< StateVector
	                         (const StateVector&, const ControlVector&) >
	        TransitionFunction;
	typedef boost::function< TransitionJacobian
	                         (const StateVector&, const ControlVector&) > 
	        TransitionJacobianFunction;
	
	typedef Eigen::Matrix<double, ObsDim, 1>            ObservationVector;
	typedef Eigen::Matrix<double, ObsDim, ObsDim>       ObservationCovariance;
	typedef Eigen::Matrix<double, ObsDim, StateDim>     ObservationJacobian;
	
	typedef boost::function< ObservationVector (const StateVector&) >
	        ObservationFunction;
	typedef boost::function< ObservationJacobian (const StateVector&) > 
	        ObservationJacobianFunction;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	ExtendedKalmanFilter() {}

	TransitionFunction& TransFunction() { return f; }
	TransitionJacobianFunction& TransJacFunction() { return F; }
	StateCovariance& TransCovariance() { return Q; }
	
	ObservationFunction& ObsFunction() { return h; }
	ObservationJacobianFunction& ObsJacFunction() { return H; }
	ObservationCovariance& ObsCovariance() { return R; }
	
	StateVector& EstimateMean() { return x; }
	StateCovariance& EstimateCovariance() { return S; }
	
	/*! \brief Execute a predict step with no controls. */
	void Predict() { Predict( Q ); };

	/*! \brief Execute a predict step with no controls and prescribed covariance. */
	void Predict( const StateCovariance& q ) 
	{
		ControlVector u = ControlVector::Zeros();
		Predict( u, q );
	}

	/*! \brief Execute a predict step with controls. */
	void Predict( const ControlVector& u )
	{
		Predict( u, Q );
	}
	
	/*! \brief Execute a predict step with controls and prescribed covariance. */
	void Predict( const ControlVector& u, const StateCovariance& q )
	{
		TransitionJacobian J = F(x,u);
		x = f(x,u);
		S = J*S*J.transpose() + q;
	}
	
	/*! \brief Execute a measurement update step. */
	void Update( const ObservationVector& z ) { Update( z, R ); }

	/*! \brief Execute a measurement update with a prescribed covariance. */
	void Update( const ObservationVector& z, const ObservationCovariance& r )
	{
		ObservationJacobian J = H(x);
		ObservationVector yres = z - h(x);
		ObservationCovariance Sres = J*S*J.transpose() + r;
		Eigen::ColPivHouseholderQR<ObservationCovariance> dec( Sres.transpose() );
		ObservationJacobian K = dec.solve( J*S.transpose() ).transpose();
		x = x + K*yres;
		S = ( StateCovariance::Identity() - K*J )*S;
	}

protected:

	/*! \brief The current state estimate mean and covariance. */
	StateVector x;
	StateCovariance S;

	/*! \brief The state transition functions */
	TransitionFunction f;
	TransitionJacobianFunction F;

	/*! \brief The state transition covariance matrix. */
	StateCovariance Q;

	/*! \brief The state observation functions */
	ObservationFunction h;
	ObservationJacobianFunction H;

	/*! \brief The observation covariance matrix. */
	ObservationCovariance R;

};
	
}
