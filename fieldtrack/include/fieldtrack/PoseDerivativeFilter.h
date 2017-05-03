#pragma once

#include "argus_utils/utils/LinalgTypes.h"
#include "argus_utils/filter/FilterInfo.h"
#include "argus_utils/random/MultivariateGaussian.hpp"

#include "fieldtrack/FieldtrackCommon.h"

#include <Eigen/Cholesky>
#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/variant.hpp>
#include <iostream>

namespace argus
{

/*! \brief A Kalman filter that tracks pose and N derivatives. */
class PoseDerivativeFilter
: public boost::static_visitor<UpdateInfo>
{
public:
	
	typedef boost::function <MatrixType (double)> TransMatFunc;
	
	PoseDerivativeFilter( unsigned int order = 1 );

	unsigned int PoseDim() const;
	unsigned int DerivsDim() const;
	unsigned int CovDim() const;

	PoseSE3& Pose();
	const PoseSE3& Pose() const;

	VectorType& Derivs();
	const VectorType& Derivs() const;

	Eigen::Block<MatrixType> PoseCov();
	Eigen::Block<const MatrixType> PoseCov() const;
	Eigen::Block<MatrixType> DerivsCov();
	Eigen::Block<const MatrixType> DerivsCov() const;

	MatrixType& FullCov();
	const MatrixType& FullCov() const;

	// TODO Get rid of this?
	/*! \brief Displace the pose mean by a world displacement (left multiply). */
	void WorldDisplace( const PoseSE3& d, const PoseSE3::CovarianceMatrix& Q );

	/*! \brief Perform a predict step where the derivatives are used
	 * to integrate the forward pose. */
	PredictInfo Predict( const MatrixType& Q, double dt );

	MatrixType ComputeKalmanGain( const MatrixType& C, const MatrixType& R,
	                              MatrixType& V );
	void UpdateCovariance( const MatrixType& K, const MatrixType& C, const MatrixType& R );

	UpdateInfo operator()( const PoseObservation& obs );
	UpdateInfo operator()( const PositionObservation& obs );
	UpdateInfo operator()( const OrientationObservation& obs );
	UpdateInfo operator()( const DerivObservation& obs );

private:

	unsigned int _order;
	PoseSE3 _pose;
	VectorType _derivs;
	MatrixType _cov;
	TransMatFunc _tfunc;
	
};

class ObservationLikelihoodVisitor
: public boost::static_visitor<double>
{
public:

	ObservationLikelihoodVisitor( const PoseDerivativeFilter& filter );

	double operator()( const PoseObservation& obs );
	double operator()( const PositionObservation& obs );
	double operator()( const OrientationObservation& obs );
	double operator()( const DerivObservation& obs );

private:

	const PoseDerivativeFilter& _filter;
};

}
