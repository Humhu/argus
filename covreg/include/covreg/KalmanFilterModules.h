#pragma once

#include <percepto/compo/AdditiveWrapper.hpp>
#include <percepto/compo/DifferenceWrapper.hpp>
#include <percepto/compo/TransformWrapper.hpp>
#include <percepto/compo/InverseWrapper.hpp>
#include <percepto/compo/ProductWrapper.hpp>
#include <percepto/compo/ScaleWrapper.hpp>

#include <percepto/optim/GaussianLogLikelihoodCost.hpp>

#include "covreg/PositiveDefiniteModules.h"
#include "covreg/CovarianceEstimator.h"
#include "argus_utils/utils/LinalgTypes.h"

namespace argus
{

// Represents the KF predict step for the estimate covariance
struct KalmanFilterPredictModule
{
	// State
	percepto::VectorTransformWrapper xminus;

	// Covariances
	percepto::TerminalSource<VectorType> qInput;
	CovarianceEstimator::ModuleType Q;
	percepto::ScaleWrapper<MatrixType> Qdt;
	percepto::TransformWrapper FSFT;
	percepto::AdditiveWrapper<MatrixType> Sminus;

	percepto::Source<VectorType>* xprev;
	percepto::Source<MatrixType>* Sprev;

	// Here Sprev refers to the previous estimate covariance
	// Need to set Q properties
	KalmanFilterPredictModule( percepto::Source<VectorType>* xprev,
		                       percepto::Source<MatrixType>* sprev,
	                           const CovarianceEstimator::ModuleType& q,
	                           double dt,
	                           const VectorType& input,
	                           const MatrixType& F );

	percepto::Source<VectorType>* GetTailState();
	percepto::Source<MatrixType>* GetTailCov();

	// NOTE We do not invalidate Sprev because we can't foreprop it
	void Invalidate();
	void Foreprop();

private:
	// Forbid copying
	KalmanFilterPredictModule( const KalmanFilterPredictModule& other );

	// Forbid assigning
	KalmanFilterPredictModule& operator=( const KalmanFilterPredictModule& other );
};

std::ostream& operator<<( std::ostream& os, 
                          const KalmanFilterPredictModule& module );

// Represents the KF update step for the estiamte covariance
struct KalmanFilterUpdateModule
{
	// State
	percepto::TerminalSource<VectorType> y;
	percepto::VectorTransformWrapper ypred;
	percepto::DifferenceWrapper<VectorType> innov;

	// Vinv * v
	percepto::VectorProductWrapper Vinvv;
	// H^T * Vinv * v
	percepto::VectorTransformWrapper HTVinvv;
	// S- * H^T * Vinv * v = x correction
	percepto::VectorProductWrapper xcorr;
	// x- + K*v = x+
	percepto::AdditiveWrapper<VectorType> xplus;

	// Here S- denotes S_previous, the estimate covariance before the update
	// Usually this is the covariance after a predict step, but sometimes
	// it can be after a simultaneous update
	// H * S- * H^T
	percepto::TransformWrapper HSHT;

	percepto::TerminalSource<VectorType> rInput;

	// The estimated R matrix
	CovarianceEstimator::ModuleType R;
	
	// The innovation covariance R + H * S- * H^T = V
	percepto::AdditiveWrapper<MatrixType> V;

	// The inverse innovation covariance V^-1
	percepto::InverseWrapper<percepto::EigLDL> Vinv;

	// The transformed inverse innovation covariance H^T * V^-1 * H
	percepto::TransformWrapper HTVinvH;

	// Product S- * H^T * V^-1 * H
	percepto::ProductWrapper SHTVinvH;

	// Product S- * H^T * V^-1 * H * S-
	percepto::ProductWrapper SHTVinvHS;

	// Sum S+ = S- - S- * H^T * V^-1 * H * S-
	percepto::DifferenceWrapper<MatrixType> Splus;

	// Log likelihood of innovation given V
	percepto::DynamicGaussianLogLikelihoodCost innovationLL;

	percepto::Source<VectorType>* xprev;
	percepto::Source<MatrixType>* Sprev;
	
	bool active;

	std::string sourceName;

	// Need to set R properties
	// Set source name, innovation
	KalmanFilterUpdateModule( percepto::Source<VectorType>* xprev,
	                          percepto::Source<MatrixType>* sPrev,
	                          const CovarianceEstimator::ModuleType& r,
	                          const VectorType& input,
	                          const VectorType& obs,
	                          const MatrixType& H );

	percepto::Source<VectorType>* GetTailState();
	percepto::Source<MatrixType>* GetTailCov();
	void Activate();

	void Invalidate();

	void Foreprop();

private:
	// Forbid copying
	KalmanFilterUpdateModule( const KalmanFilterUpdateModule& other );

	// Forbid assigning
	KalmanFilterUpdateModule& operator=( const KalmanFilterUpdateModule& other );

};

std::ostream& operator<<( std::ostream& os, const KalmanFilterUpdateModule& module );

}