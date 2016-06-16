#pragma once

#include <percepto/compo/ConstantRegressor.hpp>
#include <percepto/compo/ExponentialWrapper.hpp>
#include <percepto/compo/OffsetWrapper.hpp>
#include <percepto/compo/ModifiedCholeskyWrapper.hpp>
#include <percepto/neural/NetworkTypes.h>

#include <percepto/compo/AdditiveWrapper.hpp>
#include <percepto/compo/DifferenceWrapper.hpp>
#include <percepto/compo/TransformWrapper.hpp>
#include <percepto/compo/InverseWrapper.hpp>
#include <percepto/compo/ProductWrapper.hpp>

#include <percepto/optim/ParameterL2Cost.hpp>
#include <percepto/optim/GaussianLogLikelihoodCost.hpp>
#include <percepto/optim/MeanCost.hpp>
#include <percepto/optim/StochasticMeanCost.hpp>

#include "argus_utils/utils/LinalgTypes.h"

namespace argus
{

// TODO Allow more dynamic construction
struct PositiveDefiniteModule
{
	percepto::TerminalSource<VectorType> dInput;
	percepto::ConstantVectorRegressor lReg;
	// percepto::ReLUNet dReg;
	percepto::ConstantVectorRegressor dReg;
	percepto::ExponentialWrapper<VectorType> expModule;
	percepto::ModifiedCholeskyWrapper psdModule;
	percepto::OffsetWrapper<MatrixType> pdModule;

	// Need to:
	// dInput.SetOutput
	// lReg.SetParameters
	// dReg.SetParameters
	PositiveDefiniteModule( unsigned int inputDim, 
	                        unsigned int matDim,
	                        unsigned int numHiddenLayers,
	                        unsigned int layerWidth );

	// Copy assignment rewires all connections and should result in
	// shared parameters with the original
	PositiveDefiniteModule( const PositiveDefiniteModule& other );

	percepto::Source<MatrixType>* GetOutputSource();

	void Invalidate();

	void Foreprop();

	MatrixType GetOutput() const;

	MatrixType Evaluate( const VectorType& in );

private:

	// Forbid assigning
	PositiveDefiniteModule& operator=( const PositiveDefiniteModule& other );
};

// Represents the KF predict step for the estimate covariance
struct KalmanFilterPredictModule
{
	PositiveDefiniteModule Q;

	percepto::TransformWrapper FSFT;

	percepto::AdditiveWrapper<MatrixType> Sminus;

	// Here Sprev refers to the previous estimate covariance
	// Need to set Q properties
	KalmanFilterPredictModule( percepto::Source<MatrixType>* Sprev,
	                           const PositiveDefiniteModule& q,
	                           const VectorType& input,
	                           const MatrixType& F );

	void SetRootSource( percepto::Source<MatrixType>* Sprev );

	percepto::Source<MatrixType>* GetTailSource();

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
	// Here S- denotes S_previous, the estimate covariance before the update
	// Usually this is the covariance after a predict step, but sometimes
	// it can be after a simultaneous update
	// H * S- * H^T
	percepto::TransformWrapper HSHT;

	// The estimated R matrix
	PositiveDefiniteModule R;
	
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
	percepto::GaussianLogLikelihoodCost innovationLL;

	bool active;

	percepto::Source<MatrixType>* Sprev;

	// Need to set R properties
	// Set source name, innovation
	KalmanFilterUpdateModule( percepto::Source<MatrixType>* sPrev,
	                          const PositiveDefiniteModule& r,
	                          const VectorType& input,
	                          const MatrixType& H,
	                          const VectorType& innovation );

	percepto::Source<MatrixType>* GetTailSource();

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