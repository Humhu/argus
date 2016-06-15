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

namespace argus
{

#define POSDEF_OFFSET_SCALE (1E-9)

// TODO Allow more dynamic construction
struct PositiveDefiniteModule
{
	percepto::TerminalSource<VectorType> dInput;
	percepto::ConstantVectorRegressor lReg;
	percepto::ReLUNet dReg;
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
	                        unsigned int layerWidth )
	: lReg( matDim*(matDim-1)/2 ),
	  dReg( inputDim, matDim, numHiddenLayers, layerWidth,
      percepto::HingeActivation( 1.0, 1E-3 ) )
	{
		dReg.SetSource( &dInput );
		expModule.SetSource( &dReg.GetOutputSource() );
		psdModule.SetLSource( &lReg );
		psdModule.SetDSource( &expModule );
		pdModule.SetSource( &psdModule );
		pdModule.SetOffset( POSDEF_OFFSET_SCALE * 
		                    MatrixType::Identity( matDim, matDim ) );
	}

	// Copy assignment rewires all connections and should result in
	// shared parameters with the original
	PositiveDefiniteModule( const PositiveDefiniteModule& other )
	: lReg( other.lReg ), dReg( other.dReg ), pdModule( other.pdModule )
	{
		dReg.SetSource( &dInput );
		expModule.SetSource( &dReg.GetOutputSource() );
		psdModule.SetLSource( &lReg );
		psdModule.SetDSource( &expModule );
		pdModule.SetSource( &psdModule );
	}

	percepto::Source<MatrixType>* GetOutputSource()
	{
		return &pdModule;
	}

	void Invalidate()
	{
		dInput.Invalidate();
		lReg.Invalidate();
	}

	void Foreprop()
	{
		dInput.Foreprop();
		lReg.Foreprop();
	}

	MatrixType GetOutput() const { return pdModule.GetOutput(); }

	MatrixType Evaluate( const VectorType& in )
	{
		dInput.SetOutput( in );
		Invalidate();
		Foreprop();
		return pdModule.GetOutput();
	}

private:

	// Forbid assigning
	PositiveDefiniteModule& operator=( const PositiveDefiniteModule& other );
};

#define POSDEF_RELU_NUM_LAYERS (2)
#define POSDEF_RELU_WIDTH_SCALE (10) // Width = scale * max( input, output )

inline PositiveDefiniteModule
create_posdef_module( unsigned int inputDim, unsigned int matDim )
{
	return PositiveDefiniteModule( inputDim, 
	                               matDim, 
	                               POSDEF_RELU_NUM_LAYERS,
	                               POSDEF_RELU_WIDTH_SCALE * 
	                               std::max( inputDim, matDim ) );
}

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
	                           const MatrixType& F )
	: Q( q )
	{
		Q.dInput.SetOutput( input );
		FSFT.SetSource( Sprev );
		FSFT.SetTransform( F );
		Sminus.SetSourceA( Q.GetOutputSource() );
		Sminus.SetSourceB( &FSFT );
	}

	void SetRootSource( percepto::Source<MatrixType>* Sprev )
	{
		FSFT.SetSource( Sprev );
	}

	percepto::Source<MatrixType>* GetTailSource()
	{
		return &Sminus;
	}

	// NOTE We do not invalidate Sprev because we can't foreprop it
	void Invalidate() { Q.Invalidate(); }
	void Foreprop() { Q.Foreprop(); }

private:
	// Forbid copying
	KalmanFilterPredictModule( const KalmanFilterPredictModule& other );

	// Forbid assigning
	KalmanFilterPredictModule& operator=( const KalmanFilterPredictModule& other );
};

inline
std::ostream& operator<<( std::ostream& os, const KalmanFilterPredictModule& module )
{
	os << "Predict module: " << std::endl;
	os << "Q: " << std::endl << module.Q.GetOutput();
	return os;
}

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

	// Need to set R properties
	// Set source name, innovation
	KalmanFilterUpdateModule( percepto::Source<MatrixType>* Sprev,
	                          const PositiveDefiniteModule& r,
	                          const VectorType& input,
	                          const MatrixType& H,
	                          const VectorType& innovation )
	: R( r )
	{
		R.dInput.SetOutput( input );
		HSHT.SetSource( Sprev );
		HSHT.SetTransform( H );
		V.SetSourceA( &HSHT );
		V.SetSourceB( R.GetOutputSource() );
		Vinv.SetSource( &V );
		HTVinvH.SetSource( &Vinv );
		HTVinvH.SetTransform( H.transpose() );
		SHTVinvH.SetLeftSource( Sprev );
		SHTVinvH.SetRightSource( &HTVinvH );
		SHTVinvHS.SetLeftSource( &SHTVinvH );
		SHTVinvHS.SetRightSource( Sprev );
		Splus.SetPlusSource( Sprev );
		Splus.SetMinusSource( &SHTVinvHS );
		innovationLL.SetSource( &V );
		innovationLL.SetSample( innovation );
	}

	void SetRootSource( percepto::Source<MatrixType>* Sprev )
	{
		HSHT.SetSource( Sprev );
		SHTVinvH.SetLeftSource( Sprev );
		SHTVinvHS.SetRightSource( Sprev );
		Splus.SetPlusSource( Sprev );
	}

	percepto::Source<MatrixType>* GetTailSource()
	{
		return &Splus;
	}

	void Invalidate() { R.Invalidate(); }
	void Foreprop() { R.Foreprop(); }

private:
	// Forbid copying
	KalmanFilterUpdateModule( const KalmanFilterUpdateModule& other );

	// Forbid assigning
	KalmanFilterUpdateModule& operator=( const KalmanFilterUpdateModule& other );
};

inline
std::ostream& operator<<( std::ostream& os, const KalmanFilterUpdateModule& module )
{
	os << "Update module: " << std::endl;
	os << "R: " << std::endl << module.R.GetOutput() << std::endl;
	os << "V: " << std::endl << module.V.GetOutput() << std::endl;
	os << "inno: " << module.innovationLL.GetSample().transpose() << std::endl;
	os << "innoLL: " << module.innovationLL.GetOutput();
	return os;
}


}