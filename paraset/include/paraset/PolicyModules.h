#pragma once

#include <percepto/compo/LinearRegressor.hpp>
#include <percepto/compo/ConstantRegressor.hpp>
#include <percepto/compo/ExponentialWrapper.hpp>
#include <percepto/compo/HingeExponentialWrapper.hpp>
#include <percepto/compo/OffsetWrapper.hpp>
#include <percepto/compo/ModifiedCholeskyWrapper.hpp>
#include <percepto/compo/SubvectorWrapper.hpp>
#include <percepto/compo/NormalizationWrapper.hpp>
#include <percepto/neural/NetworkTypes.h>

#include "argus_utils/utils/LinalgTypes.h"

#include <memory>

namespace argus
{

class NormalizedPerceptron
: public percepto::Source<VectorType>
{
public:

	typedef std::shared_ptr<NormalizedPerceptron> Ptr;

	typedef percepto::Source<VectorType> InputSourceType;
	typedef percepto::Source<VectorType> OutputSourceType;
	typedef percepto::Sink<VectorType> SinkType;

	NormalizedPerceptron( unsigned int inputDim,
	                      unsigned int outDim,
	                      unsigned int numHiddenLayers,
	                      unsigned int layerWidth );

	NormalizedPerceptron( const NormalizedPerceptron& other );

	percepto::Parameters::Ptr CreateParameters();

	void SetSource( InputSourceType* s );

	OutputSourceType& GetProbabilitySource();

	void SetOutputOffsets( const VectorType& probs );

	virtual void Foreprop();

	virtual void BackpropImplementation( const MatrixType& nextDodx );

private:

	percepto::PerceptronNet _net;
	percepto::L1NormalizationWrapper _pmf;

	SinkType _outputPort;

};

class VarReLUGaussian
{
public:

	typedef percepto::Source<VectorType> VectorSourceType;
	typedef percepto::Source<MatrixType> MatrixSourceType;
	typedef percepto::Sink<MatrixType> SinkType;

	typedef std::shared_ptr<VarReLUGaussian> Ptr;

	percepto::ReLUNet reg; // Regresses mean and variances
	percepto::ConstantVectorRegressor lReg;

	percepto::SubvectorWrapper meanWrapper;
	percepto::SubvectorWrapper varWrapper;

	percepto::ExponentialWrapper expModule;
	percepto::ModifiedCholeskyWrapper psdModule;
	percepto::OffsetWrapper<MatrixType> pdModule;

	VarReLUGaussian( unsigned int inputDim,
	                 unsigned int matDim,
	                 unsigned int numHiddenLayers,
	                 unsigned int layerWidth );

	// Copy assignment rewires all connections and should result in
	// shared parameters with the original
	// dReg.SetParameters
	// dReg.SetParameters
	VarReLUGaussian( const VarReLUGaussian& other );

	percepto::Parameters::Ptr CreateParameters();

	void Foreprop();
	void Invalidate();

	VectorSourceType& GetMeanSource();
	MatrixSourceType& GetInfoSource();

	void SetOutputOffsets( const VectorType& means,
	                       const VectorType& vars );

	friend std::ostream& operator<<( std::ostream& os, const VarReLUGaussian& vrlg );

};

std::ostream& operator<<( std::ostream& os, const VarReLUGaussian& vrlg );

}