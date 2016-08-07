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

	virtual void Foreprop();

	virtual void BackpropImplementation( const MatrixType& nextDodx );

private:

	percepto::PerceptronNet _net;
	percepto::L1NormalizationWrapper _pmf;

	SinkType _outputPort;

};

class PolicyModule
: public percepto::Source<MatrixType>
{
public:
	
	typedef percepto::Source<VectorType> InputSourceType;
	typedef percepto::Source<MatrixType> OutputSourceType;
	typedef percepto::Sink<MatrixType> SinkType;

	PolicyModule();
	virtual ~PolicyModule();

	virtual percepto::Parameters::Ptr CreateParameters() = 0;

	virtual void SetSource( InputSourceType* s ) = 0;

	virtual void Foreprop();

	virtual void BackpropImplementation( const MatrixType& nextDodx );

protected:

	SinkType _outputPort;

	void SetOutputModule( OutputSourceType* outputModule );

private:

	PolicyModule( const PolicyModule& other );
	PolicyModule& operator=( const PolicyModule& other );


};

class VarReLUGaussian
: public PolicyModule
{
public:

	typedef percepto::Source<VectorType> InputSourceType;
	typedef percepto::Source<MatrixType> OutputSourceType;
	typedef percepto::Sink<MatrixType> SinkType;

	typedef std::shared_ptr<VarReLUGaussian> Ptr;

	percepto::TerminalSource<VectorType> input;
	percepto::ReLUNet reg; // Regresses mean and variances

	percepto::SubvectorWrapper meanWrapper;
	percepto::SubvectorWrapper varWrapper;

	percepto::ConstantVectorRegressor lReg;
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

	virtual percepto::Parameters::Ptr CreateParameters();

	virtual void SetSource( InputSourceType* s );

	virtual void Invalidate();

	virtual void Foreprop();

	virtual void BackpropImplementation( const MatrixType& nextDodx );

private:

	percepto::Sink<VectorType> _inputPort;

};

}