#pragma once

#include <percepto/compo/LinearRegressor.hpp>
#include <percepto/compo/ConstantRegressor.hpp>
#include <percepto/compo/ExponentialWrapper.hpp>
#include <percepto/compo/OffsetWrapper.hpp>
#include <percepto/compo/ModifiedCholeskyWrapper.hpp>
#include <percepto/neural/NetworkTypes.h>

#include "argus_utils/utils/LinalgTypes.h"


namespace argus
{

class ContinuousPolicyModule
{
public:

	typedef std::shared_ptr<ContinuousPolicyModule> Ptr;

	typedef percepto::Source<VectorType> VectorSourceType;
	typedef percepto::Source<MatrixType> MatrixSourceType;

	ContinuousPolicyModule();
	virtual ~ContinuousPolicyModule();

	virtual void SetInputSource( VectorSourceType* src ) = 0;
	virtual VectorSourceType& GetMeanSource() = 0;
	virtual MatrixSourceType& GetInfoSource() = 0;

	virtual void Foreprop() = 0;
	virtual void Invalidate() = 0;

	virtual percepto::Parameters::Ptr CreateMeanParameters() = 0;
	virtual percepto::Parameters::Ptr CreateCovParameters() = 0;

	virtual void InitializeMean( const VectorType& u ) = 0;
	virtual void InitializeInformation( const MatrixType& n ) = 0;

	virtual std::string Print() const = 0;
};

std::ostream& operator<<( std::ostream& os, const ContinuousPolicyModule& m );

class LinearGaussian
: public ContinuousPolicyModule
{
public:

	typedef percepto::Source<VectorType> VectorSourceType;
	typedef percepto::Source<MatrixType> MatrixSourceType;

	typedef std::shared_ptr<LinearGaussian> Ptr;

	percepto::LinearRegressor mean;
	percepto::ConstantVectorRegressor correlations;
	percepto::ConstantVectorRegressor logVariances;

	percepto::ExponentialWrapper variances;
	percepto::ModifiedCholeskyWrapper psdModule;
	percepto::OffsetWrapper<MatrixType> information;

	LinearGaussian( unsigned int inputDim,
	                unsigned int matDim );

	LinearGaussian( const LinearGaussian& other );

	virtual void SetInputSource( VectorSourceType* src );
	virtual VectorSourceType& GetMeanSource();
	virtual MatrixSourceType& GetInfoSource();

	virtual void Foreprop();
	virtual void Invalidate();

	virtual percepto::Parameters::Ptr CreateMeanParameters();
	virtual percepto::Parameters::Ptr CreateCovParameters();

	virtual void InitializeMean( const VectorType& u );
	virtual void InitializeInformation( const MatrixType& n );

	virtual std::string Print() const;
};

class FixedVarianceGaussian
: public ContinuousPolicyModule
{
public:

	typedef percepto::Source<VectorType> VectorSourceType;
	typedef percepto::Source<MatrixType> MatrixSourceType;

	typedef std::shared_ptr<FixedVarianceGaussian> Ptr;

	percepto::PerceptronNet mean;
	percepto::ConstantVectorRegressor correlations;
	percepto::ConstantVectorRegressor logVariances;

	percepto::ExponentialWrapper variances;
	percepto::ModifiedCholeskyWrapper psdModule;
	percepto::OffsetWrapper<MatrixType> information;

	FixedVarianceGaussian( unsigned int inputDim,
	                       unsigned int matDim,
	                       unsigned int numHiddenLayers,
	                       unsigned int layerWidth );

	FixedVarianceGaussian( const FixedVarianceGaussian& other );

	virtual void SetInputSource( VectorSourceType* src );
	virtual VectorSourceType& GetMeanSource();
	virtual MatrixSourceType& GetInfoSource();

	virtual void Foreprop();
	virtual void Invalidate();

	virtual percepto::Parameters::Ptr CreateMeanParameters();
	virtual percepto::Parameters::Ptr CreateCovParameters();

	virtual void InitializeMean( const VectorType& u );
	virtual void InitializeInformation( const MatrixType& n );

	virtual std::string Print() const;

	percepto::Parameters::Ptr corrParams;
};

class VariableVarianceGaussian
: public ContinuousPolicyModule
{
public:

	typedef percepto::Source<VectorType> VectorSourceType;
	typedef percepto::Source<MatrixType> MatrixSourceType;

	typedef std::shared_ptr<VariableVarianceGaussian> Ptr;

	percepto::PerceptronNet mean;
	percepto::PerceptronNet correlations;
	percepto::PerceptronNet logVariances;

	percepto::ExponentialWrapper variances;
	percepto::ModifiedCholeskyWrapper psdModule;
	percepto::OffsetWrapper<MatrixType> information;

	VariableVarianceGaussian( unsigned int inputDim,
	                          unsigned int matDim,
	                          unsigned int numHiddenLayers,
	                          unsigned int layerWidth );

	VariableVarianceGaussian( const VariableVarianceGaussian& other );

	virtual void SetInputSource( VectorSourceType* src );
	virtual VectorSourceType& GetMeanSource();
	virtual MatrixSourceType& GetInfoSource();

	virtual void Foreprop();
	virtual void Invalidate();

	virtual percepto::Parameters::Ptr CreateMeanParameters();
	virtual percepto::Parameters::Ptr CreateCovParameters();

	virtual void InitializeMean( const VectorType& u );
	virtual void InitializeInformation( const MatrixType& n );

	virtual std::string Print() const;

	percepto::Parameters::Ptr corrParams;
};

}