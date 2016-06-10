#pragma once

#include <percepto/compo/ConstantRegressor.hpp>
#include <percepto/compo/ExponentialWrapper.hpp>
#include <percepto/compo/OffsetWrapper.hpp>
#include <percepto/compo/ModifiedCholeskyWrapper.hpp>
#include <percepto/neural/NetworkTypes.h>

#include <argus_utils/utils/LinalgTypes.h>

#include "covreg/CovarianceEstimatorInfo.h"

namespace argus
{

class InnovationClipOptimizer;

class CovarianceEstimator
{
public:

	typedef percepto::ConstantRegressor CorrRegressor;
	typedef percepto::ReLUNet VarBaseRegressor;
	typedef percepto::InputWrapper<VarBaseRegressor> VarBaseModule;
	typedef percepto::ExponentialWrapper<VarBaseModule> VarModule;
	typedef percepto::ModifiedCholeskyWrapper<CorrRegressor,
	                                          VarModule> PSDModule;
	typedef percepto::OffsetWrapper<PSDModule> PDModule;
	typedef percepto::InputChainWrapper<VarBaseModule, PDModule> PDRegressor;

	const std::string sourceName;

	/*! \brief Construct an estimator from the given parameter message. */
	CovarianceEstimator( const covreg::CovarianceEstimatorInfo& info );

	/*! \brief Construct an estimator with the specified parameters. */
	CovarianceEstimator( const std::string& source,
	                     unsigned int matDim, 
	                     unsigned int featDim,
	                     unsigned int numHiddenLayers, 
	                     unsigned int layerWidth );

	void SetParameters( const VectorType& params );
	VectorType GetParameters() const;

	void RandomizeVarianceParams();
	void ZeroCorrelationParams();

	MatrixType Evaluate( const VectorType& input );

	/*! \brief Outputs a parameter message. */
	covreg::CovarianceEstimatorInfo GetInfoMessage() const;

private:

	friend class InnovationClipOptimizer;

	CorrRegressor _cReg;
	VarBaseRegressor _vBaseReg;
	VarBaseModule _vBaseMod;
	VarModule _vMod;
	PSDModule _psdMod;
	PDModule _pdMod;
	PDRegressor _pdReg;
	percepto::ParametricWrapper _paramWrapper;

	

};

}