#pragma once

#include <argus_utils/utils/LinalgTypes.h>

#include "covreg/ModuleDefinitions.h"
#include "covreg/CovarianceEstimatorInfo.h"

namespace argus
{

class CovarianceEstimator
{
public:

	const std::string sourceName;

	/*! \brief Construct an estimator from the given parameter message. */
	// CovarianceEstimator( const covreg::CovarianceEstimatorInfo& info );

	/*! \brief Construct an estimator and create new parameters. */
	CovarianceEstimator( const std::string& source,
	                     unsigned int featDim,
	                     unsigned int matDim, 
	                     unsigned int numHiddenLayers, 
	                     unsigned int layerWidth );

	/*! \brief Default implementation of copy constructor is fine. Note that
	 * the copied estimator will share parameters. */
	//CovarianceEstimator( const CovarianceEstimator& other );

	void RandomizeVarianceParams();
	void ZeroCorrelationParams();

	MatrixType Evaluate( const VectorType& input );

	/*! \brief Outputs a parameter message. */
	// covreg::CovarianceEstimatorInfo GetInfoMessage() const;

	/*! \brief Used to plug in to other optimization pipelines. */
	const PositiveDefiniteModule& GetModule();
	percepto::Parameters::Ptr GetParamSet();

private:

	PositiveDefiniteModule _psd;

	percepto::Parameters::Ptr _lParams;
	std::vector<percepto::Parameters::Ptr> _dParams;
	percepto::ParameterWrapper::Ptr _params;

};

}