#pragma once

#include <argus_utils/utils/LinalgTypes.h>

#include <yaml-cpp/yaml.h>

#include "covreg/PositiveDefiniteModules.h"
#include "covreg/CovarianceEstimatorInfo.h"

namespace argus
{

class CovarianceEstimator
{
public:

	const std::string sourceName;

	/*! \brief Construct an estimator from the given parameter message. */
	// CovarianceEstimator( const covreg::CovarianceEstimatorInfo& info );

	// typedef LinearPosDefModule ModuleType;
	typedef VarReLUPosDefModule ModuleType;
	// typedef ConstantPosDefModule ModuleType;

	// TODO Parse this from an info message of some sort
	CovarianceEstimator( const std::string& source, const YAML::Node& info );

	/*! \brief Default implementation of copy constructor is fine. Note that
	 * the copied estimator will share parameters. */
	//CovarianceEstimator( const CovarianceEstimator& other );

	unsigned int InputDim() const;
	unsigned int OutputDim() const;

	void ZeroParams();
	void RandomizeVarianceParams();
	void ZeroCorrelationParams();
	void SetVarianceOffsets( const VectorType& v );
	void EnableCorrelationLearning();

	// Returns covariance
	// TODO: Make this explicitly EvaluateCovariance
	MatrixType Evaluate( const VectorType& input );

	covreg::CovarianceEstimatorInfo GetParamsMsg() const;
	void SetParamsMsg( const covreg::CovarianceEstimatorInfo& msg );

	/*! \brief Outputs a parameter message. */
	// covreg::CovarianceEstimatorInfo GetInfoMessage() const;

	/*! \brief Used to plug in to other optimization pipelines. */
	const ModuleType& GetModule();
	percepto::Parameters::Ptr GetParamSet();

private:

	std::string _sourceName;
	unsigned int _inDim;
	unsigned int _outDim;

	percepto::TerminalSource<VectorType> _psdPort;
	ModuleType _psd;

	bool _covarianceOutput;
	bool _learnCorrelations;
	percepto::Parameters::Ptr _lParams;
	percepto::Parameters::Ptr _dParams;
	percepto::ParameterWrapper::Ptr _params;

};

}