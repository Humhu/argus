#pragma once

#include <percepto/PerceptoTypes.h>

#include <percepto/compo/ConstantRegressor.hpp>
#include <percepto/compo/ExponentialWrapper.hpp>
#include <percepto/compo/OffsetWrapper.hpp>
#include <percepto/compo/ModifiedCholeskyWrapper.hpp>

#include <percepto/compo/AdditiveWrapper.hpp>
#include <percepto/compo/TransformWrapper.hpp>
#include <percepto/compo/InputWrapper.hpp>

#include <percepto/neural/NetworkTypes.h>

#include <percepto/optim/ParameterL2Cost.hpp>
#include <percepto/optim/GaussianLogLikelihoodCost.hpp>
#include <percepto/optim/MeanPopulationCost.hpp>
#include <percepto/optim/StochasticPopulationCost.hpp>
#include <percepto/optim/OptimizerTypes.h>

#include <argus_utils/utils/LinalgTypes.h>
#include <argus_utils/filters/FilterInfo.h>

#include <memory>
#include <unordered_map>

namespace argus
{

/*! \brief ReLU-based matrix regressor. */
class MatrixRegressor
{
public:

	typedef percepto::ReLUNet VarianceBaseRegressor;
	typedef percepto::ConstantRegressor CorrelationRegressor;
	typedef percepto::ExponentialWrapper<VarianceBaseRegressor> 
	        VarianceRegressor;
	typedef percepto::ModifiedCholeskyWrapper<CorrelationRegressor,
	                                          VarianceRegressor>
	        PSDRegressor;
	typedef percepto::RegressorOffsetWrapper<PSDRegressor> PDRegressor;

	MatrixRegressor( unsigned int matDim, unsigned int featDim,
	                 unsigned int numHiddenLayers, unsigned int layerWidth );

	void SetParameters( const VectorType& params );
	VectorType GetParameters() const;

	MatrixType Evaluate( const VectorType& input );

	PDRegressor& Regressor() { return _pdReg; }

private:

	VarianceBaseRegressor _vBaseReg;
	VarianceRegressor _vReg;
	CorrelationRegressor _cReg;
	PSDRegressor _psdReg;
	PDRegressor _pdReg;

};

struct InnovationClipParameters
{
	unsigned int numClipsToKeep;
	unsigned int maxClipLength;
	double l2Weight;
	unsigned int batchSize;
	percepto::SimpleConvergenceCriteria optimizerCriteria;

	InnovationClipParameters()
	: numClipsToKeep( 500 ), maxClipLength( 10 ),
	l2Weight( 1E-6 ), batchSize( 30 ) {}
};


class InnovationClipOptimizer
{
public:

	typedef MatrixRegressor::PDRegressor RegressorType;

	InnovationClipOptimizer( MatrixRegressor& qReg,
	                         const InnovationClipParameters& params =
	                         InnovationClipParameters() );

	void AddObservationReg( MatrixRegressor& reg, const std::string& name );

	void AddPredict( const PredictInfo& info, const VectorType& input );

	void AddUpdate( const UpdateInfo& info, const VectorType& input,
	                const std::string& name );

private:

	// A regressor evaluated at some input
	// Used for clip Qs and R
	typedef percepto::InputWrapper<RegressorType> EstimateType;
	
	// A regressor estimate linearly transformed
	// Used for clip Qs
	typedef percepto::TransformWrapper<EstimateType> TransformedEstimate;

	// Sum of all transformed clip Qs
	// H_t * Q_t * H_t^T + H_t * F_t * Q_t-1 * F_t^T * H_t^T + ...
	typedef percepto::AdditiveSumWrapper<TransformedEstimate> SumTransQ;
	
	// Sum of R with all transformed clip Qs
	// R_t + ...
	typedef percepto::AdditiveWrapper<SumTransQ,EstimateType> SumTransQR;

	// Innovation covariance, equal to transformed previous estimate
	// covariance plus R plus all transformed clip Qs
	// Sum of H_t * F_t * ... * S_t-N-1^+ * ... F_t^T * H_t^T plus term from before
	typedef percepto::OffsetWrapper<SumTransQR> InnovationCov;

	// The log-likelihood of an innovation sample
	typedef percepto::GaussianLogLikelihoodCost<InnovationCov> InnoLL;

	// Stochastic population cost of innovation log likelihoods
	typedef percepto::StochasticPopulationCost<InnoLL, std::deque> 
	        StochasticInnoLL;

	// Above cost penalized with L2 on parameters
	typedef percepto::ParameterL2Cost<StochasticInnoLL> PenalizedSILL;

	RegressorType& _transReg;
	std::unordered_map <std::string, RegressorType&> _obsRegs;

	// Current clip accumulator
	unsigned int _maxClipLength;
	typedef std::pair<PredictInfo, VectorType> PredictData;
	std::deque<PredictData> _predictBuffer; // Newest in front

	// Subset of an episode ranging from after an update to the next update
	struct ClipData
	{
		typedef std::shared_ptr<ClipData> Ptr;

		EstimateType estR;
		std::vector<EstimateType> estQs;
		std::vector<TransformedEstimate> transQs;
		SumTransQ sumTransQs;
		SumTransQR sumTransQsR;
		std::shared_ptr<InnovationCov> estV;

		ClipData( RegressorType& qReg, RegressorType& rReg, 
		          const UpdateInfo& info,
		          const VectorType& in, std::deque<PredictData>& buff );
	};

	// Clip memories
	unsigned int _maxNumClips;
	std::deque<ClipData> _clips;

	// Optimization
	std::deque<InnoLL> _innoLLs;
	StochasticInnoLL _sill;
	PenalizedSILL _psill;
	percepto::AdamStepper _stepper;
	percepto::SimpleConvergence _convergence;
	percepto::AdamOptimizer _optimizer;

};

}