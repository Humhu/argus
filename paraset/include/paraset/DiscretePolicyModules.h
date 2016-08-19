#pragma once

#include <percepto/compo/ModifiedCholeskyWrapper.hpp>
#include <percepto/compo/SubvectorWrapper.hpp>
#include <percepto/compo/NormalizationWrapper.hpp>
#include <percepto/neural/NetworkTypes.h>

#include "argus_utils/utils/LinalgTypes.h"

#include <memory>

namespace argus
{

class DiscretePolicyModule
{
public:

	typedef std::shared_ptr<DiscretePolicyModule> Ptr;

	typedef percepto::Source<VectorType> VectorSourceType;

	DiscretePolicyModule();
	virtual ~DiscretePolicyModule();

	virtual void SetInputSource( VectorSourceType* src ) = 0;
	virtual VectorSourceType& GetProbabilitySource() = 0;

	virtual void Foreprop() = 0;
	virtual void Invalidate() = 0;

	virtual percepto::Parameters::Ptr CreateParameters() = 0;

	virtual void InitializeProbabilities( const VectorType& probs ) = 0;

};

class NormalizedPerceptron
: public DiscretePolicyModule
{
public:

	typedef std::shared_ptr<NormalizedPerceptron> Ptr;
	
	percepto::PerceptronNet factors;
	percepto::L1NormalizationWrapper probabilities;

	NormalizedPerceptron( unsigned int inputDim,
	                      unsigned int outDim,
	                      unsigned int numHiddenLayers,
	                      unsigned int layerWidth );

	NormalizedPerceptron( const NormalizedPerceptron& other );

	virtual void SetInputSource( VectorSourceType* src );
	virtual VectorSourceType& GetProbabilitySource();

	virtual void Foreprop();
	virtual void Invalidate();

	virtual percepto::Parameters::Ptr CreateParameters();

	virtual void InitializeProbabilities( const VectorType& probs );

};

// TODO operator<<

}