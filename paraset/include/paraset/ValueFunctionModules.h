#pragma once

#include <percepto/neural/NetworkTypes.h>
#include <percepto/compo/ElementWrapper.hpp>

#include "argus_utils/utils/LinalgTypes.h"

namespace argus
{

template <typename InType, typename OutType>
class FunctionApproximator
{
public:

	typedef std::shared_ptr<FunctionApproximator> Ptr;
	typedef percepto::Source<InType> InputSourceType;
	typedef percepto::Source<OutType> OutputSourceType;

	FunctionApproximator() {}
	virtual ~FunctionApproximator() {}

	virtual void SetInputSource( InputSourceType* src ) = 0;
	virtual OutputSourceType& GetOutputSource() = 0;

	virtual void Foreprop() = 0;
	virtual void Invalidate() = 0;

	virtual percepto::Parameters::Ptr CreateParameters() = 0;

	virtual void InitializeOutput( const OutType& o ) = 0;

	virtual std::string Print() const = 0;
};

typedef FunctionApproximator<VectorType, double> ScalarFieldApproximator;

class PerceptronFunctionApproximator
: public ScalarFieldApproximator
{
public:

	typedef std::shared_ptr<PerceptronFunctionApproximator> Ptr;

	percepto::PerceptronNet network;
	percepto::ElementWrapper<VectorType> output;

	PerceptronFunctionApproximator( unsigned int inputDim,
	                                unsigned int outputDim,
	                                unsigned int numHiddenLayers,
	                                unsigned int layerWidth );

	PerceptronFunctionApproximator( const PerceptronFunctionApproximator& other );

	virtual void SetInputSource( InputSourceType* src );
	virtual OutputSourceType& GetOutputSource();

	virtual void Foreprop();
	virtual void Invalidate();

	virtual percepto::Parameters::Ptr CreateParameters();

	virtual void InitializeOutput( const double& o );

	virtual std::string Print() const;

};

}