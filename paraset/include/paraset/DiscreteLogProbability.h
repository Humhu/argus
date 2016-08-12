#pragma once

#include "argus_utils/utils/LinalgTypes.h"
#include "percepto/compo/Interfaces.hpp"

namespace argus
{

// Represents the log-probability of selecting a particular action
// from a discrete probability distribution
class DiscreteLogProbability
: percepto::Source<VectorType>
{
public:

	typedef percepto::Source<VectorType>

	DiscreteLogProbability();

	virtual void Foreprop();

private:

	Sink<VectorType>

};

}

