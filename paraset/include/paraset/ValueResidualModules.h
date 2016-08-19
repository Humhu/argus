#pragma once

#include "paraset/ValueFunctionModules.h"
#include "percepto/compo/ScaleWrapper.hpp"
#include "percepto/compo/OffsetWrapper.hpp"
#include "percepto/optim/SquaredLoss.hpp"
#include "percepto/compo/DifferenceWrapper.hpp"

namespace argus
{

struct BellmanResidualModule
{
	BellmanResidualModule( ScalarFieldApproximator::Ptr estValueModule,
	                       const VectorType& prevIn,
	                       ScalarFieldApproximator::Ptr nextValueModule,
	                       const VectorType& nextIn,
	                       double reward,
	                       double gamma );

	percepto::TerminalSource<VectorType> input;
	ScalarFieldApproximator::Ptr estValue;

	percepto::TerminalSource<VectorType> nextInput;
	ScalarFieldApproximator::Ptr nextValue;
	percepto::ScaleWrapper<double> discountedNextValue;
	percepto::OffsetWrapper<double> targetValue;
	
	percepto::DifferenceWrapper<double> residual;
	percepto::SquaredLoss<double> loss;
};

}