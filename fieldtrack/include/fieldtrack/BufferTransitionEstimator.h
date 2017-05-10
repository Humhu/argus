#pragma once

#include "fieldtrack/KalmanChain.h"
#include "fieldtrack/CovarianceModels.h"

#include "argus_utils/filter/FilterInfo.h"

namespace argus
{
class BufferTransitionEstimator
	: public boost::static_visitor<void>
{
public:

	BufferTransitionEstimator();

	void SetStepSize( double alpha );
	void Initialize( const MatrixType& Q );

	void Step();
	const MatrixType& GetQ() const;

	void ProcessInfo( const VectorType& x, const VectorType& P,
	                  const FilterInfo& info );
	void operator()( const PredictInfo& info );
	void operator()( const UpdateInfo& info );

private:

	double _alpha;
	std::shared_ptr<KalmanChain> _chain;
	std::deque<ConstantModule> _Rs;
	std::shared_ptr<FixedCovariance> _transCov;
	std::shared_ptr<SinkModule> _meanLL;
};
}