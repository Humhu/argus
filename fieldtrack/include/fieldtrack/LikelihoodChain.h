#pragma once

#include "fieldtrack/KalmanChain.h"
#include "fieldtrack/CovarianceModels.h"
#include "modprop/optim/optim.hpp"
#include "argus_utils/filter/FilterInfo.h"

#include <unordered_map>
#include <deque>

namespace argus
{
class LikelihoodChain
	: public boost::static_visitor<void>,
	public KalmanChain
{
public:

	typedef KalmanChain::PredictModulePtr PredictModulePtr;
	typedef KalmanChain::UpdateModulePtr UpdateModulePtr;
	typedef KalmanChain::ModulePtrPair ModulePtrPair;

	LikelihoodChain();
	
	// TODO Don't really need/want this anymore
	void SetDiscountFactor( double gamma );

	void Foreprop();
	void Backprop();
	void Invalidate();
	double GetMeanLL();

	void InitializeChain( const VectorType& x, const MatrixType& P );
	void ClearChain();
	void RegisterTransCov( const CovarianceModel::Ptr& model );
	void RegisterObsSource( const std::string& name,
	                        const CovarianceModel::Ptr& model );

	void ProcessInfo( const FilterInfo& info );
	void operator()( const PredictInfo& info );
	void operator()( const UpdateInfo& info );

private:

	typedef std::unordered_map<std::string, CovarianceModel::Ptr> SourceRegistry;
	SourceRegistry _obsModels;

	double _gamma;
	bool _initialized;

	std::deque<PredictInfo> _accPreds;

	CovarianceModel::Ptr _transCov;
	std::deque<ScalingModule> _scalers;
	std::deque<GaussianLikelihoodModule> _glls;
	MeanModule _meanLL;
	SinkModule _meanSink;

	void ProcessPreds();
};
}