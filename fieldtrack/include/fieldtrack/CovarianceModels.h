#pragma once

#include "modprop/compo/core.hpp"
#include "modprop/kalman/kalman.hpp"
#include "modprop/optim/optim.hpp"
#include <deque>

#include "argus_utils/filter/FilterInfo.h"

namespace argus
{
class CovarianceModel
{
public:

	typedef std::shared_ptr<CovarianceModel> Ptr;

	CovarianceModel();
	~CovarianceModel();

	virtual void Foreprop() = 0;
	virtual void Invalidate() = 0;
	virtual void UnregisterAll() = 0;

	virtual void BindPredictModule( PredictModule& pred,
	                                const PredictInfo& info ) = 0;
	virtual void BindUpdateModule( UpdateModule& upd,
	                               const UpdateInfo& info ) = 0;

	// NOTE Default implementations assume no parameters
	virtual VectorType GetParameters() const;
	virtual void SetParameters( const VectorType& p );
	virtual MatrixType GetBackpropValue() const;
};

class PassCovariance
	: public CovarianceModel
{
public:

	typedef std::shared_ptr<PassCovariance> Ptr;

	PassCovariance();

	void Foreprop();
	void Invalidate();
	void UnregisterAll();

	void BindPredictModule( PredictModule& pred,
	                        const PredictInfo& info );
	void BindUpdateModule( UpdateModule& upd,
	                       const UpdateInfo& info );

private:

	std::deque<ConstantModule> _covs;
};

class FixedCovariance
	: public CovarianceModel
{
public:

	typedef std::shared_ptr<FixedCovariance> Ptr;

	FixedCovariance();
	FixedCovariance( const FixedCovariance& other );

	void Foreprop();
	void Invalidate();
	void UnregisterAll();

	void BindPredictModule( PredictModule& pred,
	                        const PredictInfo& info );
	void BindUpdateModule( UpdateModule& upd,
	                       const UpdateInfo& info );

	VectorType GetParameters() const;
	void SetParameters( const VectorType& p );
	MatrixType GetBackpropValue() const;

	void EnableL( bool enable );
	void EnableD( bool enable );
	void Initialize( const MatrixType& cov );

	void SetL( const VectorType& L );
	VectorType GetL() const;
	void SetLogD( const VectorType& D );
	VectorType GetLogD() const;
	
	MatrixType GetValue() const;
	MatrixType GetLValue() const;
	VectorType GetDValue() const;

	MatrixType GetLBackpropValue() const;
	MatrixType GetLogDBackpropValue() const;

	OutputPort& GetCovOut();

private:

	bool _enableL;
	bool _enableD;

	ConstantModule _logD;
	ExponentialModule _expD;
	ConstantModule _lVec;
	ReshapeModule _lReshape;
	ReshapeModule _dReshape;
	XTCXModule _ldlt;

	void LinkPorts();
};

class TimeScaledCovariance
	: public FixedCovariance
{
public:

	typedef std::shared_ptr<TimeScaledCovariance> Ptr;

	TimeScaledCovariance();

	void UnregisterAll();

	void BindPredictModule( PredictModule& pred,
	                        const PredictInfo& info );
	void BindUpdateModule( UpdateModule& upd,
	                       const UpdateInfo& info );

private:

	std::deque<ScaleModule> _scalers;
};

// TODO Enable/disable diagonal output
class AdaptiveCovariance
	: public CovarianceModel
{
public:

	typedef std::shared_ptr<AdaptiveCovariance> Ptr;

	AdaptiveCovariance();

	void SetWindowSize( unsigned int s );
	void SetDefaultValue( const MatrixType& S );

	void Foreprop();
	void Invalidate();
	void UnregisterAll();

	void BindPredictModule( PredictModule& pred,
	                        const PredictInfo& info );
	// NOTE BindUpdateModule saves a pointer to upd which it uses
	// on the next call to instantiate the appropriate modules!
	void BindUpdateModule( UpdateModule& upd,
	                       const UpdateInfo& info );

private:

	struct PointEstimate
	{
		RepOuterProductModule _uOp;
		InnerXTCXModule _CPCT;
		AdditionModule _denseR;
		ReshapeModule _diagR;

		PointEstimate( UpdateModule& upd );
		OutputPort& GetOutput();
	};

	ConstantModule _defaultCov;

	// TODO HACK!
	UpdateModule* _lastUpdate;

	unsigned int _windowSize;
	std::deque<PointEstimate> _pointRs; // Ordered oldest to newest
	std::deque<MeanModule> _estRs;
};
}