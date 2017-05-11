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

	virtual void BindPredictModule( PredictModule& pred ) = 0;
	virtual void BindUpdateModule( UpdateModule& upd ) = 0;

	virtual VectorType GetParameters() const = 0;
	virtual void SetParameters( const VectorType& p ) = 0;
	virtual MatrixType GetBackpropValue() const = 0;
};

class FixedCovariance
: public CovarianceModel
{
public:

	typedef std::shared_ptr<FixedCovariance> Ptr;

	FixedCovariance();

	void Foreprop();
	void Invalidate();
	void UnregisterAll();

	void BindPredictModule( PredictModule& pred );
	void BindUpdateModule( UpdateModule& upd );

	VectorType GetParameters() const;
	void SetParameters( const VectorType& p );
	MatrixType GetBackpropValue() const;	

	void Initialize( const MatrixType& cov );
	void SetL( const VectorType& L );
	VectorType GetL() const;
	void SetLogD( const VectorType& D );
	VectorType GetLogD() const;

	const MatrixType& GetLBackpropValue() const;
	const MatrixType& GetLogDBackpropValue() const;

	OutputPort& GetCovOut();

private:

	ConstantModule _logD;
	ExponentialModule _expD;
	ConstantModule _lVec;
	ReshapeModule _lReshape;
	ReshapeModule _dReshape;
	XTCXModule _ldlt;
};

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

	void BindPredictModule( PredictModule& pred );
	void BindUpdateModule( UpdateModule& upd );

	VectorType GetParameters() const;
	void SetParameters( const VectorType& p );
	MatrixType GetBackpropValue() const;

private:

	struct PointEstimate
	{
		RepOuterProductModule _uOp;
		AdditionModule _estR;

		PointEstimate( UpdateModule& upd );
		OutputPort& GetOutput();
	};

	ConstantModule _defaultCov;

	unsigned int _windowSize;
	std::deque<PointEstimate> _pointRs; // Ordered oldest to newest
	std::deque<MeanModule> _estRs;
};

}