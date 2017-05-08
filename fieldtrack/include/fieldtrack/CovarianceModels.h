#pragma once

#include "modprop/compo/core.hpp"
#include "modprop/kalman/kalman.hpp"

namespace argus
{

class FixedCovariance
{
public:

	FixedCovariance();

	void Initialize( const MatrixType& cov );
	void Foreprop();
	void Invalidate();

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

}