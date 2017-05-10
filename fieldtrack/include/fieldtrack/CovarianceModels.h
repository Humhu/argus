#pragma once

#include "modprop/compo/core.hpp"
#include "modprop/kalman/kalman.hpp"
#include "modprop/optim/optim.hpp"
#include <deque>

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

// NOTE This uses the raw residuals, which makes the optimization a bit
// unstable for longer chains...
class AdjustedInnovationCovariance
{
public:

	AdjustedInnovationCovariance();

	void Foreprop();
	void Invalidate();
	void SetC( const MatrixType& C );

	void SetOffset( const VectorType& u );

	InputPort& AddUIn();
	InputPort& GetPIn();
	OutputPort& GetCovOut();

private:

	ConstantModule _offU;
	RepOuterProductModule _opU;
	std::deque<RepOuterProductModule> _rawUs;
	MeanModule _meanU;
	InnerXTCXModule _HPHT;
	AdditionModule _estR;
};

}