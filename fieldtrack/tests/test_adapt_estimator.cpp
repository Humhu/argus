#include "fieldtrack/KalmanChain.h"
#include "fieldtrack/CovarianceModels.h"
#include "argus_utils/random/MultivariateGaussian.hpp"
#include <boost/circular_buffer.hpp>

using namespace argus;

int main( int argc, char** argv )
{
	unsigned int state_dim = 4;
	unsigned int obs_dim = 3;

	MatrixType A = MatrixType::Identity( state_dim, state_dim ) +
	               1E-3 * MatrixType::Random( state_dim, state_dim );
	MatrixType C = MatrixType::Identity( obs_dim, state_dim );

	MatrixType Q = 1E-2 * MatrixType::Identity( state_dim, state_dim );
	MatrixType R = 1E-2 * MatrixType::Identity( obs_dim, obs_dim );
	MatrixType Qguess = 1E-2 * MatrixType::Identity( state_dim, state_dim );
	MatrixType Rguess = 1E-2 * MatrixType::Identity( obs_dim, obs_dim );

	MultivariateGaussian<> stateNoise( state_dim );
	MultivariateGaussian<> obsNoise( obs_dim );
	stateNoise.SetCovariance( Q );
	obsNoise.SetCovariance( R );

	MatrixType P0 = 1E-1 * MatrixType::Identity( state_dim, state_dim );
	MultivariateGaussian<> initDist( state_dim );
	initDist.SetCovariance( P0 );

	VectorType x0 = initDist.Sample();
	SinkModule meanLL;

	KalmanChain chain;
	chain.Initialize( x0, P0 );
	link_ports( chain.GetMeanLikelihood(), meanLL.GetInput() );

	unsigned num_iters = 10;

	FixedCovariance Qmodel, R0model;
	Qmodel.Initialize( Qguess );
	R0model.Initialize( Rguess );

	std::vector<OutputPort*> Routs;
	Routs.push_back( &R0model.GetCovOut() );
	std::deque<AdjustedInnovationCovariance> Rmodels( num_iters - 1 );
	BOOST_FOREACH( AdjustedInnovationCovariance & r, Rmodels )
	{
		r.SetOffset( 1E-2 * VectorType::Ones( obs_dim ) );
		r.SetC( C );
		Routs.push_back( &r.GetCovOut() );
	}

	unsigned int windowSize = 5;

	VectorType x_curr = x0;
	std::vector<VectorType> xs;
	for( unsigned int i = 0; i < num_iters; ++i )
	{
		// Predict
		x_curr = A * x_curr + stateNoise.Sample();
		chain.AddLinearPredict( A, Qmodel.GetCovOut() );
		xs.push_back( x_curr );

		// Update
		VectorType y = C * x_curr + obsNoise.Sample();

		std::vector<InputPort*> uIns;
		unsigned int numToAdd = std::min( windowSize, num_iters - i - 1 );
		for( unsigned int j = 0; j < numToAdd; ++j )
		{
			std::cout << "Adding model " << i + j << std::endl;
			uIns.push_back( &Rmodels[i + j].AddUIn() );
		}

		std::vector<InputPort*> xIns, PIns, vIns, SIns;
		if( i < num_iters - 1 )
		{
			std::cout << "Capturing Pin for " << i << std::endl;
			PIns.push_back( &Rmodels[i].GetPIn() );
		}
		chain.AddLinearUpdate( C, y, *Routs[i],
		                       xIns, PIns, vIns, SIns, uIns );
	}

	// // Optimize
	unsigned num_opt_iters = 200;
	double alpha = 1E-1;
	time_t start = clock();
	for( unsigned int i = 0; i < num_opt_iters; ++i )
	{
		chain.Foreprop();
		Qmodel.Foreprop();
		R0model.Foreprop();
		BOOST_FOREACH( AdjustedInnovationCovariance & r, Rmodels )
		{
			r.Foreprop();
		}
		std::cout << "Iter " << i << " Mean LL " << meanLL.GetValue() << std::endl;
		for( unsigned int j = 0; j < Rmodels.size(); ++j )
		{
			std::cout << "R: " << Rmodels[j].GetCovOut().GetValue() << std::endl;
		}

		meanLL.Backprop( MatrixType::Identity( 1, 1 ) );
		MatrixType dQD = Qmodel.GetLogDBackpropValue();
		MatrixType dQL = Qmodel.GetLBackpropValue();
		Eigen::Map<VectorType> dQDvec( dQD.data(), dQD.size(), 1 );
		Eigen::Map<VectorType> dQLvec( dQL.data(), dQL.size(), 1 );

		std::cout << "Q:" << std::endl << Qmodel.GetCovOut().GetValue() << std::endl;

		chain.Invalidate();
		Qmodel.Invalidate();
		BOOST_FOREACH( AdjustedInnovationCovariance & r, Rmodels )
		{
			r.Invalidate();
		}

		std::cout << "dQlogD: " << dQDvec.transpose() << std::endl;
		std::cout << "dQL: " << dQLvec.transpose() << std::endl;

		Qmodel.SetLogD( Qmodel.GetLogD() + alpha * dQDvec );
		Qmodel.SetL( Qmodel.GetL() + alpha * dQLvec );
	}
	time_t finish = clock();
	std::cout << "Took " << (finish - start) / (float) CLOCKS_PER_SEC << " seconds." << std::endl;
}