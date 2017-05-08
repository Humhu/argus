#include "fieldtrack/KalmanChain.h"
#include "fieldtrack/CovarianceModels.h"
#include "argus_utils/random/MultivariateGaussian.hpp"

using namespace argus;

int main( int argc, char** argv )
{
	unsigned int state_dim = 3;
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


	FixedCovariance Qmodel, Rmodel;
	Qmodel.Initialize( Qguess );
	Rmodel.Initialize( Rguess );

	unsigned num_iters = 10;
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
		chain.AddLinearUpdate( C, y, Rmodel.GetCovOut() );
	}

	// Optimize
	unsigned num_opt_iters = 200;
	double alpha = 1E-1;
	for( unsigned int i = 0; i < num_opt_iters; ++i )
	{
		chain.Foreprop();
		Qmodel.Foreprop();
		Rmodel.Foreprop();
		std::cout << "Iter " << i << " Mean LL " << meanLL.GetValue() << std::endl;
		
		meanLL.Backprop(MatrixType::Identity(1,1));
		MatrixType dQD = Qmodel.GetLogDBackpropValue();
		MatrixType dQL = Qmodel.GetLBackpropValue();
		MatrixType dRD = Rmodel.GetLogDBackpropValue();
		MatrixType dRL = Rmodel.GetLBackpropValue();
		Eigen::Map<VectorType> dQDvec( dQD.data(), dQD.size(), 1 );
		Eigen::Map<VectorType> dQLvec( dQL.data(), dQL.size(), 1 );		
		Eigen::Map<VectorType> dRDvec( dRD.data(), dRD.size(), 1 );
		Eigen::Map<VectorType> dRLvec( dRL.data(), dRL.size(), 1 );

		std::cout << "Q:" << std::endl << Qmodel.GetCovOut().GetValue() << std::endl;
		std::cout << "R:" << std::endl << Rmodel.GetCovOut().GetValue() << std::endl;

		chain.Invalidate();
		Qmodel.Invalidate();
		Rmodel.Invalidate();

		std::cout << "dQlogD: " << dQDvec.transpose() << std::endl;
		std::cout << "dQL: " << dQLvec.transpose() << std::endl;
		std::cout << "dRlogD: " << dRDvec.transpose() << std::endl;
		std::cout << "dRL: " << dRLvec.transpose() << std::endl;

		Qmodel.SetLogD( Qmodel.GetLogD() + alpha * dQDvec );
		Qmodel.SetL( Qmodel.GetL() + alpha * dQLvec );		
		Rmodel.SetLogD( Rmodel.GetLogD() + alpha * dRDvec );
		Rmodel.SetL( Rmodel.GetL() + alpha * dRLvec );		
	}
}