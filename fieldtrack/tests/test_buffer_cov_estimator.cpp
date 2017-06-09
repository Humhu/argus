#include "fieldtrack/LikelihoodChain.h"
#include "fieldtrack/CovarianceModels.h"

#include "argus_utils/filter/KalmanFilter.h"
#include "argus_utils/random/MultivariateGaussian.hpp"
#include "argus_utils/utils/MathUtils.h"

using namespace argus;

int main( int argc, char** argv )
{
	unsigned int state_dim = 2;
	unsigned int system_order = 2;
	unsigned int obs_dim = 3;
	unsigned int full_dim = state_dim * (system_order + 1);
	double dt = 0.1;

	MatrixType A = IntegralMatrix<double>( dt, state_dim, system_order );
	MatrixType C = MatrixType::Identity( obs_dim, full_dim );

	MatrixType Q = 1E-2 * MatrixType::Identity( full_dim, full_dim );
	MatrixType R = 1E-1 * MatrixType::Identity( obs_dim, obs_dim );
	MatrixType Qguess = 1E-1 * MatrixType::Identity( full_dim, full_dim );
	MatrixType Rguess = MatrixType::Identity( obs_dim, obs_dim );

	MultivariateGaussian<> stateNoise( full_dim );
	MultivariateGaussian<> obsNoise( obs_dim );
	stateNoise.SetCovariance( Q * dt );
	obsNoise.SetCovariance( R );

	MatrixType P0 = 1E-1 * MatrixType::Identity( full_dim, full_dim );
	MultivariateGaussian<> initDist( full_dim );
	initDist.SetCovariance( P0 );

	VectorType x0 = initDist.Sample();

	TimeScaledCovariance::Ptr Qmodel = std::make_shared<TimeScaledCovariance>();
	Qmodel->Initialize( Qguess );
	Qmodel->EnableL( false );
	
	// FixedCovariance::Ptr Rmodel = std::make_shared<FixedCovariance>();
	// Rmodel->Initialize( Rguess );
	// AdaptiveCovariance::Ptr Rmodel = std::make_shared<AdaptiveCovariance>();
	// Rmodel->SetWindowSize( 10 );
	// Rmodel->SetDefaultValue( Rguess );
	PassCovariance::Ptr Rmodel = std::make_shared<PassCovariance>();

	LikelihoodChain chain;
	chain.RegisterTransCov( Qmodel );
	chain.RegisterObsSource( "obs", Rmodel );
	SinkModule meanLL;
	link_ports( chain.GetMeanLL(), meanLL.GetInput() );

	unsigned int numOuterIters = 100;
	time_t start = clock();
	for( unsigned int n = 0; n < numOuterIters; n++ )
	{
		chain.ClearChain();

		KalmanFilter filter;
		filter.SetTransitionMatrix( A );
		filter.SetTransitionCovariance( Qguess );
		filter.SetObservationMatrix( C );
		filter.SetObservationCovariance( Rguess );
		filter.Initialize( x0, P0 );

		unsigned num_iters = 100;
		VectorType x_curr = x0;
		for( unsigned int i = 0; i < num_iters; ++i )
		{
			x_curr = A * x_curr + stateNoise.Sample();
			VectorType y = C * x_curr + obsNoise.Sample();
			
			// Predict
			PredictInfo predInfo = filter.Predict();
			predInfo.step_dt = dt;
			UpdateInfo upInfo = filter.Update( y );
			upInfo.frameId = "obs";

			// Update
			chain.ProcessInfo( predInfo );
			chain.ProcessInfo( upInfo );
		}

		// Optimize
		unsigned num_opt_iters = 10;
		double alpha = 100;
		double maxNorm = 0.1;
		for( unsigned int i = 0; i < num_opt_iters; ++i )
		{
			chain.Invalidate();
			chain.Foreprop();
			std::cout << "Mean LL: " << chain.GetMeanLL() << std::endl;
			meanLL.Backprop( MatrixType::Identity(1, 1 ) );

			MatrixType dQ = Qmodel->GetBackpropValue();
			Eigen::Map<VectorType> dQvec( dQ.data(), dQ.size(), 1 );
			std::cout << "dQ: " << dQvec.transpose() << std::endl;
			VectorType qStep = dQvec * alpha;
			double qStepNorm = qStep.lpNorm<1>();
			if( qStepNorm > maxNorm ) { qStep = qStep * maxNorm / qStepNorm;  }
			VectorType pQ = Qmodel->GetParameters() + qStep;
			std::cout << "Q params: " << pQ.transpose() << std::endl;

			// MatrixType dR = Rmodel->GetBackpropValue();
			// Eigen::Map<VectorType> dRvec( dR.data(), dR.size(), 1 );
			// std::cout << "dR: " << dRvec.transpose() << std::endl;
			// VectorType pR = Rmodel->GetParameters() + dRvec * alpha;
			// std::cout << "R params: " << pR.transpose() << std::endl;
			
			Qmodel->SetParameters( pQ );
			// Rmodel->SetParameters( pR );
		}

	}
	time_t finish = clock();
	std::cout << "Took " << (finish - start) / (float) CLOCKS_PER_SEC << " seconds." << std::endl;
}