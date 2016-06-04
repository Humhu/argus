#include "covreg/PerceptoInterface.h"
#include "percepto/utils/Randomization.hpp"

using namespace argus;

int main( int argc, char** argv )
{
	unsigned int inputDim = 10;
	unsigned int outputDim = 6;
	unsigned int numHiddenLayers = 2;
	unsigned int layerWidth = 20;

	MatrixRegressor matReg( inputDim, outputDim, numHiddenLayers, layerWidth );
	VectorType params;
	

	VectorType input( inputDim );
	percepto::randomize_vector( input );
	MatrixType output = matReg.Evaluate( input );

	MatrixRegressor transReg( matReg );
	MatrixRegressor gyroReg( matReg );
	MatrixRegressor voReg( matReg );

	InnovationClipOptimizer clipOptimizer( transReg );
	PredictInfo predict;
	UpdateInfo update;
	VectorType predInput, upInput;

	clipOptimizer.AddObservationReg( gyroReg, "gyro" );
	clipOptimizer.AddObservationReg( voReg, "vo" );

	clipOptimizer.AddPredict( predict, predInput );
	clipOptimizer.AddUpdate( update, upInput, "gyro" );
	clipOptimizer.AddPredict( predict, predInput );
	clipOptimizer.AddUpdate( update, upInput, "vo" );

	// clipOptimizer.UpdateRegressors();

	return 0;
}