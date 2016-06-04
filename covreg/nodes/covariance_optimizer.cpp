#include "covreg/PerceptoInterface.h"
#include "argus_msgs/FilterStepInfo.h"
#include "argus_msgs/PredictionFeatures.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/filters/FilterUtils.h"

#include <ros/ros.h>

using namespace argus;

class Optimizer
{
public:

	static const unsigned int qInputDim = 13;
	static const unsigned int qOutputDim = 18;
	static const unsigned int qHiddenLayers = 2;
	static const unsigned int qLayerWidth = 30;

	static const unsigned int gInputDim = 13;
	static const unsigned int gOutputDim = 3;
	static const unsigned int gHiddenLayers = 2;
	static const unsigned int gLayerWidth = 10;

	static const unsigned int voInputDim = 13;
	static const unsigned int voOutputDim = 3;
	static const unsigned int voHiddenLayers = 2;
	static const unsigned int voLayerWidth = 10;

	Optimizer( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph ),
	qReg( qInputDim, qOutputDim, qHiddenLayers, qLayerWidth ),
	gyroReg( gInputDim, gOutputDim, gHiddenLayers, gLayerWidth ),
	voReg( voInputDim, voOutputDim, voHiddenLayers, voLayerWidth ),
	clipOptimizer( qReg )
	{
		qReg.RandomizeVarianceParams();
		qReg.ZeroCorrelationParams();
		gyroReg.RandomizeVarianceParams();
		gyroReg.ZeroCorrelationParams();
		voReg.RandomizeVarianceParams();
		voReg.ZeroCorrelationParams();

		clipOptimizer.AddObservationReg( gyroReg, "gyro" );
		clipOptimizer.AddObservationReg( voReg, "vo" );

		featSub = nodeHandle.subscribe( "features", 10, &Optimizer::FeatureCallback, this );
		featSub = nodeHandle.subscribe( "filter_info", 10, &Optimizer::FilterCallback, this );
	}

	void FeatureCallback( const argus_msgs::PredictionFeatures::ConstPtr& msg )
	{
		ParseMatrix( msg->features, latestFeatures );
	}

	void FilterCallback( const argus_msgs::FilterStepInfo::ConstPtr& msg )
	{
		if( msg->isPredict )
		{
			PredictInfo info = MsgToPredict( *msg );
			clipOptimizer.AddPredict( info, latestFeatures );
		}
		else
		{
			UpdateInfo info = MsgToUpdate( *msg );
			clipOptimizer.AddUpdate( info, latestFeatures, msg->header.frame_id );
		}

		if( clipOptimizer.NumClips() > 1000 )
		{
			percepto::SimpleConvergenceCriteria criteria;
			criteria.maxRuntime = 10; // 10 seconds
			clipOptimizer.Optimize( criteria );
		}
	}

private:

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;

	MatrixRegressor qReg, gyroReg, voReg;
	InnovationClipOptimizer clipOptimizer;

	VectorType latestFeatures;

	ros::Subscriber featSub;
	ros::Subscriber filterSub;

};

int main( int argc, char** argv )
{

	ros::init( argc, argv, "covariance_optimizer" );

	ros::NodeHandle nh, ph("~");

	Optimizer optimizer( nh, ph );
	
	ros::spin();
}