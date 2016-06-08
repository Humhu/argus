#include "covreg/ClipOptimizer.h"
#include "covreg/CovarianceEstimator.h"

#include "argus_msgs/FilterStepInfo.h"
#include "argus_msgs/PredictionFeatures.h"

#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/filters/FilterUtils.h"

#include <ros/ros.h>
#include <signal.h>

#include "argus_utils/synchronization/SynchronizationTypes.h"
#include "argus_utils/synchronization/ThreadsafeQueue.hpp"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

using namespace argus;

class Optimizer
{
public:

	static const unsigned int qInputDim = 6;
	static const unsigned int qOutputDim = 18;
	static const unsigned int qHiddenLayers = 1;
	static const unsigned int qLayerWidth = 50;

	static const unsigned int gInputDim = 6;
	static const unsigned int gOutputDim = 3;
	static const unsigned int gHiddenLayers = 1;
	static const unsigned int gLayerWidth = 50;

	static const unsigned int voInputDim = 6;
	static const unsigned int voOutputDim = 3;
	static const unsigned int voHiddenLayers = 1;
	static const unsigned int voLayerWidth = 10;

	Optimizer( const ros::NodeHandle& nh, const ros::NodeHandle& ph )
	: nodeHandle( nh ), privHandle( ph ),
	_dataBuffer( 100 ),
	_qReg( qOutputDim, qInputDim, qHiddenLayers, qLayerWidth ),
	_gyroReg( gOutputDim, gInputDim, gHiddenLayers, gLayerWidth ),
	_voReg( voOutputDim, voInputDim, voHiddenLayers, voLayerWidth )
	{
		_qReg.RandomizeVarianceParams();
		_qReg.ZeroCorrelationParams();
		_gyroReg.RandomizeVarianceParams();
		_gyroReg.ZeroCorrelationParams();
		_voReg.RandomizeVarianceParams();
		_voReg.ZeroCorrelationParams();

		InnovationClipParameters clipParams;
		GetParam<unsigned int>( privHandle, "max_clips_to_keep", clipParams.numClipsToKeep, 200 );
		GetParam<double>( privHandle, "l2_weight", clipParams.l2Weight, 1E-6 );
		GetParam<unsigned int>( privHandle, "batch_size", clipParams.batchSize, 30 );
		_clipOptimizer = std::make_shared<InnovationClipOptimizer>( _qReg, clipParams );
		_clipOptimizer->AddObservationReg( _gyroReg, "gyro" );
		_clipOptimizer->AddObservationReg( _voReg, "vo" );

		double minibatchTime;
		GetParam<double>( privHandle, "minibatch_time", minibatchTime, 1E-2 );
		GetParam<unsigned int>( privHandle, "min_clips_optimize", _minOptimizeSize );

		percepto::SimpleConvergenceCriteria criteria;
		criteria.maxRuntime = minibatchTime;
		_clipOptimizer->InitializeOptimization( criteria );

		double printRate;
		GetParam<double>( privHandle, "print_rate", printRate, 1.0 );
		_printPeriod = ros::Duration( 1.0/printRate );
		_lastPrintTime = ros::Time::now();

		_optimizerThread = boost::thread( boost::bind( &Optimizer::OptimizerCallback, this ) );

		_featSub = nodeHandle.subscribe( "features", 10, &Optimizer::FeatureCallback, this );
		_infoSub = nodeHandle.subscribe( "filter_info", 10, &Optimizer::FilterCallback, this );
	}

	// Destructor only gets called as the program is exiting, but it only exits
	// once all threads have terminated, so we have to tell the thread to stop
	void Terminate()
	{	
		ROS_INFO_STREAM( "Terminating optimizer..." );
		_optimizerThread.interrupt();
		_optimizerThread.join();
		ROS_INFO_STREAM( "Optimizer done." );
		_clipOptimizer->Print(50);
	}

	void FeatureCallback( const argus_msgs::PredictionFeatures::ConstPtr& msg )
	{
		// TODO Sort features by source
		WriteLock lock( _featureMutex );
		_latestFeatures = VectorType( msg->features.size() );
		ParseMatrix( msg->features, _latestFeatures );
	}

	void FilterCallback( const argus_msgs::FilterStepInfo::ConstPtr& msg )
	{
		ReadLock flock( _featureMutex );
		if( _latestFeatures.size() == 0 )
		{
			ROS_WARN_STREAM( "Skipping filter info until features received." );
			return;
		}
		VectorType feats = _latestFeatures;
		flock.unlock();

		_dataBuffer.EmplaceBack( feats, *msg );
	}

private:

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;

	MatrixRegressor _qReg, _gyroReg, _voReg;
	std::shared_ptr<InnovationClipOptimizer> _clipOptimizer;

	typedef std::pair<VectorType,argus_msgs::FilterStepInfo> InfoData;
	ThreadsafeQueue<InfoData> _dataBuffer;

	Mutex _featureMutex;
	VectorType _latestFeatures;

	ros::Subscriber _featSub;
	ros::Subscriber _infoSub;

	//std::shared_ptr<ros::Timer> _printTimer;
	ros::Time _lastPrintTime;
	ros::Duration _printPeriod;

	unsigned _minOptimizeSize;
	unsigned _ministeps;

	Mutex _optimizerMutex;
	boost::thread _optimizerThread;

	void OptimizerCallback()
	{
		boost::this_thread::interruption_enabled();
		try
		{
			while( !ros::isShuttingDown() )
			{
				boost::this_thread::interruption_point();

				ProcessBuffer();
				if( _clipOptimizer->NumClips() > _minOptimizeSize )
				{
					_clipOptimizer->Optimize();
				}
				PrintStatus();
			}
		}
		catch( boost::thread_interrupted e ) { return; }
	}

	void PrintStatus()
	{
		ros::Time now = ros::Time::now();
		if( now - _lastPrintTime < _printPeriod ) { return; }
		_lastPrintTime = now;

		if( _clipOptimizer->NumClips() == 0 ) { return; }

		double cost = _clipOptimizer->GetCost();
		// ROS_INFO_STREAM( "Q Params: " << _qReg.GetParameters().transpose() );
		// ROS_INFO_STREAM( "Gyro Params: " << _gyroReg.GetParameters().transpose() );
		// ROS_INFO_STREAM( "VO Params: " << _voReg.GetParameters().transpose() );
		ROS_INFO_STREAM( "Has " << _clipOptimizer->NumClips() << " clips." );
		ROS_INFO_STREAM( "Total cost: " << cost << std::endl
		                 << "Cost per clip: " << cost / _clipOptimizer->NumClips() );
		ROS_INFO_STREAM( "Message buffer size: " << _dataBuffer.Size() );
	}

	void ProcessBuffer()
	{
		size_t bsize = _dataBuffer.Size();
		for( size_t i = 0; i < bsize; ++i )
		{
			InfoData data;
			_dataBuffer.WaitPopBack( data );
			VectorType& features = data.first;
			argus_msgs::FilterStepInfo& msg = data.second;

			if( msg.isPredict )
			{
				PredictInfo info = MsgToPredict( msg );
				_clipOptimizer->AddPredict( info, features );
			}
			else
			{
				UpdateInfo info = MsgToUpdate( msg );
				_clipOptimizer->AddUpdate( info, features, msg.header.frame_id );
			}
		}
	}

};

int main( int argc, char** argv )
{

	ros::init( argc, argv, "covariance_optimizer" );

	ros::NodeHandle nh, ph("~");

	Optimizer optimizer( nh, ph );
	
	// ros::AsyncSpinner spinner(1);
	// spinner.start();
	// ros::waitForShutdown();
	ros::spin();
	optimizer.Terminate();

	return 0;
}