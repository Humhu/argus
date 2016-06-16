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

#include "broadcast/BroadcastReceiver.h"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <fstream>

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
	_testRx( "odom_features", 100 ),
	_qReg( "transition", qInputDim, qOutputDim, qHiddenLayers, qLayerWidth ),
	_gyroReg( "gyro", gInputDim, gOutputDim, gHiddenLayers, gLayerWidth ),
	_voReg( "vo", voInputDim, voOutputDim, voHiddenLayers, voLayerWidth )
	{
		_qReg.RandomizeVarianceParams();
		_qReg.ZeroCorrelationParams();
		_gyroReg.RandomizeVarianceParams();
		_gyroReg.ZeroCorrelationParams();
		_voReg.RandomizeVarianceParams();
		_voReg.ZeroCorrelationParams();

		InnovationClipParameters clipParams;
		GetParam<unsigned int>( privHandle, "max_eps_length", clipParams.maxEpisodeLength, 100 );
		GetParam<unsigned int>( privHandle, "max_eps_to_keep", clipParams.numEpisodesToKeep, 50 );
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

		_infoSub = nodeHandle.subscribe( "filter_info", 10, &Optimizer::FilterCallback, this );
	}

	void Print( std::ostream& os )
	{
		os << *_clipOptimizer << std::endl;
		os << "transition params: " << _qReg.GetParamSet()->GetParamsVec().transpose() << std::endl;
		os << "gyro params: " << _gyroReg.GetParamSet()->GetParamsVec().transpose() << std::endl;
		os << "vo params: " << _voReg.GetParamSet()->GetParamsVec().transpose() << std::endl;
	}

	// Destructor only gets called as the program is exiting, but it only exits
	// once all threads have terminated, so we have to tell the thread to stop
	void Terminate()
	{	
		_optimizerThread.interrupt();
		_optimizerThread.join();
	}

	void FilterCallback( const argus_msgs::FilterStepInfo::ConstPtr& msg )
	{
		if( !_testRx.HasReceived() )
		{
			ROS_WARN_STREAM( "Skipping filter info until features received." );
			return;
		}
		VectorType feats = _testRx.GetClosestReceived( msg->header.stamp );
		_dataBuffer.EmplaceBack( feats, *msg );
	}

private:

	ros::NodeHandle nodeHandle;
	ros::NodeHandle privHandle;

	BroadcastReceiver _testRx;
	CovarianceEstimator _qReg, _gyroReg, _voReg;
	std::shared_ptr<InnovationClipOptimizer> _clipOptimizer;

	typedef std::pair<VectorType,argus_msgs::FilterStepInfo> InfoData;
	ThreadsafeQueue<InfoData> _dataBuffer;

	ros::Subscriber _featSub;
	ros::Subscriber _infoSub;

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
				if( _clipOptimizer->NumEpisodes() > 1 ||
				    _clipOptimizer->CurrentEpisodeLength() > _minOptimizeSize )
				{
					// ROS_INFO_STREAM( "Beginning optimization..." );
					_clipOptimizer->Optimize();
					// ROS_INFO_STREAM( "Finished optimization!" );
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

		if( _clipOptimizer->NumEpisodes() == 0 ) { return; }

		double cost = _clipOptimizer->CalculateCost();
		// ROS_INFO_STREAM( "Q Params: " << _qReg.GetParameters().transpose() );
		// ROS_INFO_STREAM( "Gyro Params: " << _gyroReg.GetParameters().transpose() );
		// ROS_INFO_STREAM( "VO Params: " << _voReg.GetParameters().transpose() );
		ROS_INFO_STREAM( "Has " << _clipOptimizer->NumEpisodes() << " episodes." );
		ROS_INFO_STREAM( "Total cost: " << cost << std::endl );
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
	
	std::string outputPath;
	ph.param<std::string>( "output_path", outputPath, "results.txt" );

	std::ofstream outputLog( outputPath );
	if( !outputLog.is_open() )
	{
		ROS_WARN_STREAM( "Could not open log at: " << outputPath );
	}

	ros::spin();

	optimizer.Terminate();
	
	if( outputLog.is_open() )
	{
		optimizer.Print( outputLog );
	}

	return 0;
}