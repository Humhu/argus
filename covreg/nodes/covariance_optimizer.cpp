#include "covreg/ClipOptimizer.h"
#include "covreg/CovarianceEstimator.h"
#include "covreg/CovarianceEstimatorInfo.h"
#include "covreg/EstimatorInfoParsers.h"
#include "covreg/OptimizerStatus.h"

#include "argus_msgs/FilterStepInfo.h"

#include "argus_utils/utils/YamlUtils.h"
#include "argus_utils/utils/MatrixUtils.h"
#include "argus_utils/utils/ParamUtils.h"
#include "argus_utils/filters/FilterUtils.h"

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>
#include <signal.h>

#include "argus_utils/synchronization/SynchronizationTypes.h"
#include "argus_utils/synchronization/ThreadsafeQueue.hpp"

#include "broadcast/BroadcastReceiver.h"

#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

#include <fstream>

using namespace argus_msgs;
using namespace argus;
using namespace covreg;

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
	static const unsigned int voLayerWidth = 50;

	Optimizer( ros::NodeHandle& nodeHandle, ros::NodeHandle& ph )
	: privHandle( ph ),
	rxInitialized( false ),
	epsInitialized( false ),
	lastSeq( 0 )
	{
		GetParam<double>( privHandle, "feature_cache_time", 
		                  _featureCacheTime, 1.0 );

		// Number of info updates to store max
		unsigned int infoBufferSize;
		GetParam<unsigned int>( privHandle, "info_buffer_size", infoBufferSize, 0 );
		_dataBuffer.SetMaxSize( infoBufferSize );

		std::string logPath;
		if( GetParam( privHandle, "output_log_path", logPath ) )
		{
			_logOutFile.open( logPath );
		}

		// Iterate over all models
		WriteLock lock( _initMutex );
		XmlRpc::XmlRpcValue modelsXml;
		GetParam( privHandle, "models", modelsXml );
		YAML::Node modelsYaml = XmlToYaml( modelsXml );
		YAML::Node::const_iterator iter;
		for( iter = modelsYaml.begin(); iter != modelsYaml.end(); ++iter )
		{
			const std::string& modelName = iter->first.as<std::string>();
			YAML::Node modelYaml = iter->second;
			RegisterModel( modelName, modelYaml );
		}

		if( _estRegistry.count( "transition" ) == 0 )
		{
			ROS_ERROR_STREAM( "Must specify transition model." );
			exit( -1 );
		}

		InnovationClipParameters clipParams;
		GetParam<unsigned int>( privHandle, "max_eps_length", clipParams.maxEpisodeLength, 100 );
		// GetParam<unsigned int>( privHandle, "max_eps_to_keep", clipParams.numEpisodesToKeep, 50 );
		GetParam<double>( privHandle, "l2_weight", clipParams.l2Weight, 1E-6 );
		GetParam<unsigned int>( privHandle, "batch_size", clipParams.batchSize, 30 );

		CovarianceEstimator& transReg = *(_estRegistry["transition"].estimator);
		_clipOptimizer = std::make_shared<InnovationClipOptimizer>( transReg, clipParams );
		// _clipOptimizer = std::make_shared<InnovationClipOptimizer>( clipParams );
		
		// TODO Iterate and add all non-transition regs to the optimizer
		typedef EstimatorRegistry::value_type Item;
		BOOST_FOREACH( Item& item, _estRegistry )
		{
			_clipOptimizer->AddObservationReg( *item.second.estimator, item.first );
		}

		GetParam<unsigned int>( privHandle, "num_optimizer_steps", _numOptimizerSteps, 1 );
		// GetParam<unsigned int>( privHandle, "min_eps_to_optimize", _minOptimizeSize );
		GetParamRequired<double>( privHandle, "min_span_to_optimize", _minOptimizeTime );
		GetParamRequired<double>( privHandle, "max_span_to_keep", _maxEpisodeSpan );
		GetParam<bool>( privHandle, "clear_after_optimize", _clearAfterOptimize, false );

		percepto::SimpleConvergenceCriteria criteria;
		GetParam<double>( privHandle, "convergence/max_time", criteria.maxRuntime, std::numeric_limits<double>::infinity() );
		GetParam<unsigned int>( privHandle, "convergence/max_iters", criteria.maxIterations, std::numeric_limits<unsigned int>::max() );
		GetParam<double>( privHandle, "convergence/min_avg_delta", criteria.minAverageDelta, -std::numeric_limits<double>::infinity() );
		GetParam<double>( privHandle, "convergence/min_avg_grad", criteria.minAverageGradient, -std::numeric_limits<double>::infinity() );
		percepto::AdamParameters optParams;
		GetParam( privHandle, "optimizer/step_size", optParams.alpha, 1E-3 );
		GetParam( privHandle, "optimizer/max_step", optParams.maxStepElement, 1.0 );
		GetParam( privHandle, "optimizer/beta1", optParams.beta1, 0.9 );
		GetParam( privHandle, "optimizer/beta2", optParams.beta2, 0.99 );
		GetParam( privHandle, "optimizer/epsilon", optParams.epsilon, 1E-7 );
		GetParam( privHandle, "optimizer/enable_decay", optParams.enableDecay, false );
		_clipOptimizer->InitializeOptimization( criteria, optParams );

		double bufferProcessRate;
		GetParam<double>( privHandle, "buffer_process_rate", bufferProcessRate, 10.0 );
		_printPeriod = ros::Duration( bufferProcessRate );

		double statusPublishRate;
		GetParam<double>( privHandle, "status_publish_rate", statusPublishRate, 1.0 );
		_statusPeriod = ros::Duration( 1.0/statusPublishRate );
		_statusPub = privHandle.advertise<OptimizerStatus>( "optimizer_status", 10 );
		_startTime = ros::Time::now();

		double printRate;
		GetParam<double>( privHandle, "print_rate", printRate, 1.0 );
		_printPeriod = ros::Duration( 1.0/printRate );
		_lastPrintTime = ros::Time::now();

		double paramPublishRate;
		GetParam<double>( privHandle, "param_publish_rate", paramPublishRate, 1.0 );
		_publishPeriod = ros::Duration( 1.0/paramPublishRate );
		_lastPublishTime = ros::Time::now();

		_infoSub = nodeHandle.subscribe( "filter_info", infoBufferSize, &Optimizer::FilterCallback, this );

		_waitTimer = nodeHandle.createTimer( ros::Duration( 1.0 ), 
		                                     &Optimizer::ReceiveWaitTimerCallback, 
		                                     this, true );
		
		_optimizerThread = boost::thread( boost::bind( &Optimizer::OptimizerCallback, this ) );
	}


	void RegisterModel( const std::string& name, const YAML::Node& info )
	{
		if( _estRegistry.count( name ) > 0 )
		{
			ROS_ERROR_STREAM( "Already registered estimator: " << name );
			exit( -1 );
		}
		ROS_INFO_STREAM( "Registering model: " << name );

		EstimatorRegistration& reg = _estRegistry[ name ];
		std::string paramTopic = name + "/param_updates";
		reg.estimator = std::make_shared<CovarianceEstimator>( name, info );
		reg.paramPublisher = privHandle.advertise<CovarianceEstimatorInfo>( paramTopic, 10 );
		if( name != "transition" )
		{
			reg.weight = TryYamlField<double>( info, "weight" );
		}

		// Register all features this model needs
		//reg.features = TryYamlField<std::vector<std::string>>( info, "features" );
		unsigned int fDim = 0;
		YAML::Node inputStreams = info["features"];
		YAML::const_iterator iter;
		for( iter = inputStreams.begin(); iter != inputStreams.end(); ++iter )
		{
			const std::string& featureName = iter->first.as<std::string>();
			RegisterReceiver( featureName, iter->second );
			fDim += _receivers.at( featureName ).GetDim();
		}

		if( fDim != reg.estimator->InputDim() )
		{
			ROS_ERROR_STREAM( "Model: " << name << " has input dim: " << 
			                  reg.estimator->InputDim() << " but features have total dim: " <<
			                  fDim );
			exit( -1 );
		}

		// Load parameters if we have them
		bool loaded = false;
		if( info["load_path"] )
		{
			CovarianceEstimatorInfo calib;
			std::string infoPath = info["load_path"].as<std::string>();
			if( ReadInfo( infoPath, calib ) )
			{
				ROS_INFO_STREAM( "Loading params for: " << name <<
				                 " from: " << infoPath );
				reg.estimator->SetParamsMsg( calib );
				loaded = true;
			}
			else
			{
				ROS_WARN_STREAM( "Could not load parameters for: " << name <<
			                     " from: " << infoPath );
			}
		}
		if( !loaded )
		{
			ROS_INFO_STREAM( "Initializing: " << name << " to default params." );
			reg.estimator->RandomizeVarianceParams();
			reg.estimator->ZeroCorrelationParams();
			// reg.estimator->ZeroParams();
			if( info["offsets"] )
			{
				ROS_INFO_STREAM( "Setting offsets." );
				std::vector<double> off = info["offsets"].as<std::vector<double>>();
				reg.estimator->SetVarianceOffsets( GetVectorView( off ) );
			}
		}



		// Verify the output path is OK
		if( info["save_path"] ) 
		{
			reg.outputPath = info["save_path"].as<std::string>();
			std::ofstream outFile( reg.outputPath );
			if( !outFile.is_open() )
			{
				ROS_ERROR_STREAM( "Could not open output file for: " << name <<
				                  " at: " << reg.outputPath );
				exit( -1 );
			}
		}
	}

	void RegisterReceiver( const std::string& streamName, const YAML::Node& props )
	{
		if( _receivers.count( streamName ) > 0 ) { return; }
		_receivers[streamName].Initialize( streamName, props );
	}

	void SaveLog()
	{
		if( _logOutFile.is_open() )
		{
			// _clipOptimizer->CalculateCost();
			_logOutFile << *_clipOptimizer << std::endl;
			typedef EstimatorRegistry::value_type Item;
			BOOST_FOREACH( Item& item, _estRegistry )
			{
				_logOutFile << item.first << " params: " << 
				               item.second.estimator->GetParamSet()->GetParamsVec().transpose() << 
				               std::endl;
			}
		}
		_logOutFile.close();
	}

	void SaveParameters()
	{
		typedef EstimatorRegistry::value_type Item;
		BOOST_FOREACH( Item& item, _estRegistry )
		{
			const std::string& outputPath = item.second.outputPath;
			if( outputPath.empty() ) { continue; }
			ROS_INFO_STREAM( "Writing parameters for " << item.first <<
			                 " to: " << outputPath );
			if( !WriteInfo( outputPath, item.second.estimator->GetParamsMsg() ) )
			{
				ROS_WARN_STREAM( "Failed to write parameters for: " << item.first <<
				                " to: " << outputPath );
			}
		}
	}

	// Destructor only gets called as the program is exiting, but it only exits
	// once all threads have terminated, so we have to tell the thread to stop
	void Terminate()
	{
		_optimizerThread.interrupt();
		_optimizerThread.join();
	}

	void ReceiveWaitTimerCallback( const ros::TimerEvent& event )
	{
		WriteLock lock( _initMutex );
		typedef ReceiverRegistry::value_type Item;
		BOOST_FOREACH( Item& item, _receivers )
		{
			if( !item.second.IsReady() )
			{
				ROS_WARN_STREAM( "Stream: " << item.second.GetStreamName() <<
				                 " has not received data yet." );
				_waitTimer.stop();
				_waitTimer.setPeriod( ros::Duration( 1.0 ) );
				_waitTimer.start();
				return;
			}
		}
		ROS_INFO_STREAM( "All streams have received. Optimizer is ready." );
		rxInitialized = true;
	}

private:

	ros::NodeHandle privHandle;

	struct EstimatorRegistration
	{
		ros::Publisher paramPublisher;
		std::shared_ptr<CovarianceEstimator> estimator;
		std::vector<std::string> features;
		std::string outputPath;
		double weight;
	};
	typedef std::unordered_map<std::string, EstimatorRegistration> EstimatorRegistry;
	EstimatorRegistry _estRegistry;

	typedef std::unordered_map<std::string, BroadcastReceiver> ReceiverRegistry;
	ReceiverRegistry _receivers;

	std::shared_ptr<InnovationClipOptimizer> _clipOptimizer;

	typedef std::pair<VectorType,argus_msgs::FilterStepInfo> InfoData;
	ThreadsafeQueue<InfoData> _dataBuffer;

	double _featureCacheTime;

	ros::Time _startTime;

	unsigned int _numOptimizerSteps;
	ros::Subscriber _infoSub;
	
	ros::Publisher _statusPub;
	ros::Time _lastStatusTime;
	ros::Duration _statusPeriod;

	ros::Time _lastPrintTime;
	ros::Duration _printPeriod;

	ros::Time _lastPublishTime;
	ros::Duration _publishPeriod;
	ros::Duration _processPeriod;

	unsigned _minOptimizeSize;
	double _minOptimizeTime;
	double _maxEpisodeSpan;
	unsigned _ministeps;
	bool _clearAfterOptimize;

	Mutex _optimizerMutex;
	boost::thread _optimizerThread;

	std::ofstream _logOutFile;

	ros::Timer _waitTimer;
	Mutex _initMutex;
	bool rxInitialized;
	bool epsInitialized;

	unsigned int lastSeq;
	
	void FilterCallback( const argus_msgs::FilterStepInfo::ConstPtr& msg )
	{
		WriteLock lock( _initMutex );
		if( !rxInitialized ) { return; }

		const std::string& sourceName = msg->header.frame_id;
		if( sourceName != "transition" && _estRegistry.count( sourceName ) == 0 )
		{
			ROS_WARN_STREAM( "Received step info from unregistered model: " << sourceName );
			return;
		}

		const EstimatorRegistration& reg = _estRegistry.at( msg->header.frame_id );
		VectorType feat( reg.estimator->InputDim() );
		unsigned int featInd = 0;

		try{
			BOOST_FOREACH( const std::string& featureName, reg.features )
			{
				BroadcastReceiver& rx = _receivers.at( featureName );
				StampedFeatures f;
				rx.ReadStream( msg->header.stamp, f );
				feat.segment( featInd, rx.GetDim() ) = f.features;
				featInd += rx.GetDim();
			}
		}
		catch( std::runtime_error e ) 
		{ 
			ROS_WARN_STREAM( "Could not get features for timestamp: " << msg->header.stamp );
			return; 
		}

		_dataBuffer.EmplaceBack( feat, *msg );
	}

	void OptimizerCallback()
	{
		boost::this_thread::interruption_enabled();
		try
		{
			while( !ros::isShuttingDown() )
			{
				boost::this_thread::interruption_point();
				PublishParams();

				ProcessBuffer();
				ros::Time now = ros::Time::now();
				ros::Time earliestTime = _clipOptimizer->GetEarliestTime();
				double span = (now - earliestTime).toSec();

				if( !epsInitialized )
				{
					if( span > _minOptimizeTime )
					{
						epsInitialized = true;
						// ROS_INFO_STREAM( "Episode buffer filled. Starting optimization." );
					}
					else
					{
						// ROS_INFO_STREAM( "Span: " << span );
						_processPeriod.sleep();
						continue;
					}
				}

				while( _clipOptimizer->NumEpisodes() > 0 && span > _maxEpisodeSpan )
				{
					// ROS_INFO_STREAM( "Span: " << span << " greater than max. Trimming oldest episode." );
					_clipOptimizer->RemoveEarliestEpisode();
					earliestTime = _clipOptimizer->GetEarliestTime();
					span = (now - earliestTime).toSec();
				}

				if( _clipOptimizer->NumEpisodes() == 0 )
				{
					ros::Duration( 1.0 ).sleep();
					continue;
				}

				// if( _clipOptimizer->NumEpisodes() >= _minOptimizeSize )

				bool converged = false;
				for( unsigned int i = 0; i < _numOptimizerSteps; i++ )
				{
					converged = _clipOptimizer->Optimize();
				}
				if( converged )
				{
					ROS_INFO_STREAM( "Optimizer has converged. Saving..." );
					break;
				}
				PrintStatus();
				PublishStatus();
				if( _clearAfterOptimize )
				{
					epsInitialized = false;
					ROS_INFO_STREAM( "Clearing episodes..." );
					while( _clipOptimizer->NumEpisodes() > 0 )
					{
						_clipOptimizer->RemoveEarliestEpisode();
					}
					_dataBuffer.Clear();
				}


			}
			SaveParameters();
			SaveLog();
			exit( 0 );
		}
		catch( std::exception e ) 
		{
			ROS_INFO_STREAM( "Optimizer thread: " << e.what() );
			SaveParameters();
			SaveLog();
			return; 
		}
	}

	void PublishStatus()
	{
		ros::Time now = ros::Time::now();
		if( now - _lastStatusTime < _statusPeriod ) { return; }
		_lastStatusTime = now;

		if( _clipOptimizer->NumEpisodes() == 0 ) { return; }

		OptimizerStatus msg;
		msg.header.stamp = now;
		msg.header.frame_id = "covariance_optimizer";
		msg.current_objective = _clipOptimizer->CalculateCost();
		msg.secs_since_start = ( now - _startTime ).toSec();
		_statusPub.publish( msg );
	}

	void PrintStatus()
	{
		ros::Time now = ros::Time::now();
		if( now - _lastPrintTime < _printPeriod ) { return; }
		_lastPrintTime = now;

		if( _clipOptimizer->NumEpisodes() == 0 ) { return; }

		double cost = _clipOptimizer->CalculateCost();
		ROS_INFO_STREAM( "Has " << _clipOptimizer->NumEpisodes() << " episodes." );
		ROS_INFO_STREAM( "Total cost: " << cost << std::endl );
		ROS_INFO_STREAM( "Message buffer size: " << _dataBuffer.Size() );
		typedef EstimatorRegistry::value_type Item;
		BOOST_FOREACH( const Item& item, _estRegistry )
		{
			ROS_INFO_STREAM( "Model: " << item.first << std::endl << item.second.estimator->GetModule().dReg );
		}
		// ROS_INFO_STREAM( "vo: " << std::endl << _estRegistry.at("vo").estimator->GetModule().dReg );
	}

	void ProcessBuffer()
	{
		size_t bsize = _dataBuffer.Size();
		for( size_t i = 0; i < bsize; ++i )
		{
			InfoData data;
			_dataBuffer.WaitPopFront( data );
			VectorType& features = data.first;
			argus_msgs::FilterStepInfo& msg = data.second;

			if( msg.step_num != lastSeq + 1 )
			{
				ROS_WARN_STREAM( "Got sequence " << msg.step_num <<
				                 " but expected " << lastSeq + 1 );
				_clipOptimizer->BreakCurrentEpisode();
			}

			lastSeq = msg.step_num;
			if( msg.isPredict )
			{
				PredictInfo info = MsgToPredict( msg );
				_clipOptimizer->AddPredict( info, features );
			}
			else
			{
				const std::string& sourceName = msg.header.frame_id;
				UpdateInfo info = MsgToUpdate( msg );
				_clipOptimizer->AddUpdate( info, 
				                           features, 
				                           sourceName, 
				                           _estRegistry[sourceName].weight,
				                           msg.header.stamp );
			}
		}
	}

	void PublishParams()
	{
		ros::Time now = ros::Time::now();
		if( now - _lastPublishTime < _publishPeriod ) { return; }
		_lastPublishTime = now;

		typedef EstimatorRegistry::value_type Item;
		BOOST_FOREACH( Item& item, _estRegistry )
		{
			item.second.paramPublisher.publish( item.second.estimator->GetParamsMsg() );
		}
	}

};

int main( int argc, char** argv )
{

	ros::init( argc, argv, "covariance_optimizer" );

	ros::NodeHandle nh, ph("~");

	Optimizer optimizer( nh, ph );
	
	try
	{
		ros::spin();
	}
	catch( std::runtime_error e ) 
	{
		ROS_ERROR_STREAM( "Error: " << e.what() );
	}

	optimizer.Terminate();

	return 0;
}
