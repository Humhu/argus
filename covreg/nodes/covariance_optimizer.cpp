#include "covreg/ClipOptimizer.h"
#include "covreg/CovarianceEstimator.h"
#include "covreg/CovarianceEstimatorInfo.h"
#include "covreg/EstimatorInfoParsers.h"

#include "argus_msgs/FilterStepInfo.h"
#include "argus_msgs/PredictionFeatures.h"

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
	initialized( false ),
	lastSeq( 0 )
	{
		GetParam<double>( privHandle, "feature_cache_time", 
		                  _featureCacheTime, 1.0 );

		// Number of info updates to store max
		unsigned int infoBufferSize;
		GetParam<unsigned int>( privHandle, "info_buffer_size", infoBufferSize, 500 );
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
		GetParam<unsigned int>( privHandle, "max_eps_to_keep", clipParams.numEpisodesToKeep, 50 );
		GetParam<double>( privHandle, "l2_weight", clipParams.l2Weight, 1E-6 );
		GetParam<unsigned int>( privHandle, "batch_size", clipParams.batchSize, 30 );

		CovarianceEstimator& transReg = *(_estRegistry["transition"].estimator);
		_clipOptimizer = std::make_shared<InnovationClipOptimizer>( transReg, clipParams );
		
		// TODO Iterate and add all non-transition regs to the optimizer
		typedef EstimatorRegistry::value_type Item;
		BOOST_FOREACH( Item& item, _estRegistry )
		{
			_clipOptimizer->AddObservationReg( *item.second.estimator, item.first );
		}

		GetParam<unsigned int>( privHandle, "min_clips_optimize", _minOptimizeSize );

		percepto::SimpleConvergenceCriteria criteria;
		GetParam<double>( privHandle, "convergence/batch_time", criteria.maxRuntime, 1E-2 );
		percepto::AdamParameters optParams;
		GetParam( privHandle, "optimizer/step_size", optParams.alpha, 1E-3 );
		GetParam( privHandle, "optimizer/beta1", optParams.beta1, 0.9 );
		GetParam( privHandle, "optimizer/beta2", optParams.beta2, 0.99 );
		GetParam( privHandle, "optimizer/epsilon", optParams.epsilon, 1E-7 );
		_clipOptimizer->InitializeOptimization( criteria, optParams );

		double printRate;
		GetParam<double>( privHandle, "print_rate", printRate, 1.0 );
		_printPeriod = ros::Duration( 1.0/printRate );
		_lastPrintTime = ros::Time::now();

		double paramPublishRate;
		GetParam<double>( privHandle, "param_publish_rate", paramPublishRate, 1.0 );
		_publishPeriod = ros::Duration( 1.0/paramPublishRate );
		_lastPublishTime = ros::Time::now();

		_optimizerThread = boost::thread( boost::bind( &Optimizer::OptimizerCallback, this ) );

		// sub = std::make_shared<message_filters::Subscriber<FilterStepInfo>>( nodeHandle, "filter_info", infoBufferSize );
		// seq = std::make_shared<message_filters::TimeSequencer<FilterStepInfo>>( *sub, ros::Duration(1.0), ros::Duration(0.1), infoBufferSize);
		// seq->registerCallback( &Optimizer::FilterCallback, this );

		_infoSub = nodeHandle.subscribe( "filter_info", infoBufferSize, &Optimizer::FilterCallback, this );

		_waitTimer = nodeHandle.createTimer( ros::Duration( 1.0 ), 
		                                     &Optimizer::ReceiveWaitTimerCallback, 
		                                     this, true );
	}

	std::shared_ptr<message_filters::Subscriber<FilterStepInfo>> sub;
	std::shared_ptr<message_filters::TimeSequencer<FilterStepInfo>> seq;

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
		reg.features = info["features"].as<std::vector<std::string>>();
		if( name != "transition" )
		{
			//reg.ssRate = info["ss_rate"].as<double>();
			reg.samplesPerEp = info["samples_per_episode"].as<unsigned int>();
		}

		// Register all features this model needs
		unsigned int fDim = 0;
		BOOST_FOREACH( const std::string& feature, reg.features )
		{
			RegisterReceiver( feature );
			fDim += _receivers.at( feature ).OutputDim();
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

	void RegisterReceiver( const std::string& streamName )
	{
		if( _receivers.count( streamName ) > 0 ) { return; }
		_receivers.emplace( std::piecewise_construct,
		                    std::forward_as_tuple( streamName ),
		                    std::forward_as_tuple( streamName, _featureCacheTime ) );
		// NOTE Can't do this since we would block the main thread from getting 
		// to the ros::spin() to pump the message queue!
		// while( !_receivers[streamName].HasReceived() )
		// {
		// 	ROS_WARN_STREAM( "Waiting for stream: " << streamName << "..." );
		// 	ros::Duration( 1.0 ).sleep();
		// }
	}

	void SaveLog()
	{
		if( _logOutFile.is_open() )
		{
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
			if( !item.second.HasReceived() )
			{
				ROS_WARN_STREAM( "Stream: " << item.second.StreamName() <<
				                 " has not received data yet." );
				_waitTimer.start();
				return;
			}
		}
		ROS_INFO_STREAM( "All streams have received. Optimizer is ready." );
		initialized = true;
	}

private:

	ros::NodeHandle privHandle;

	struct EstimatorRegistration
	{
		ros::Publisher paramPublisher;
		std::shared_ptr<CovarianceEstimator> estimator;
		std::vector<std::string> features;
		std::string outputPath;
		double ssRate;
		unsigned int samplesPerEp;
	};
	typedef std::unordered_map<std::string, EstimatorRegistration> EstimatorRegistry;
	EstimatorRegistry _estRegistry;

	typedef std::unordered_map<std::string, BroadcastReceiver> ReceiverRegistry;
	ReceiverRegistry _receivers;

	std::shared_ptr<InnovationClipOptimizer> _clipOptimizer;

	typedef std::pair<VectorType,argus_msgs::FilterStepInfo> InfoData;
	ThreadsafeQueue<InfoData> _dataBuffer;

	double _featureCacheTime;

	ros::Subscriber _infoSub;

	ros::Time _lastPrintTime;
	ros::Duration _printPeriod;

	ros::Time _lastPublishTime;
	ros::Duration _publishPeriod;

	unsigned _minOptimizeSize;
	unsigned _ministeps;

	Mutex _optimizerMutex;
	boost::thread _optimizerThread;

	std::ofstream _logOutFile;

	ros::Timer _waitTimer;
	Mutex _initMutex;
	bool initialized;

	unsigned int lastSeq;
	
	void FilterCallback( const argus_msgs::FilterStepInfo::ConstPtr& msg )
	{
		ReadLock lock( _initMutex );
		if( !initialized ) { return; }

		const std::string& sourceName = msg->header.frame_id;
		if( _estRegistry.count( sourceName ) == 0 )
		{
			ROS_WARN_STREAM( "Received step info from unregistered model: " << sourceName );
			return;
		}

		const EstimatorRegistration& reg = _estRegistry.at( msg->header.frame_id );
		VectorType feat( reg.estimator->InputDim() );
		unsigned int featInd = 0;
		BOOST_FOREACH( const std::string& featureName, reg.features )
		{
			BroadcastReceiver& rx = _receivers.at( featureName );
			feat.segment( featInd, rx.OutputDim() ) = rx.GetClosestReceived( msg->header.stamp );
			featInd += rx.OutputDim();
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

				ProcessBuffer();
				if( _clipOptimizer->NumEpisodes() > 1 ||
				    _clipOptimizer->CurrentEpisodeLength() > _minOptimizeSize )
				{
					ROS_INFO_STREAM( "Beginning optimization..." );
					_clipOptimizer->Optimize();
					ROS_INFO_STREAM( "Finished optimization!" );
					PublishParams();
				}
				PrintStatus();
			}
			SaveParameters();
			SaveLog();
		}
		catch( std::exception e ) 
		{
			ROS_INFO_STREAM( "Optimizer thread: " << e.what() );
			SaveParameters();
			SaveLog();
			return; 
		}
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

			if( msg.header.seq != lastSeq + 1 )
			{
				ROS_WARN_STREAM( "Got sequence " << msg.header.seq <<
				                 " but expected " << lastSeq + 1 );
				lastSeq = msg.header.seq;
				continue;
			}

			lastSeq = msg.header.seq;
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
				                           _estRegistry[sourceName].samplesPerEp );
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
