#include "fieldtrack/NoiseLearner.h"

#include "argus_utils/utils/ParamUtils.h"

namespace argus
{
NoiseLearner::NoiseLearner() 
{
	link_ports( _meanChainLL.GetOutput(), _meanLL.GetInput() );
}

void NoiseLearner::Initialize( ros::NodeHandle& ph )
{
	_numIterations = 0;
	GetParam( ph, "step_decay_rate", _stepDecayRate, 1.0 );

	GetParam( ph, "max_buffer_size", _maxBufferSize, (unsigned int) 100 );
	GetParamRequired( ph, "min_num_modules", _minNumModules );
	if( _maxBufferSize < _minNumModules )
	{
		throw std::logic_error( "Max buffer size less than min num modules!" );
	}

	GetParam( ph, "steps_per_batch", _stepsPerBatch, (unsigned int) 1 );
	GetParam( ph, "step_size", _stepSize, 1.0 );
	GetParam( ph, "max_step_l1", _maxL1Norm, std::numeric_limits<double>::infinity() );
}

void NoiseLearner::RegisterTransModel( const CovarianceModel::Ptr& model )
{
	WriteLock cLock( _chainMutex );
	_transCov = model;
}

void NoiseLearner::RegisterObsModel( const std::string& name,
                                     const CovarianceModel::Ptr& model )

{
	WriteLock cLock( _chainMutex );
	_obsModels[name] = model;
}

void NoiseLearner::BufferInfo( const FilterInfo& info )
{
	WriteLock lock( _bufferMutex );
	_infoBuffer.push_back( info );
	while( _infoBuffer.size() > _maxBufferSize ) { _infoBuffer.pop_front();
	}
}

void NoiseLearner::ClearBuffer()
{
	WriteLock bLock( _bufferMutex );
	_infoBuffer.clear();
}

unsigned int NoiseLearner::NumCappedChains() const
{
	ReadLock cLock( _chainMutex );
	return NumCappedChains( cLock );
}

void NoiseLearner::StartNewChain()
{
	WriteLock cLock( _chainMutex );	
	
	// First flush the info buffer
	ProcessBuffer( cLock );

	// Cap the chain if it exists
	if( !_chains.empty() )
	{
		if( GetCurrentChain().NumUpdates() == 0 )
		{
			ROS_WARN_STREAM( "Current chain had 0 updates! Cannot cap." );
			_chains.pop_back();
		}
		else
		{
			_meanChainLL.RegisterSource( GetCurrentChain().GetMeanLL() );
		}
	}
	

	// Create the new chain and set its models
	_chains.emplace_back( std::make_shared<LikelihoodChain>() );
	GetCurrentChain().RegisterTransCov( _transCov );
	typedef ObsModelRegistry::value_type Item;
	BOOST_FOREACH( const Item& item, _obsModels )
	{
		const std::string& name = item.first;
		const CovarianceModel::Ptr& model = item.second;
		GetCurrentChain().RegisterObsSource( name, model );
	}
}

void NoiseLearner::ClearChains()
{
	WriteLock cLock( _chainMutex );
	_meanChainLL.UnregisterAllSources( false );
	_chains.clear();	
}

LikelihoodChain& NoiseLearner::GetCurrentChain()
{
	if( _chains.empty() )
	{
		throw std::runtime_error( "No chains!" );
	}
	return *_chains.back();
}

void NoiseLearner::ProcessBuffer( WriteLock& cLock )
{
	CheckLockOwnership( cLock, &_chainMutex );
	WriteLock bLock( _bufferMutex );

	while( !_infoBuffer.empty() )
	{
		GetCurrentChain().ProcessInfo( _infoBuffer.front() );
		_infoBuffer.pop_front();
	}
}

void NoiseLearner::LearnSpin()
{
	WriteLock cLock( _chainMutex );
	if( NumCappedChains( cLock ) == 0 )
	{
		ROS_WARN_STREAM( "No chains to learn on!" );
		return;
	}

	for( unsigned int i = 0; i < _stepsPerBatch; ++i )
	{
		RunChains();

		double stepSize = GetStepSize();
		VectorType step = stepSize * GetDerivatives();
		ROS_INFO_STREAM( "Beta: " << stepSize << " Deriv: " << step.transpose() );
		double l1Norm = step.lpNorm<1>();
		if( l1Norm > _maxL1Norm ) { step = step * _maxL1Norm / l1Norm; }
		StepParameters( step );
		_numIterations++;
	}
}

void NoiseLearner::Invalidate()
{
	for( int i = 0; i < _chains.size() - 1; ++i )
	{
		_chains[i]->Invalidate();
	}
	_meanLL.Invalidate();
	
	_transCov->Invalidate();
	typedef ObsModelRegistry::value_type Item;
	BOOST_FOREACH( Item& item, _obsModels )
	{
		CovarianceModel::Ptr& model = item.second;
		model->Invalidate();
	}
}

void NoiseLearner::Foreprop()
{
	_transCov->Foreprop();
	typedef ObsModelRegistry::value_type Item;
	BOOST_FOREACH( Item& item, _obsModels )
	{
		CovarianceModel::Ptr& model = item.second;
		model->Foreprop();
	}
	for( int i = 0; i < _chains.size() - 1; ++i )
	{
		_chains[i]->Foreprop();
		double ll = _chains[i]->GetMeanLL().GetValue()(0);
		ROS_INFO_STREAM( "Chain " << i << " mean LL: " << ll );
	}
}

void NoiseLearner::RunChains()
{
	Invalidate();
	Foreprop();
	_meanLL.Backprop( MatrixType::Identity(1, 1) );
}

double NoiseLearner::GetStepSize() const
{
	return _stepSize / std::sqrt( _stepDecayRate * _numIterations + 1 );
}

unsigned int NoiseLearner::GetParamDim() const
{
	unsigned int dim = 0;
	dim += _transCov->GetParameters().size();
	typedef ObsModelRegistry::value_type Item;
	BOOST_FOREACH( const Item& item, _obsModels )
	{
		const CovarianceModel::Ptr model = item.second;		
		dim += model->GetParameters().size();
	}
	return dim;
}

VectorType NoiseLearner::GetDerivatives() const
{
	VectorType out = VectorType::Zero( GetParamDim() );

	unsigned int ind = 0;
	unsigned int step = _transCov->GetParameters().size();
	MatrixType dTrans = _transCov->GetBackpropValue();
	if( dTrans.size() == step )
	{
		Eigen::Map<const VectorType> dTransVec( dTrans.data(), dTrans.size(), 1 );
		out.segment( ind, dTransVec.size() ) = dTransVec;
		std::cout << "dTransVec: " << dTransVec.transpose() << std::endl;
	}
	else
	{
		std::cout << "dTrans not right size!: " << dTrans << std::endl;
	}
	ind += step;

	typedef ObsModelRegistry::value_type Item;
	BOOST_FOREACH( const Item& item, _obsModels )
	{
		const CovarianceModel::Ptr model = item.second;
		step = model->GetParameters().size();
		MatrixType dObs = model->GetBackpropValue();
		if( dObs.size() == step )
		{
			Eigen::Map<const VectorType> dVec( dObs.data(), dObs.size(), 1 );
			out.segment( ind, dVec.size() ) = dVec;
		}
		ind += step;
	}
	return out;
}

void NoiseLearner::StepParameters( const VectorType& s )
{
	if( s.size() != GetParamDim() )
	{
		throw std::invalid_argument( "Incorrect step dimension" );
	}

	unsigned int ind = 0;
	VectorType transParams = _transCov->GetParameters();
	VectorType dTrans = s.segment( ind, transParams.size() );
	_transCov->SetParameters( transParams + dTrans );
	ind += dTrans.size();

	typedef ObsModelRegistry::value_type Item;
	BOOST_FOREACH( const Item& item, _obsModels )
	{
		const CovarianceModel::Ptr model = item.second;		
		VectorType obsParams = model->GetParameters();
		VectorType dObs = s.segment( ind, obsParams.size() );
		model->SetParameters( obsParams + dObs );
		ind += obsParams.size();
	}
}
}