#include "fieldtrack/NoiseLearner.h"

#include "argus_utils/utils/ParamUtils.h"

namespace argus
{
NoiseLearner::NoiseLearner() {}

void NoiseLearner::Initialize( ros::NodeHandle& ph )
{
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

void NoiseLearner::BufferInfo( const FilterInfo& info )
{
	WriteLock lock( _bufferMutex );
	_infoBuffer.push_back( info );
	while( _infoBuffer.size() > _maxBufferSize ) { _infoBuffer.pop_front();
	}
}

void NoiseLearner::RegisterTransModel( const CovarianceModel::Ptr& model )
{
	_transCov = model;
	_chain.RegisterTransCov( model );
}

void NoiseLearner::RegisterObsModel( const std::string& name,
                                     const CovarianceModel::Ptr& model )

{
	_obsModels.push_back( model );
	_chain.RegisterObsSource( name, model );
}

void NoiseLearner::LearnSpin()
{
	WriteLock lock( _bufferMutex );
	if( _infoBuffer.size() < _minNumModules ) 
	{ 
		ROS_INFO_STREAM( "Info buffer has: " << _infoBuffer.size() );
		return; 
	}

	_chain.ClearChain();
	while( !_infoBuffer.empty() &&
	       _chain.NumModules() < _minNumModules )
	{
		_chain.ProcessInfo( _infoBuffer.front() );
		_infoBuffer.pop_front();
	}
	lock.unlock();

	for( unsigned int i = 0; i < _stepsPerBatch; ++i )
	{
		_chain.Invalidate();
		_chain.Foreprop();
		ROS_INFO_STREAM( "Mean LL: " << _chain.GetMeanLL() );
		_chain.Backprop();

		VectorType step = _stepSize * GetDerivatives();
		ROS_INFO_STREAM( "Deriv: " << step.transpose() );
		double l1Norm = step.lpNorm<1>();
		if( l1Norm > _maxL1Norm ) { step = step * _maxL1Norm / l1Norm; }
		StepParameters( step );
	}
}

unsigned int NoiseLearner::GetParamDim() const
{
	unsigned int dim = 0;
	dim += _transCov->GetParameters().size();
	BOOST_FOREACH( const CovarianceModel::Ptr & obs, _obsModels )
	{
		dim += obs->GetParameters().size();
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
	}
	ind += step;

	BOOST_FOREACH( const CovarianceModel::Ptr& obs, _obsModels )
	{
		step = obs->GetParameters().size();
		MatrixType dObs = obs->GetBackpropValue();
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

	BOOST_FOREACH( const CovarianceModel::Ptr& obs, _obsModels )
	{
		VectorType obsParams = obs->GetParameters();
		VectorType dObs = s.segment( ind, obsParams.size() );
		obs->SetParameters( obsParams + dObs );
		ind += obsParams.size();
	}
}
}