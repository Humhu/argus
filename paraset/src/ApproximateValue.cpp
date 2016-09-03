#include "paraset/ApproximateValue.h"
#include "paraset/RewardStamped.h"

#include "percepto/utils/Randomization.hpp"

namespace argus
{

ApproximateValue::ApproximateValue() {}

void ApproximateValue::Initialize( ros::NodeHandle& nh,
                                   ros::NodeHandle& ph )
{
	ros::NodeHandle ih( ph.resolveName( "input_streams" ) );
	_receiver.Initialize( ih );
	unsigned int inputDim = _receiver.GetDim();

	ros::NodeHandle mh( ph.resolveName( "network" ) );
	unsigned int numHiddenLayers, layerWidth;
	GetParamRequired( mh, "num_hidden_layers", numHiddenLayers );
	GetParamRequired( mh, "layer_width", layerWidth );

	GetParamRequired( mh, "type", _moduleType );
	if( _moduleType == "perceptron" )
	{
		_approximator = std::make_shared<PerceptronFunctionApproximator>( inputDim,
		                                                                  1,
		                                                                  numHiddenLayers,
		                                                                  layerWidth );
	}
	else
	{
		throw std::invalid_argument( "ApproximateValue: Unknown network type: " + _moduleType );
	}
	_approximator->SetInputSource( &_approximatorInput );
	_approximatorParams = _approximator->CreateParameters();

	double initialValue, initialMagnitude;
	GetParamRequired( mh, "initial_magnitude", initialMagnitude );
	GetParamRequired( ph, "initial_value", initialValue );

	VectorType w = _approximatorParams->GetParamsVec();
	percepto::randomize_vector( w, -initialMagnitude, initialMagnitude );
	_approximatorParams->SetParamsVec( w );
	_approximator->InitializeOutput( initialValue );

	ROS_INFO_STREAM( "ApproximateValue: Network initialized: " << std::endl << _approximator->Print() );

	std::string updateTopic;
	if( GetParam( ph, "update_topic", updateTopic ) )
	{
		_paramsSub = nh.subscribe( updateTopic, 1, &ApproximateValue::ParamsCallback, this );
	}
}

double ApproximateValue::Evaluate( const ParamAction& x ) const
{
	// TODO Move to common class
	StampedFeatures features;
	if( !_receiver.ReadStream( x.time, features ) )
	{
		throw std::out_of_range( "ApproximateValue: Could not get input stream." );
	}

	_approximatorInput.Invalidate();
	_approximator->Invalidate();
	_approximatorInput.SetOutput( GetInput( x.time ) );
	_approximatorInput.Foreprop();
	_approximator->Foreprop();
	return _approximator->GetOutputSource().GetOutput();
}

VectorType ApproximateValue::GetInput( const ros::Time& time ) const
{
	StampedFeatures features;
	if( !_receiver.ReadStream( time, features ) )
	{
		throw std::out_of_range( "ApproximateValue: Could not get input stream." );
	}
	return features.features;
}

percepto::Parameters::Ptr ApproximateValue::GetParameters() const
{
	return _approximatorParams;
}

ScalarFieldApproximator::Ptr 
ApproximateValue::CreateApproximatorModule() const
{
	if( _moduleType == "perceptron" )
	{
		PerceptronFunctionApproximator::Ptr net = 
			std::dynamic_pointer_cast<PerceptronFunctionApproximator>( _approximator );
		return std::make_shared<PerceptronFunctionApproximator>( *net );
	}
	else
	{
		throw std::invalid_argument( "ApproximateValue: Unknown network type: " + _moduleType );
	}
}

void ApproximateValue::ParamsCallback( const argus_msgs::FloatVectorStamped::ConstPtr& msg )
{
	_approximatorParams->SetParamsVec( GetVectorView( msg->values ) );
}

}