#include "paraset/ApproximateValue.h"
#include "paraset/RewardStamped.h"

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

	_outputPub = ph.advertise<paraset::RewardStamped>( "output", 0 );
}

// TODO Move this to some common class
void ApproximateValue::Publish( const ParamAction& x ) const
{
	paraset::RewardStamped msg;
	msg.header.stamp = x.time;
	msg.reward = Evaluate( x );
	_outputPub.publish( msg );
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
	_approximatorInput.SetOutput( features.features );
	_approximatorInput.Foreprop();
	_approximator->Foreprop();
	return _approximator->GetOutputSource().GetOutput();
}

ScalarFieldApproximator::Ptr 
ApproximateValue::GetApproximatorModule() const
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

}