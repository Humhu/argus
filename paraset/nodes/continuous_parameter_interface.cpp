#include <ros/ros.h>
#include "paraset/ContinuousParamPolicy.h"
#include "argus_utils/utils/ParamUtils.h"
#include "percepto_msgs/SetParameters.h"

using namespace argus;
using namespace percepto;

class ContinuousParameterInterfaceNode
{
public:

	ContinuousParameterInterfaceNode( ros::NodeHandle& nh,
	                               ros::NodeHandle& ph )
	{
		_interface.Initialize( nh, ph );
		_setterServer = ph.advertiseService( "set_parameters", 
		                                     &ContinuousParameterInterfaceNode::SetterCallback,
		                                     this );
	}

private:

	ContinuousParamPolicy _interface;
	ros::ServiceServer _setterServer;

	bool SetterCallback( percepto_msgs::SetParameters::Request& req,
	                     percepto_msgs::SetParameters::Response& res )
	{
		VectorType out = GetVectorView( req.parameters );
		if( out.size() != _interface.GetNumOutputs() )
		{
			ROS_ERROR_STREAM( "Received " << out.size() << " parameters but interface takes " <<
			                  _interface.GetNumOutputs() );
			return false;
		}
		_interface.SetOutput( out );
		return true;
	}
};

int main( int argc, char** argv )
{
	ros::init( argc, argv, "continuous_parameter_interface" );

	ros::NodeHandle nh, ph( "~" );
	ContinuousParameterInterfaceNode cppn( nh, ph );
	ros::spin();
	return 0;
}