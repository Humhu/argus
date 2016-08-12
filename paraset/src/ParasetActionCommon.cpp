#include "paraset/ParasetActionCommon.h"
#include "argus_utils/utils/MatrixUtils.h"

namespace argus
{

ParamAction::ParamAction() {}

ParamAction::ParamAction( const ros::Time& t, const VectorType& in )
: time( t ), input( in ) {}

ContinuousParamAction::ContinuousParamAction() {}

ContinuousParamAction::ContinuousParamAction( const MsgType& msg )
: ParamAction( msg.header.stamp, GetVectorView( msg.input ) ),
  output( GetVectorView( msg.output ) ) {}

ContinuousParamAction::ContinuousParamAction( const ros::Time& t,
                                              const VectorType& in,
                                              const VectorType& out )
: ParamAction( t, in ), output( out ) {}

ContinuousParamAction::MsgType ContinuousParamAction::ToMsg() const
{
	MsgType msg;
	msg.header.stamp = time;
	SerializeMatrix( input, msg.input );
	SerializeMatrix( output, msg.output );
	return msg;
}

DiscreteParamAction::DiscreteParamAction() {}

DiscreteParamAction::DiscreteParamAction( const MsgType& msg )
: ParamAction( msg.header.stamp, GetVectorView( msg.input ) ),
  index( msg.index ) {}

DiscreteParamAction::DiscreteParamAction( const ros::Time& t,
                                          const VectorType& in,
                                          unsigned int ind )
: ParamAction( t, in ), index( ind ) {}

DiscreteParamAction::MsgType DiscreteParamAction::ToMsg() const
{
	MsgType msg;
	msg.header.stamp = time;
	SerializeMatrix( input, msg.input );
	msg.index = index;
	return msg;
}

}