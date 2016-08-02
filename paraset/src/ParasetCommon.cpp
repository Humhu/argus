#include "paraset/ParasetCommon.h"

namespace argus
{

RuntimeParam MsgToParamVariant( const paraset::RuntimeParameter& msg )
{
	switch( msg.type )
	{
		case paraset::RuntimeParameter::PARAM_INVALID:
		{
			throw std::runtime_error( "MsgToParamVariant: Cannot parse invalid param message." );
		}
		case paraset::RuntimeParameter::PARAM_INT:
		{
			return RuntimeParam( msg.int_value );
		}
		case paraset::RuntimeParameter::PARAM_FLOAT:
		{
			return RuntimeParam( msg.float_value );
		}
		case paraset::RuntimeParameter::PARAM_STRING:
		{
			return RuntimeParam( msg.string_value );
		}
		case paraset::RuntimeParameter::PARAM_BOOL:
		{
			bool val = msg.bool_value;
			return RuntimeParam( val );
		}
		default:
		{
			throw std::runtime_error( "MsgToParamVariant: Unknown parameter type." );
		}
	} // end switch
}

paraset::RuntimeParameter ParamVariantToMsg( const RuntimeParam& var )
{
	return boost::apply_visitor( ParamToMsgVisitor(), var );
}

ParamToMsgVisitor::ParamToMsgVisitor() {}

ParamToMsgVisitor::MessageType
ParamToMsgVisitor::operator()( long value ) const
{
	ParamToMsgVisitor::MessageType msg;
	msg.type = ParamToMsgVisitor::MessageType::PARAM_INT;
	msg.int_value = value;
	return msg;
}

ParamToMsgVisitor::MessageType
ParamToMsgVisitor::operator()( double value ) const
{
	ParamToMsgVisitor::MessageType msg;
	msg.type = ParamToMsgVisitor::MessageType::PARAM_FLOAT;
	msg.float_value = value;
	return msg;
}

ParamToMsgVisitor::MessageType
ParamToMsgVisitor::operator()( const std::string& value ) const
{
	ParamToMsgVisitor::MessageType msg;
	msg.type = ParamToMsgVisitor::MessageType::PARAM_STRING;
	msg.string_value = value;
	return msg;
}

ParamToMsgVisitor::MessageType
ParamToMsgVisitor::operator()( bool value ) const
{
	ParamToMsgVisitor::MessageType msg;
	msg.type = ParamToMsgVisitor::MessageType::PARAM_BOOL;
	msg.bool_value = value;
	return msg;
}

}