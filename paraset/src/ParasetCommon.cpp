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
		case paraset::RuntimeParameter::PARAM_NUMERIC:
		{
			return RuntimeParam( msg.numeric_value );
		}
		case paraset::RuntimeParameter::PARAM_STRING:
		{
			return RuntimeParam( msg.string_value );
		}
		case paraset::RuntimeParameter::PARAM_BOOLEAN:
		{
			bool val = msg.boolean_value;
			return RuntimeParam( val );
		}
		default:
		{
			throw std::runtime_error( "MsgToParamVariant: Unknown parameter type." );
		}
	} // end switch
}

RuntimeParamType StringToParamType( const std::string& s )
{
	if( s == "invalid" )
	{
		return PARAM_INVALID;
	}
	else if( s == "numeric" )
	{
		return PARAM_NUMERIC;
	}
	else if( s == "string" )
	{
		return PARAM_STRING;
	}
	else if( s == "boolean" )
	{
		return PARAM_BOOLEAN;
	}
	else
	{
		throw std::runtime_error( "StringToParamType: Unknown param type: " + s );
	}
}

std::string ParamTypeToString( RuntimeParamType t )
{
	switch( t )
	{
		case PARAM_INVALID:
			return "invalid";
		case PARAM_NUMERIC:
			return "numeric";
		case PARAM_STRING:
			return "string";
		case PARAM_BOOLEAN:
			return "boolean";
		default:
			throw std::runtime_error( "ParamTypeToString: Unknown param type received." );
	}
}

paraset::RuntimeParameter ParamVariantToMsg( const RuntimeParam& var )
{
	return boost::apply_visitor( ParamToMsgVisitor(), var );
}

ParamToMsgVisitor::ParamToMsgVisitor() {}

ParamToMsgVisitor::MessageType
ParamToMsgVisitor::operator()( double value ) const
{
	ParamToMsgVisitor::MessageType msg;
	msg.type = ParamToMsgVisitor::MessageType::PARAM_NUMERIC;
	msg.numeric_value = value;
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
	msg.type = ParamToMsgVisitor::MessageType::PARAM_BOOLEAN;
	msg.boolean_value = value;
	return msg;
}

ParamPrintVisitor::ParamPrintVisitor() {}

std::string ParamPrintVisitor::operator()( double value ) const
{
	return "float: " + std::to_string( value );
}

std::string ParamPrintVisitor::operator()( const std::string& value ) const
{
	return "string: " + value;
}

std::string ParamPrintVisitor::operator()( bool value ) const
{
	return "boolean: " + std::to_string( value );
}

std::string ParamVariantToString( const RuntimeParam& var )
{
	return boost::apply_visitor( ParamPrintVisitor(), var );
}

// std::ostream& operator<<( std::ostream& os, const RuntimeParam& param )
// {
// 	os << boost::apply_visitor( ParamPrintVisitor(), param );
// 	return os;
// }

ParamEqualityVisitor::ParamEqualityVisitor() {}

bool operator==( const RuntimeParam& lhs, const RuntimeParam& rhs )
{
	return boost::apply_visitor( ParamEqualityVisitor(), lhs, rhs );
}

bool operator!=( const RuntimeParam& lhs, const RuntimeParam& rhs )
{
	return !boost::apply_visitor( ParamEqualityVisitor(), lhs, rhs );
}

template <>
const std::string RuntimeParamTraits<double>::name = "numeric";
template <>
const RuntimeParamType RuntimeParamTraits<double>::type = PARAM_NUMERIC;

template <>
const std::string RuntimeParamTraits<std::string>::name = "string";
template <>
const RuntimeParamType RuntimeParamTraits<std::string>::type = PARAM_STRING;

template <>
const std::string RuntimeParamTraits<bool>::name = "boolean";
template <>
const RuntimeParamType RuntimeParamTraits<bool>::type = PARAM_BOOLEAN;

}