#pragma once

#include "paraset/RuntimeParameter.h"

#include <boost/variant.hpp>

namespace argus
{

typedef boost::variant<long, double, std::string, bool> RuntimeParam;

RuntimeParam MsgToParamVariant( const paraset::RuntimeParameter& msg );
paraset::RuntimeParameter ParamVariantToMsg( const RuntimeParam& var );

class ParamToMsgVisitor
: public boost::static_visitor<paraset::RuntimeParameter>
{
public:

	typedef paraset::RuntimeParameter MessageType;

	ParamToMsgVisitor();

	MessageType operator()( long value ) const;
	MessageType operator()( double value ) const;
	MessageType operator()( const std::string& value ) const;
	MessageType operator()( bool value ) const;
};

}