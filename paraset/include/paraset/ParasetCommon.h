#pragma once

#include "paraset/RuntimeParameter.h"

#include <boost/variant.hpp>
#include <boost/foreach.hpp>

namespace argus
{

typedef boost::variant<long, double, std::string, bool> RuntimeParam;

enum RuntimeParamType
{
	PARAM_INVALID = paraset::RuntimeParameter::PARAM_INVALID,
	PARAM_INTEGER = paraset::RuntimeParameter::PARAM_INTEGER,
	PARAM_FLOAT = paraset::RuntimeParameter::PARAM_FLOAT,
	PARAM_STRING = paraset::RuntimeParameter::PARAM_STRING,
	PARAM_BOOLEAN = paraset::RuntimeParameter::PARAM_BOOLEAN
};
RuntimeParamType StringToParamType( const std::string& s );
std::string ParamTypeToString( RuntimeParamType t );

template <template<typename,typename> class Container, typename Data, typename InAlloc>
Container<RuntimeParam, std::allocator<RuntimeParam>> 
ConvertToParamVariants( const Container<Data, InAlloc>& data )
{
	Container<RuntimeParam, std::allocator<RuntimeParam>> variants;
	variants.reserve( data.size() );
	BOOST_FOREACH( const Data& d, data )
	{
		variants.emplace_back( d );
	}
	return variants;
}

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

template <typename CType>
struct RuntimeParamTraits
{
	const static std::string name;
	const static RuntimeParamType type;
};

}