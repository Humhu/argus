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

class ParamPrintVisitor
: public boost::static_visitor<std::string>
{
public:

	typedef paraset::RuntimeParameter MessageType;

	ParamPrintVisitor();

	std::string operator()( long value ) const;
	std::string operator()( double value ) const;
	std::string operator()( const std::string& value ) const;
	std::string operator()( bool value ) const;
};
// NOTE Causes issues with ambiguous print calls on ParameterManager...
// std::ostream& operator<<( std::ostream& os, const RuntimeParam& param );
std::string ParamVariantToString( const RuntimeParam& var );

class ParamEqualityVisitor
: public boost::static_visitor<bool>
{
public:

	ParamEqualityVisitor();

	template <typename T, typename U>
	bool operator()( const T& lhs, const U& rhs ) const
	{
		return false;
	}

	template <typename T>
	bool operator()( const T& lhs, const T& rhs ) const
	{
		return lhs == rhs;
	}
};
bool operator==( const RuntimeParam& lhs, const RuntimeParam& rhs );
bool operator !=( const RuntimeParam& lhs, const RuntimeParam& rhs );

template <typename CType>
struct RuntimeParamTraits
{
	const static std::string name;
	const static RuntimeParamType type;
};

}