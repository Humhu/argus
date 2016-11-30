#pragma once

#include "paraset/RuntimeParameter.h"

#include "argus_utils/utils/ParamUtils.h"

#include <boost/variant.hpp>
#include <boost/foreach.hpp>

namespace argus
{

/*! \brief The C++-side representation of a runtime parameter. Can take on one
 * of the constituent types. */
typedef boost::variant<double, std::string, bool> RuntimeParam;

/*! \brief Enumeration that mirrors the ROS service constants. */
enum RuntimeParamType
{
	PARAM_INVALID = paraset::RuntimeParameter::PARAM_INVALID,
	PARAM_NUMERIC = paraset::RuntimeParameter::PARAM_NUMERIC,
	PARAM_STRING = paraset::RuntimeParameter::PARAM_STRING,
	PARAM_BOOLEAN = paraset::RuntimeParameter::PARAM_BOOLEAN
};
RuntimeParamType StringToParamType( const std::string& s );
std::string ParamTypeToString( RuntimeParamType t );

/*! \brief Traits mapping from C++ types to their corresponding enum and name. */
template <typename CType>
struct RuntimeParamTraits
{
	const static std::string name;
	const static RuntimeParamType type;
};

/*! \brief Convert STL-compatible containers of types into containers of parameter variants. */
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

/*! \brief Conversion to and from the ROS message type and the variant type. */
RuntimeParam MsgToParamVariant( const paraset::RuntimeParameter& msg );
paraset::RuntimeParameter ParamVariantToMsg( const RuntimeParam& var );

/*! \brief Visitor for converting variants to ROS messages. */
class ParamToMsgVisitor
: public boost::static_visitor<paraset::RuntimeParameter>
{
public:

	typedef paraset::RuntimeParameter MessageType;

	ParamToMsgVisitor();

	MessageType operator()( double value ) const;
	MessageType operator()( const std::string& value ) const;
	MessageType operator()( bool value ) const;
};

/*! \brief Visitor for turning a variant into a string. */
class ParamPrintVisitor
: public boost::static_visitor<std::string>
{
public:

	typedef paraset::RuntimeParameter MessageType;

	ParamPrintVisitor();

	std::string operator()( double value ) const;
	std::string operator()( const std::string& value ) const;
	std::string operator()( bool value ) const;
};
// NOTE Causes issues with ambiguous print calls on ParameterManager...
// std::ostream& operator<<( std::ostream& os, const RuntimeParam& param );
std::string ParamVariantToString( const RuntimeParam& var );

/*! \brief Visitor for checking equality of two variants. */
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

}