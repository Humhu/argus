#include "fieldtrack/DimensionParser.h"
#include "argus_utils/geometry/PoseSE2.h"
#include "argus_utils/geometry/PoseSE3.h"

#include <boost/algorithm/string.hpp>

namespace argus
{

unsigned int twod_to_threed( unsigned int d )
{
	// NOTE Integer division rounding down
	unsigned int order = d / PoseSE2::TangentDimension;
	unsigned int ind = d - order * PoseSE2::TangentDimension;
	if( ind == 2 ) { ind = 5; } // 2D angle maps to 3D z angle
	return order * PoseSE3::TangentDimension + ind;
}

MatrixType promote_3d_matrix( bool twoDimensional, unsigned int order )
{
	unsigned int fullDim = PoseSE3::TangentDimension * (order + 1);
	if( !twoDimensional )
	{
		return MatrixType::Identity( fullDim, fullDim );
	}
	
	unsigned int dim = twoDimensional ? PoseSE2::TangentDimension
							: PoseSE3::TangentDimension;
	unsigned int inDim = dim * (order + 1);
	
	MatrixType p = MatrixType::Zero( fullDim, inDim );

	for( unsigned int i = 0; i < inDim; ++i )
	{
		p( twod_to_threed(i), i ) = 1;
	}
	return p;
}

unsigned int parse_dim_string( const std::string& s,
                               bool twoDimensional,
                               unsigned int maxOrder,
                               unsigned int minOrder )
{
	std::vector<std::string> splits;
	boost::split( splits, s, boost::is_any_of( "_" ), boost::token_compress_on );
	if( splits.size() != 3 )
	{
		throw std::invalid_argument( "String " + s + " invalid format" );
	}

	std::string typeStr = splits[0];
	std::string dimStr = splits[1];

	unsigned int order = std::stoi( splits[2] ); // NOTE Should really use stoul
	if( order < minOrder || order > maxOrder )
	{
		throw std::invalid_argument( "String " + s + " order exceeds order limits" );
	}
	order = order - minOrder;

	unsigned int typeInd, dimInd;
	if( typeStr == "pos" )
	{
		typeInd = 0;
	}
	else if( typeStr == "ori" )
	{
		typeInd = twoDimensional ? 0 : 3;
	}
	else
	{
		throw std::invalid_argument( "String " + s + " type invalid" );
	}

	if( dimStr == "x" )
	{
		dimInd = 0;
	}
	else if( dimStr == "y" )
	{
		dimInd = 1;
	}
	else if( dimStr == "z" )
	{
		dimInd = 2;
	}
	else
	{
		throw std::invalid_argument( "String " + s + " dim invalid" );
	}

	if( twoDimensional )
	{
		if( (typeStr == "pos" && dimStr == "z") ||
		    (typeStr == "ori" && dimStr == "x") ||
		    (typeStr == "ori" && dimStr == "y") )
		{
			throw std::invalid_argument( "String " + s + " invalid in 2D mode" );
		}
	}

	unsigned int baseDim = twoDimensional ? 3 : 6;

	return baseDim * order + typeInd + dimInd;
}

std::vector<unsigned int> parse_dim_string( const std::vector<std::string>& s,
                                            bool twoDimensional,
                                            unsigned int maxOrder,
                                            unsigned int minOrder )
{
	std::vector<unsigned int> ret;
	ret.reserve( s.size() );
	for( unsigned int i = 0; i < s.size(); ++i )
	{
		ret.push_back( parse_dim_string( s[i], twoDimensional, maxOrder, minOrder ) );
	}
	return ret;
}
}
