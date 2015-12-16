#pragma once

#include <memory>

namespace camera_array
{

/*! \brief Base distribution interface. */
template <typename X>
class Distribution
{
public:

	typedef std::shared_ptr<Distribution> Ptr;
	
	Distribution() {}
	virtual ~Distribution() {}
	
	virtual void SetMean( const X& x ) = 0;
	virtual X Sample() = 0;
	
};

}
