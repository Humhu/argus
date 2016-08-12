#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>

#include "argus_utils/synchronization/SynchronizationTypes.h"

#include "paraset/ParasetCommon.h"
#include "paraset/RuntimeParameter.h"
#include "paraset/SetRuntimeParameter.h"
#include "paraset/GetParameterInfo.h"
#include "paraset/ParamChecks.hpp"

#include <memory>
#include <deque>
#include <boost/foreach.hpp>
#include <sstream>

namespace argus
{

// TODO Introduce better support for booleans?
template <typename T>
class ParameterManager
{
public:

	ParameterManager() : _name("") {}

	// TODO Have initialval set after checks are added?
	void Initialize( ros::NodeHandle& nodeHandle,
	                 const T& initialVal,
	                 const std::string& name,
	                 const std::string& description )
	{
		WriteLock lock( _mutex );
		_name = name;
		_description = description;
		_currentValue = initialVal;
		ROS_INFO_STREAM( "Initializing runtime parameter: " << _name << std::endl <<
		                 "\tDescription: " << description << std::endl <<
		                 "\tInitial value: " << initialVal );
		_setServer = nodeHandle.advertiseService( "set_" + _name, 
		                                          &ParameterManager<T>::SetParameterCallback,
		                                          this );
		_infoServer = nodeHandle.advertiseService( "get_" + _name + "_info",
		                                           &ParameterManager<T>::GetInfoCallback,
		                                           this );
	}

	template <template<class> class Check, typename... Args >
	void AddCheck( Args&&... args )
	{
		WriteLock lock( _mutex );
		Validate();
		CheckPtr c = std::make_shared<Check<T>>( std::forward<Args>( args )... );
		_checks.push_back( c );
		ROS_INFO_STREAM( "Runtime parameter: " << _name << " added check: " << c->GetDescription() );
	}

	T GetValue() const
	{
		ReadLock lock( _mutex );
		Validate();
		return _currentValue;
	}

	operator T() const
	{
		return GetValue();
	}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

private:

	typedef typename ParameterCheck<T>::Ptr CheckPtr;

	std::string _name;
	std::string _description;

	ros::ServiceServer _setServer;
	ros::ServiceServer _infoServer;

	mutable Mutex _mutex;
	T _currentValue;
	std::deque<CheckPtr> _checks;

	virtual bool ReadVariant( const RuntimeParam& var )
	{
		try
		{
			WriteLock lock( _mutex );
			T val = boost::get<T>( var );
			BOOST_FOREACH( const CheckPtr& check, _checks )
			{
				val = check->Project( val );
			}
			_currentValue = val;
		}
		catch( boost::bad_get& e )
		{
			// This means that the type conversion failed
			ROS_WARN_STREAM( "Improper parameter type for " + _name );
			return false;
		}
		return true;
	}


	bool SetParameterCallback( paraset::SetRuntimeParameter::Request& req,
	                           paraset::SetRuntimeParameter::Response& res )
	{
		RuntimeParam var = MsgToParamVariant( req.param );
		if( !ReadVariant( var ) ) { return false; }
		var = _currentValue;
		res.actual = ParamVariantToMsg( var );
		return true;
	}

	bool GetInfoCallback( paraset::GetParameterInfo::Request& req,
	                      paraset::GetParameterInfo::Response& res )
	{
		ReadLock lock( _mutex );
		std::stringstream ss;
		ss << "Description: " << _description << std::endl;
		ss << "Type: " << RuntimeParamTraits<T>::name << std::endl;
		ss << "Constraints:";
		BOOST_FOREACH( const CheckPtr& check, _checks )
		{
			ss << " " << check->GetDescription();
		}

		res.description = ss.str();
		return true;
	}

	void Validate() const
	{
		if( _name.empty() )
		{
			throw std::runtime_error( "Parameter manager is uninitialized." );
		}
	}

};

typedef ParameterManager<double> NumericParam;
typedef ParameterManager<std::string> StringParam;
typedef ParameterManager<bool> BooleanParam;

}