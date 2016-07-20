#pragma once

#include <ros/ros.h>
#include <Eigen/Dense>

#include "argus_utils/synchronization/SynchronizationTypes.h"

#include "paraset/SetRuntimeParameter.h"
#include "paraset/GetParameterInfo.h"

#include <deque>

namespace argus
{

template <typename ParamType>
class ParameterManager
{
public:

	ParameterManager( ros::NodeHandle& nodeHandle, const std::string& paramName )
	: _name( paramName ), _currentIndex( 0 )
	{
		_setServer = nodeHandle.advertiseService( "set_" + paramName, 
		                                          &ParameterManager<ParamType>::SetParameterCallback,
		                                          this );
		_infoServer = nodeHandle.advertiseService( "get_" + paramName + "_info",
		                                           &ParameterManager<ParamType>::GetInfoCallback,
		                                           this );
	}

	void AddSetting( const ParamType& param, const std::string& description )
	{
		_settings.emplace_back( param, description );
	}

	void SetIndex( unsigned int ind )
	{
		WriteLock lock( _mutex );
		if( ind >= _settings.size() )
		{
			throw std::runtime_error( "Setting index " + std::to_string( ind ) +
                          " for parameter " + _name + " exceeds number of settings " +
                          std::to_string( _settings.size() ) );
		}
		_currentIndex = ind;
	}

	ParamType GetValue() const
	{
		ReadLock lock( _mutex );
		if( _currentIndex >= _settings.size() )
		{
			throw std::runtime_error( "Current index " + std::to_string( _currentIndex ) +
			                          " for parameter " + _name + " exceeds number of settings " +
			                          std::to_string( _settings.size() ) );
		}
		return _settings[_currentIndex].value;
	}

private:

	struct Setting
	{
		// Just in case parameters are fixed-size Eigen matrices
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW

		ParamType value;
		std::string description;

		Setting( const ParamType& p, const std::string& d )
		: value( p ), description( d ) {}
	};

	mutable Mutex _mutex;

	std::string _name;
	unsigned int _currentIndex;
	std::deque<Setting> _settings;

	ros::ServiceServer _setServer;
	ros::ServiceServer _infoServer;

	bool SetParameterCallback( paraset::SetRuntimeParameter::Request& req,
	                           paraset::SetRuntimeParameter::Response& res )
	{
		WriteLock lock( _mutex );
		if( req.setting_ind >= _settings.size() ) { return false; }
		_currentIndex = req.setting_ind;
		return true;
	}

	bool GetInfoCallback( paraset::GetParameterInfo::Request& req,
	                      paraset::GetParameterInfo::Response& res )
	{
		ReadLock lock( _mutex );
		res.num_settings = _settings.size();
		res.descriptions.reserve( _settings.size() );
		for( unsigned int i = 0; i < _settings.size(); i++ )
		{
			res.descriptions.emplace_back( _settings[i].description );
		}
		return true;
	}

};

}