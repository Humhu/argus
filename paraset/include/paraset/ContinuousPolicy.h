#pragma once

#include "paraset/ParasetCommon.h"

#include <ros/ros.h>
#include <iostream>

namespace argus
{

// 
class ContinuousPolicy
{
public:

	ContinuousPolicy();

	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	// Get the number of parameters this policy wraps
	unsigned int GetNumOutputs() const;

	std::vector<std::string> GetParameterNames() const;

	// Set each parameter to the specified parameter
	void SetOutput( const VectorType& outputs );

	const VectorType& GetLowerLimits() const;
	const VectorType& GetUpperLimits() const;

private:

	struct ParameterRegistration
	{
		std::string name;
		ros::ServiceClient setService;
		RuntimeParamType type;
	};
	std::vector<ParameterRegistration> _parameters;

	VectorType _lowerActionLimits;
	VectorType _upperActionLimits;

	friend std::ostream& operator<<( std::ostream& os, const ContinuousPolicy& policy );
};

std::ostream& operator<<( std::ostream& os, const ContinuousPolicy& policy );

}