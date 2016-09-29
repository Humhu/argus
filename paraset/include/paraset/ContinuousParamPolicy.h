#pragma once

#include "poli/PoliInterfaces.h"
#include "argus_utils/utils/LinalgTypes.h"

#include <ros/ros.h>
#include <iostream>

namespace argus
{

// Provides an interface for a percepto continuous policy to interact with
// runtime parameters
class ContinuousParamPolicy
: public percepto::ContinuousPolicyInterface
{
public:

	ContinuousParamPolicy();

	// Reads in interfaced parameter properties
	void Initialize( ros::NodeHandle& nh, ros::NodeHandle& ph );

	// Get the number of parameters this policy wraps
	virtual unsigned int GetNumOutputs() const;

	// Get the ordered parameter names
	virtual std::vector<std::string> GetParameterNames() const;

	// Set each parameter to the specified parameter
	virtual void SetOutput( const VectorType& outputs );

	virtual const VectorType& GetLowerLimits() const;
	virtual const VectorType& GetUpperLimits() const;

private:

	VectorType _lowerLimits;
	VectorType _upperLimits;
	VectorType _scales;
	VectorType _offsets;

	struct ParameterRegistration
	{
		std::string name;
		double lowerLimit;
		double upperLimit;
		double scale;
		double offset;
		ros::ServiceClient setService;
	};
	std::vector<ParameterRegistration> _parameters;

	friend std::ostream& operator<<( std::ostream& os, const ContinuousParamPolicy& policy );
};

std::ostream& operator<<( std::ostream& os, const ContinuousParamPolicy& policy );

}