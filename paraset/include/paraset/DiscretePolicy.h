#pragma once

#include "relearn/RelearnCommon.h"

#include <ros/ros.h>
#include <iostream>

namespace argus
{

// Interface that wraps process of interfacing with parameter set services
// Selects from a set of discrete actions
// TODO Implement thread to call services?
class DiscretePolicy
{
public:

	DiscretePolicy();

	void Initialize( ros::NodeHandle& ph );

	unsigned int GetNumOutputs() const;

	// Return a vector denoting the number of settings for each output
	const std::vector<unsigned int>& GetNumSettings() const;
	unsigned int GetNumCombinations() const;

	// Set each output to the specified setting index
	void SetOutputIndices( const std::vector<unsigned int>& inds );

private:

	struct ParameterRegistration
	{
		std::string name;
		ros::ServiceClient setService;
		RuntimeParamType type;
		std::vector<RuntimeParam> values;
	};
	std::vector<ParameterRegistration> _parameters;
	std::vector<unsigned int> _numSettings;

	friend std::ostream& operator<<( std::ostream& os, const DiscretePolicy& policy );
};

std::ostream& operator<<( std::ostream& os, const argus::DiscretePolicy& policy );

}
