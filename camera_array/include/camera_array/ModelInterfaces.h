#pragma once

#include <vector>
#include <memory>

namespace camera_array
{

/*! \brief The transition function interface. Should be implemented by system models
 * that will be used by policies that evaluate actions or outcomes. */
template <typename State, typename Action>
class TransitionFunction
{
public:

	typedef std::shared_ptr<TransitionFunction> Ptr;
	
	TransitionFunction() {}
	
	/*! \brief Produce the outcome of applying the specified action to the state. */
	virtual State Transition( const State& state, const Action& action ) = 0;
};

/*! \brief The action generation interface. Should be implemented by system models
 * that will be used by policies that evaluate actions or outcomes. */
template <typename State, typename Action>
class ActionGenerator
{
public:

	typedef std::shared_ptr<ActionGenerator> Ptr;
	
	typedef std::vector<Action> ActionList;
	
	ActionGenerator() {}
	virtual ~ActionGenerator() {}

	/*! \brief Generate the set of valid actions from the given state. */	
	virtual ActionList GetActions( const State& state ) = 0;
};

/*! \brief The reward function interface. Should be implemented by system models
 * that will be used by policies that evaluate actions or outcomes. */
template <typename State, typename Action>
class RewardFunction
{
public:
	
	typedef std::shared_ptr<RewardFunction> Ptr;
	
	RewardFunction() {}
	virtual ~RewardFunction() {}
	
	/*! \brief Calculates the reward for being at the specified state and taking
	 * the specified action. */
	virtual double CalculateReward( const State& state, const Action& action ) = 0;
	
};

/*! \brief The policy interface. */
template <typename State, typename Action>
class DecisionPolicy
{
public:

	typedef std::shared_ptr<DecisionPolicy> Ptr;
	typedef typename ActionGenerator<State,Action>::ActionList ActionList;
	
	DecisionPolicy() {}
	virtual ~DecisionPolicy() {}
	
	/*! \brief Select an action from the set of actions for the given state. */
	virtual Action ChooseAction( const State& state, const ActionList& actions ) = 0;
	
};

}
