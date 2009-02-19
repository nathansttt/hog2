/** An abstract class for the step by step implementation of an
algorithm. In these algorithms, StepAlgorithm will be called so
as to expand a single node.*/
#ifndef GENERICSTEPALGORITHM_H
#define GENERICSTEPALGORITHM_H

#include <vector>
#include "SearchEnvironment.h"
#include "FPUtil.h"
#include "StatCollection.h"

template <class state, class action, class environment>
class GenericStepAlgorithm {

public:
	GenericStepAlgorithm() {};
	virtual ~GenericStepAlgorithm() {};

	/**
	From GenericSearchAlgorithm. This stuff should be put back into GenericSearchAlgorithm at some
	point, with some fixes. Specifically:

	1) GetNodesExpanded and GetNodesTouched should have return type double, like below
	2) StatCollection.h should be included



	Searches for path and returns it in desired format. Returns the following status information.

	Returns 1 if a solution is found.
	Returns 0 if no solution is found.
	Returns -1 if the StepAlgorithm is not active.
	Returns -2 if the expansion limit is reached
	Returns -3 if the touched limit is reached
	Returns -4 if the checked limit is reached

	Specific algorithms may include other statuses.
	**/
	virtual int GetPath(environment *env, state from, state to, std::vector<state> &path) = 0;
	virtual int GetPath(environment *env, state from, state to, std::vector<action> &path) = 0;
	virtual const char *GetName() = 0;

	virtual unsigned long long GetNodesExpanded() = 0; // number of nodes expanded
	virtual unsigned long long GetNodesTouched() = 0; // number of nodes generated

	/**
	Returns the number of nodes on which a goal test has been performed. Note, in many cases (like A*)
	this value will be equivalent to the number of nodes expanded.
	**/
	virtual unsigned long long GetNodesChecked() = 0;

	/**
	Sets limits for the various metrics of efficiency. Enter 0 for no limit.
	Returns true if setting the limit succeeds,
	and fails otherwise.
	**/
	virtual bool SetExpandedLimit(unsigned long long limit) = 0;
	virtual bool SetTouchedLimit(unsigned long long limit) = 0;
	virtual bool SetCheckedLimit(unsigned long long limit) = 0;

	virtual void LogFinalStats(StatCollection *stats) = 0;

	/**
	Initializes the GenericStepAlgorithm by inputting a SearchEnvironment, a start state, and a goal.
	All internal data members should be initialized by this method.
	**/
	virtual bool Initialize(environment *env, state start, state goal) = 0;

	/**
	Increments the algorithm by a single step. If a solution is found, returns the path
	as a vector of actions. Single step can be defined differently for different
	algorithms.

	Returns 1 if a solution is found.
	Returns 0 if no solution is found.
	Returns -1 if the StepAlgorithm is not active.
	Returns -2 if the expansion limit is reached
	Returns -3 if the touched limit is reached
	Returns -4 if the checked limit is reached

	Specific algorithms may include other statuses.
	**/
	virtual int StepAlgorithm(std::vector<action> &path) = 0;

	/**
	Increments the algorithm by a single step. If a solution is found, returns the path
	as a vector of actions. Single step can be defined differently for different
	algorithms.

	Returns 1 if a solution is found.
	Returns 0 if no solution is found.
	Returns -1 if the StepAlgorithm is not active.
	Returns -2 if the expansion limit is reached
	Returns -3 if the touched limit is reached
	Returns -4 if the checked limit is reached

	Specific algorithms may include other statuses.
	**/
	virtual int StepAlgorithm(std::vector<state> &path) = 0;

	virtual bool Is_Step_Active() = 0;
};
#endif
