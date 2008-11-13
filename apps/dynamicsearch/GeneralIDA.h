#ifndef IDASTAR_H
#define IDASTAR_H

#include <iostream>
#include "SearchEnvironment.h"
#include "FPUtil.h"

using namespace std;

/*
Class for a general inheritable IDA star like algorithm. The
depth-first search component of the algorithm should not need
to be re-implemented. However, the to_expand algorithm is the
main part of the algorithm that may need to be altered, as
different IDA* like algorithms will determine whether a node
should or should not be expanded based on different criteria.

The GetPath algorithm is also expected to be re-implemented for
each variant as different updates will need to be performed
upon each iteration. The search_node algorithm can also be
re-implemented, but it is not suggested.

Note, the search can be modified in may ways. A bound can be
placed on the number of nodes expanded, generated, and checked.
As well, it is possible to force the search to expand the entire
iteration even after having found a solution.

Also have step by step version of IDA*. In this version, IDA* can be
asked to progress one node test (whether to expand or not) at a time.
This algorithm is similarly general and can be over ridden.
*/
template <class state, class action>
class GeneralIDA {
public:
	GeneralIDA() {
		bound_expanded = false;
		bound_generated = false;
		bound_checked = false;
		expand_full_iter = false;
		step_by_step_active = false;
	}

	virtual ~GeneralIDA() {}

	/**
	Finds a path from the node "from" to the node "to" in the search space using
	an IDA* like algorithm and stores the set of actions in thePath
	**/
	virtual void GetPath(SearchEnvironment<state, action> *env, state from, state to,
	             std::vector<action> &thePath);

	/** Get the number of nodes expanded (a node is expanded if the goal test is called) **/
	long GetNodesExpanded() { return nodesExpanded; }

	/** Get the number of nodes generated (if the successors or successor actions
	 of a node are successors, these are considered generated nodes)**/
	long GetNodesGenerated() { return nodesGenerated; }

	/** Get the number of nodes checked to be generated (a node is checked if search_node
	 is called on it, in which case to_expand is necessarily called on it except in a few
	 situations regarding expanding an entire iteration) **/
	long GetNodesChecked() {return nodesChecked; }

	/** returns the path cost of the last path found **/
	double GetPathCost() {return best_path_cost; }

	/** Sets a limit on the number of nodes to expand - input 0 for no limit **/
	void SetExpandedLimit(unsigned long limit);

	/** Sets a limit on the number of nodes to generate - input 0 for no limit **/
	void SetGeneratedLimit(unsigned long limit);

	/** Sets a limit on the number of nodes to check - input 0 for no limit **/
	void SetCheckedLimit(unsigned long limit);

	/** Sets whether the entire iteration is to be expanded or not **/
	void SetExpandFullIteration(bool ex) {expand_full_iter = ex;}

	/** Initializes the search to perform in a step by step manner for the given
	 environment, starting at the given initial state and the given goal
	 **/
	void initialize_step_by_step(SearchEnvironment<state, action> *env,
	                             state from, state to);
	/**
	Performs one step of the search. If a solution is found, it is stored in
	thePath. One step is defined as testing whether a node is to be expanded,
	and expanding if necessary. The return value defines the status.

	Returns 0 if no solution is found by the previous step.
	Returns 1 if a solution is found by that step and the search is to conclude.
	Returns 2 if ran out of nodes to expand
	Returns 3 if ran out of nodes to generate
	Returns 4 if ran out of nodes to check
	Returns -1 if a solution is found but the whole iteration has not finished searching.
	**/
	int move_one_step(std::vector<action> &thePath);

protected:
	/**
	Searches a node which involves testing whether the node should
	be expanded, performing the goal test, generating successors,
	and recursively calling the algorithm on successors.

	Returns 0 if no solution is found in subtree, 1 if a solution is
	found, and a number larger than 1 for other statuses
	ie.
	2 for ran out of nodes to expand
	3 for ran out of nodes to generate
	4 for rant out of nodes to check
	**/
	virtual int search_node(SearchEnvironment<state, action> *env,
	                   action forbiddenAction, state &currState,
	                   std::vector<action> &current_path, double g);

	state goal;
	double nextBound;
	double currentBound;

	std::vector<action> current_best_path;
	double best_path_cost;

	/**
	Checks if a node should be expanded based on its g and h value.
	Also responsible for updating nextBound if need be.
	**/
	virtual bool to_expand(double g, double h);

	// initializes search parameters
	virtual void initialize_search(SearchEnvironment<state, action> *, state, state);

	// initializes node counts
	virtual void update_node_counts();

	// initializes bounds
	virtual void update_bounds();

	unsigned long nodesExpanded, nodesGenerated, nodesChecked;
	unsigned long nodes_ex_iter, nodes_gen_iter, nodes_check_iter;
	unsigned long expanded_limit, generated_limit, checked_limit;

	bool bound_expanded;
	bool bound_generated;
	bool bound_checked;

	bool expand_full_iter;
	bool sol_found;

	std::vector<action> active_path;
	std::vector<action> inversion_stack;
	std::vector<double> g_stack;
	std::vector<double> h_stack;

	std::vector<std::vector <action> > depth_first_stack;

	state active_state;
	bool step_by_step_active;
	SearchEnvironment<state, action> *active_env;
};

template <class state, class action>
void GeneralIDA<state, action>::initialize_search(SearchEnvironment<state, action> *env, state from, state to) {
	nodesExpanded = 0;
	nodesGenerated = 0;
	nodesChecked = 0;

	current_best_path.resize(0);
	best_path_cost = -1.0;

	goal = to;

	nextBound = env->HCost(from, goal);
	currentBound = nextBound;

	sol_found = false;

	nodes_ex_iter = 0;
	nodes_gen_iter = 0;
	nodes_check_iter = 0;
}

template <class state, class action>
void GeneralIDA<state, action>::update_node_counts() {
	nodesExpanded += nodes_ex_iter;
	nodesGenerated += nodes_gen_iter;
	nodesChecked += nodes_check_iter;

	currentBound = nextBound;

	nodes_ex_iter = 0;
	nodes_gen_iter = 0;
	nodes_check_iter = 0;
}

template <class state, class action>
void GeneralIDA<state, action>::update_bounds() {
	currentBound = nextBound;
}

template <class state, class action>
void GeneralIDA<state, action>::GetPath(SearchEnvironment<state, action> *env,
                                     state from, state to,
                                     std::vector<action> &thePath)
{
	initialize_search(env, from, to);
	std::vector<action> current_path;

	std::vector<action> act;
	env->GetActions(from, act);
	int status = 0;

	step_by_step_active = false;

	while (status == 0)
	{
		printf("Starting iteration with bound %f\n", currentBound);
		status = search_node(env, act[0], from, current_path, 0);
		update_node_counts();
		update_bounds();
	}
	thePath = current_best_path;
}

template <class state, class action>
int GeneralIDA<state, action>::search_node(SearchEnvironment<state, action> *env,
                                           action forbiddenAction, state &currState,
                                           std::vector<action> &current_path, double g)
{
	// check if node bounds are applicable
	if(bound_expanded && nodesExpanded + nodes_ex_iter >= expanded_limit)
		return 2;
	if(bound_generated && nodesGenerated + nodes_gen_iter >= generated_limit)
		return 3;
	if(bound_checked && nodesChecked + nodes_check_iter >= checked_limit)
		return 4;

	double h = env->HCost(currState, goal);

	// use g + h to cut off areas of search where cannot find better solution
	if(expand_full_iter && sol_found && !fless(h + g, best_path_cost))
		return 0;

	if(!to_expand(g, h)) {
		return 0;
	}

	nodes_ex_iter++;

	if (env->GoalTest(currState, goal)) {
		current_best_path = current_path;
		best_path_cost = g;
		sol_found = true;
		return 1; // found goal
	}

	// generate applicable actions
	std::vector<action> actions;
	env->GetActions(currState, actions);

	nodes_gen_iter += actions.size();
	int depth = current_path.size();

	int my_status = 0;
	for (unsigned int x = 0; x < actions.size(); x++)
	{
		// if is inversion of last action
		if ((depth != 0) && (actions[x] == forbiddenAction))
			continue;

		// use g + h to cut off areas of search where cannot find better solution
		if(expand_full_iter && sol_found && !fless(h + g, best_path_cost))
			return my_status;

		nodes_check_iter++;

		current_path.push_back(actions[x]);

		double edgeCost = env->GCost(currState, actions[x]);
		env->ApplyAction(currState, actions[x]);
		env->InvertAction(actions[x]);

		// recursively search child
		int status = search_node(env, actions[x], currState, current_path, g + edgeCost);
		env->ApplyAction(currState, actions[x]);
		current_path.pop_back();

		// handle status
		if (status == 1) {
			if(expand_full_iter)
				my_status = 1;
			else
				return 1;
		}
		else if(status > 1) {
			return status;
		}
	}
	return my_status;
}

template <class state, class action>
void GeneralIDA<state, action>::SetExpandedLimit(unsigned long limit) {
	if(limit > 0) {
		bound_expanded = true; expanded_limit = limit;
	}
	else {
		bound_expanded = false;
	}

}

template <class state, class action>
void GeneralIDA<state, action>::SetGeneratedLimit(unsigned long limit) {
	if(limit > 0) {
		bound_generated = true; generated_limit = limit;
	}
	else {
		bound_generated = false;
	}

}

template <class state, class action>
void GeneralIDA<state, action>::SetCheckedLimit(unsigned long limit) {
	if(limit > 0) {
		bound_checked = true; checked_limit = limit;
	}
	else {
		bound_checked = false;
	}

}

template <class state, class action>
bool GeneralIDA<state, action>::to_expand(double g, double h) {

	if (fgreater(g+h, currentBound))
	{
		if(!fgreater(nextBound, currentBound) || fless(g + h, nextBound)) {
			nextBound = g + h;
		}
		return false;
	}

	return true;
}
#endif

template <class state, class action>
void GeneralIDA<state, action>::initialize_step_by_step(SearchEnvironment<state, action> *env, state from, state to) {
	step_by_step_active = true;

	active_state = from;
	goal = to;
	active_env = env;

	inversion_stack.resize(0);
	depth_first_stack.resize(0);
	g_stack.resize(0);
	h_stack.resize(0);

	initialize_search(env, from, to);

	std::vector<action> act;
	env->GetActions(from, act);

	nodes_gen_iter += act.size();
	nodes_ex_iter += 1; // since opening expansion done here

	std::vector<action> stack;

	while(act.size() > 0) {
		stack.push_back(act.back());
		act.pop_back();
	}
	depth_first_stack.push_back(stack);
	printf("Starting iteration with bound %f\n", currentBound);
}

template <class state, class action>
int GeneralIDA<state, action>::move_one_step(std::vector<action> &thePath) {

	if(!step_by_step_active) // if can't use step by step at this time
		return 5;

	// check if node bounds are applicable
	if(bound_expanded && nodesExpanded + nodes_ex_iter >= expanded_limit) {
		update_node_counts();
		step_by_step_active = false;
		if(sol_found) {
			thePath = current_best_path;
		}
		return 2;
	}
	if(bound_generated && nodesGenerated + nodes_gen_iter >= generated_limit) {
		update_node_counts();
		step_by_step_active = false;
		if(sol_found) {
			thePath = current_best_path;
		}
		return 3;
	}
	if(bound_checked && nodesChecked + nodes_check_iter >= checked_limit) {
		update_node_counts();
		step_by_step_active = false;
		if(sol_found) {
			thePath = current_best_path;
		}
		return 4;
	}

	if(depth_first_stack.size() == 0) {

		update_node_counts();
		update_bounds();

		if(sol_found) { // if have already found a goal
			step_by_step_active = false;
			thePath = current_best_path;
			return 1;
		}

		// otherwise are starting new iteration
		std::vector<action> act;
		active_env->GetActions(active_state, act);

		std::vector<action> stack;

		while(act.size() > 0) {
			stack.push_back(act.back());
			act.pop_back();
		}

		depth_first_stack.push_back(stack);

		printf("Starting iteration with bound %f\n", currentBound);
		nodes_gen_iter += stack.size();
		nodes_ex_iter += 1; // since opening expansion done here
	}

	assert(g_stack.size() == depth_first_stack.size() - 1);
	assert(h_stack.size() == depth_first_stack.size() - 1);
	assert(inversion_stack.size() == depth_first_stack.size() - 1);
	assert(active_path.size() == depth_first_stack.size() - 1);

	/* For Debugging Purposes
	for(unsigned i = 0; i < active_path.size(); i++)
		cout <<  " ";
	cout << active_state << endl << "STACK: ";

	for(int i = 0; i < depth_first_stack.size(); i++) {
		for(unsigned j = 0; j < (depth_first_stack[i]).size(); j++) {
			cout << (depth_first_stack[i])[j] << " ";
		}
		cout << ", ";
	}
	cout << endl << "Inversion Stack: ";

	for(unsigned i = 0; i < inversion_stack.size(); i++)
		cout << inversion_stack[i] << " ";

	cout << endl << "Current Path: ";
	for(unsigned i = 0; i < active_path.size(); i++)
		cout << active_path[i] << " ";
	cout << endl;
	*/

	// get next action
	action to_apply = (depth_first_stack.back()).back();
	active_path.push_back(to_apply);

	// get new state info
	double edgeCost = active_env->GCost(active_state, to_apply);
	double g = edgeCost;

	if(g_stack.size() != 0)
		g += g_stack.back();

	active_env->ApplyAction(active_state, to_apply);

	double h = active_env->HCost(active_state, goal);

	nodes_check_iter++;

	// check if node should be expanded (or if should cut off due to better path found thus far
	if((!expand_full_iter || !sol_found || fless(g + h, best_path_cost))
	   && to_expand(g, h)) {
		nodes_ex_iter++;

		// test if node is a goal
		if (active_env->GoalTest(active_state, goal)) {
			current_best_path = active_path;
			best_path_cost = g;
			sol_found = true;

			if(!expand_full_iter) {
				depth_first_stack.resize(0);
				update_node_counts();
				step_by_step_active = false;
				thePath = current_best_path;
				return 1; // found goal
			}

			// if expanding full iteration
			// remove last action from path and stack
			active_path.pop_back();
			(depth_first_stack.back()).pop_back();

			// undo last action
			active_env->InvertAction(to_apply);
			active_env->ApplyAction(active_state, to_apply);
		}
		else {

			// get applicable actions
			std::vector<action> new_actions;
			active_env->GetActions(active_state, new_actions);

			nodes_gen_iter += new_actions.size();

			// re-order them
			std::vector<action> stack;

			while(new_actions.size() > 0) {
				stack.push_back(new_actions.back());
				new_actions.pop_back();
			}

			// update stacks
			depth_first_stack.push_back(stack);
			g_stack.push_back(g);
			h_stack.push_back(h);

			active_env->InvertAction(to_apply);
			inversion_stack.push_back(to_apply);

		}
	}
	else {
		// remove last action from path and stack
		active_path.pop_back();
		(depth_first_stack.back()).pop_back();

		// undo last action
		active_env->InvertAction(to_apply);
		active_env->ApplyAction(active_state, to_apply);
	}

	// eliminate returning to previous state
	if(inversion_stack.size() > 0 &&
	   (depth_first_stack.back()).back() == inversion_stack.back()) {
		   (depth_first_stack.back()).pop_back();
	   }
		// if is not last first level of stack and top level is empty
	while(depth_first_stack.size() > 1 &&
	      ((depth_first_stack.back()).size() == 0) ||
	      (expand_full_iter && sol_found &&
	       !fless(g_stack.back() + h_stack.back(), best_path_cost))) {
		depth_first_stack.pop_back(); // remove empty level
		(depth_first_stack.back()).pop_back(); // remove explored action

			// fix stacks
		active_path.pop_back();
		g_stack.pop_back();
		h_stack.pop_back();

			// undo action
		active_env->ApplyAction(active_state, inversion_stack.back());
		inversion_stack.pop_back();

			// eliminate returning to previous state
		if(inversion_stack.size() > 0 &&
		   (depth_first_stack.back()).back() == inversion_stack.back()) {
			   (depth_first_stack.back()).pop_back();
		   }
	}

		// if stack is finished
	if(depth_first_stack.size() == 1 && (depth_first_stack.back()).size() == 0) {
		depth_first_stack.pop_back();
	}
	if(sol_found)
		return -1;
	return 0;
}
