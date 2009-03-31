#ifndef BULB_H
#define BULB_H
#include "StatCollection.h"
#include "GenericStepAlgorithm.h"
#include "BeamNode.h"
#include <assert.h>
#include <iostream>
#include <ext/hash_map>
#define NO_CHILDREN -100

template <class state, class action, class environment>
class GeneralBulb : public GenericStepAlgorithm<state, action, environment> {
public:
	GeneralBulb() {
		step_by_step_active = false;
		beam_size = 1;
		memory_limit = 50;
		bound_touched = bound_checked = bound_expanded = false;
		g_weight = 1.0;
		h_weight = 1.0;

		best_path_cost = -1.0;
		debug = false;
	}
	int GetPath(environment *env, state from, state to, std::vector<state> &path);
	int GetPath(environment *env, state from, state to, std::vector<action> &path){return 0;}

	uint64_t GetNodesExpanded(){return nodes_expanded;}
	uint64_t GetNodesTouched(){return nodes_touched;}
	uint64_t GetNodesChecked(){return nodes_checked;}

	bool SetExpandedLimit(uint64_t limit);
	bool SetTouchedLimit(uint64_t limit);
	bool SetCheckedLimit(uint64_t limit);

	void LogFinalStats(StatCollection *stats){}

	bool Initialize(environment *env, state start, state goal){return true;}
	int StepAlgorithm(std::vector<action> &path){return 0;}
	int StepAlgorithm(std::vector<state> &path){return 0;}

	bool Is_Step_Active(){return step_by_step_active;}

	const char *GetName() {return "General Bulb";}

	/**
	Returns the cost for this state based on the g and h values.
	**/
	virtual double get_cost(double g, double h);

	/**
	Prints information on all items in the beams. For debugging purposes
	**/
	void print_beams(bool last_only);

	/** returns the path cost of the last path found **/
	double GetPathCost() {return best_path_cost; }

	/** Changes the size of the beam. Default setting is 1. Returns true if succeeds, false otherwise. **/
	bool Change_Beam_Size(unsigned new_beam_size) {
		if(new_beam_size > 0) { beam_size = new_beam_size; return true; }
		return false;
	}

	/** Changes memory limit for beam search. That is, the number of nodes that can be in the beam
	 at one time. Default is set at 50.**/
	void Change_Memory_Limit(unsigned new_limit) {memory_limit = new_limit;}

	/**
	Returns the value of the applicable weight
	**/
	double Get_H_Weight() { return h_weight;}
	double Get_G_Weight() { return g_weight;}

	/**
	Changes the weights. Note, if the problem is constant cost, changing the weights
	does nothing.
	**/
	int Change_Weights(double h_g,double h_w) {
		if(!fless (h_g, 0.0) && !fless(h_w, 0.0)) {
			h_weight = h_w;
			g_weight = h_g;
			return 1;
		}
		return 0;
	}

protected:
	std::vector<std::vector <BeamNode<state> > > beams; // beam construct
	__gnu_cxx::hash_map<uint64_t, unsigned, Hash64> nodes_in_beam; // nodes in beams not in current layer
	__gnu_cxx::hash_map<uint64_t, unsigned, Hash64> nodes_in_current_layer; // nodes in beams not in current layer
	bool step_by_step_active;

	unsigned beam_size;
	unsigned memory_limit;
	uint64_t nodes_checked, nodes_expanded, nodes_touched;
	uint64_t expanded_limit, checked_limit, touched_limit;
	uint64_t nodes_ex_iter, nodes_touch_iter, nodes_check_iter;

	bool bound_expanded;
	bool bound_touched;
	bool bound_checked;

	double g_weight;
	double h_weight;

	void prepare_vars_for_search();
	/**
	Main recursive function of bulb. Takes in a number of discrepancies to use, the search environment, and a goal.
	Returns values based on the main status values of a GenericStepAlgorithm
	**/
	int BulbProbe(environment *env, unsigned discrepancies, state &goal);

	/**
	Generates all the unique successors of the last level in beam and stores them in successors. Returns 1 if
	the goal is found amongst the successors, in which case successors is only set to the goal node.
	Returns the status values of GenericStepAlgorithm if any of the limits are hit.
	**/
	int generate_successors(environment *env, std::vector<BeamNode<state> > &successors, state &goal);

	/**
	Returns the desired slice of the successors. If first_slice is set as true, returns the first slice. If
	first_slice is set to false and index is smaller than the beam_size, will return the second slice. Otherwise,
	it returns the slice starting at the given index.
	Index is then set as the beginning index of the next slice to return.
	**/
	int next_slice(environment *env, std::vector<BeamNode<state> > &slice, bool first_slice, unsigned &index, state &goal);

	/**
	Extracts the goal from the beams.  Assumes the first beam contains only the start state, and the last beam
	contains only the goal.
	**/
	void extract_goal(std::vector<state> &state_path);

	double best_path_cost;

	unsigned max_depth;

	bool debug;

	std::vector<uint64_t> iter_checked;
	std::vector<uint64_t> iter_touched;
	std::vector<uint64_t> iter_expanded;

	void update_node_counts(); // updates the node counts at the end of each iteration
	// where an iteration is whenever the discrepancies must be increased.
};

/** Calculates f-cost in standard way **/
template <class state, class action, class environment>
double GeneralBulb<state, action, environment>::get_cost(double g, double h) {
	return g_weight*g + h_weight*h;
}

template <class state, class action, class environment>
void GeneralBulb<state, action, environment>::prepare_vars_for_search() {
	nodes_checked = 0;
	nodes_expanded = 0;
	nodes_touched = 0;

	best_path_cost = -1.0;
	beams.clear();
	nodes_in_beam.clear();

	best_path_cost = -1.0;
	iter_expanded.clear();
	iter_touched.clear();
	iter_checked.clear();
}

template <class state, class action, class environment>
void GeneralBulb<state, action, environment>::update_node_counts() {
	nodes_expanded += nodes_ex_iter;
	nodes_touched += nodes_touch_iter;
	nodes_checked += nodes_check_iter;

	iter_checked.push_back(nodes_check_iter);
	iter_touched.push_back(nodes_touch_iter);
	iter_expanded.push_back(nodes_ex_iter);
}

template <class state, class action, class environment>
int GeneralBulb<state, action, environment>::GetPath(environment *env, state from, state to, std::vector<state> &path) {
	prepare_vars_for_search();

	double h_value;

	if(env->IsGoalStored()) {
		h_value = env->HCost(from);
	}
	else {
		h_value = env->HCost(from, to);
	}

	double initial_cost = get_cost(0.0, h_value);
	uint64_t initial_key = env->GetStateHash(from);

	// constructs initial beamnode
	BeamNode<state> start_node(from, 0.0, h_value, initial_cost, 0,initial_key);

	std::vector<BeamNode<state> > first_layer;
	first_layer.push_back(start_node);
	beams.push_back(first_layer);

	nodes_in_beam[initial_key] = 0;
	int status = 0;

	unsigned discrepancies = 0;

	bool first = true;

	while(status == 0) {
		max_depth = 0;
		nodes_ex_iter = nodes_touch_iter = nodes_check_iter = 0;

		if(first){
			nodes_touch_iter = 1;
			first = false;
		}

		assert(beams.size() == 1);
		assert(nodes_in_beam.size() == 1);

		status = BulbProbe(env, discrepancies, to);

		update_node_counts();

		if(max_depth <= discrepancies) { // if have explored entire space of possible solutions
			status = NO_SOLUTION;
		}

		if(status == 0)
			discrepancies++;

		if(debug)
			printf("Increasing Discrepancies\n");
	}

	if(status == 1) {
		extract_goal(path);
	}
	return status;

}

template <class state, class action, class environment>
int GeneralBulb<state, action, environment>::BulbProbe(environment *env, unsigned discrepancies, state &goal) {

	if(beams.size() - 1 > max_depth) {
		max_depth = beams.size() - 1;
	}
	std::vector<BeamNode<state> > slice;
	unsigned index;
	int status;

	bool out_of_mem = false;

	// if there are discrepancies to use up
	if(discrepancies != 0) {

		index = 0;
		// constantly get next slice
		while(true) {
			if(debug) {
				print_beams(false);
				printf("discrepancies: %d, index: %d\n", discrepancies, index);
			}
			slice.clear();

			// get the next index
			status = next_slice(env, slice, false, index, goal);

			if(status == NO_CHILDREN) // if there are no children, return
				return 0;

			if(status != 0) // if have found goal or hit a limit
				return status;

			if(slice.size() == 0) // if slice is empty (already got last slice)
				break;

			// if running out of memory
			if(nodes_in_beam.size() + slice.size() > memory_limit) {
				out_of_mem = true;
				break;
			}

			// label nodes in slice
			for(unsigned i = 0; i < slice.size(); i++) {
				nodes_in_beam[slice[i].my_key] = beams.size();
			}
			beams.push_back(slice);
			// call BulbProbe
			status = BulbProbe(env, discrepancies - 1, goal);

			// remove slice
			for(unsigned i = 0; i < slice.size(); i++) {
				nodes_in_beam.erase(slice[i].my_key);
			}

			if(status != 0) // if found goal or hit limit
				return status;

			beams.pop_back();
		}
	}

	if(out_of_mem) // if ran out of memory when expanding not the best slice, will certainly run out of memory on best slice
		return 0;

	slice.clear();

	if(debug){
		print_beams(false);
		printf("Expanding First Slice\n");
	}

	// get first slice and expand it
	status = next_slice(env, slice, true, index, goal);

	// if no children to expand
	if(status == NO_CHILDREN || slice.size() == 0)
		return 0;

	if(status != 0)
		return status;

	// if running out of memory
	if(nodes_in_beam.size() + slice.size() > memory_limit) {
		out_of_mem = true;
		return 0;
	}

	for(unsigned i = 0; i < slice.size(); i++) {
		nodes_in_beam[slice[i].my_key] = beams.size();
	}
	beams.push_back(slice);
	status = BulbProbe(env, discrepancies, goal);

	for(unsigned i = 0; i < slice.size(); i++) {
		nodes_in_beam.erase(slice[i].my_key);
	}

	if(status != 0) // if found goal or hit limit
		return status;

	beams.pop_back();

	return status;
}

template <class state, class action, class environment>
int GeneralBulb<state, action, environment>::next_slice(environment *env, std::vector<BeamNode<state> > &slice, bool first_slice, unsigned &index, state &goal) {
	slice.clear();

	std::vector<BeamNode<state> > successors;

	int status = generate_successors(env, successors, goal);

	if(status == 1) {
		slice.push_back(successors.back());
		beams.push_back(slice);
		return 1;
	}
	// if have hit some limit
	if(status != 0)
		return status;

	if(successors.size() == 0) // if there are no successors, let caller know
		return NO_CHILDREN;

	unsigned succ_index = index;

	if(first_slice) { // if are to return the first slice
		succ_index = 0;
	}
	else if(!first_slice && index < beam_size) { // if index is not to be used, jump to second slice
		succ_index = beam_size;
	}

	// build slice
	while(succ_index < successors.size() && slice.size() < beam_size) {
		slice.push_back(successors[succ_index]);
		succ_index++;
	}

	index = succ_index; // fix index

	return 0; // goal not found and no limits hit
}

template <class state, class action, class environment>
int GeneralBulb<state, action, environment>::generate_successors(environment *env, std::vector<BeamNode<state> > &successors, state &goal) {

	nodes_in_current_layer.clear(); // nodes in current successors list
	successors.clear();

	std::vector<state> children;
	assert(beams.size() > 0);

	// iterate through states in last layer of beam
	for(unsigned i = 0 ; i < beams[beams.size() - 1].size(); i++) {
		nodes_ex_iter++;
		// if hit expansion limit
		if(bound_expanded && nodes_expanded + nodes_ex_iter >= expanded_limit) {
			return EXPAND_MET;
		}

		children.clear();
		env->GetSuccessors(beams[beams.size() - 1][i].my_state, children);
		nodes_touch_iter += children.size();

		// if hit generated limit
		if(bound_touched && nodes_touched + nodes_touch_iter >= touched_limit) {
			return TOUCHED_MET;
		}

		// iterate through generated child
		for(unsigned j = 0; j < children.size(); j++) {
			uint64_t child_key = env->GetStateHash(children[j]);

			// if already have the node stored in beam or in current layer
			if(nodes_in_beam.find(child_key) != nodes_in_beam.end() ||
			   nodes_in_current_layer.find(child_key) != nodes_in_current_layer.end()) {
				continue;
			}

			double child_g = beams[beams.size() - 1][i].g_value +
				env->GCost(beams[beams.size() - 1][i].my_state, children[j]);
			double child_h;

			if(env->IsGoalStored()) {
				child_h = env->HCost(children[j]);
			}
			else {
				child_h = env->HCost(children[j], goal);
			}
			double child_cost = get_cost(child_g, child_h);

			// construct new BeamNode
			BeamNode<state> new_node(children[j], child_g, child_h, child_cost, i, child_key);
			successors.push_back(new_node); // add to list of successors

			nodes_check_iter++;

			// if hit nodes checked limit
			if(bound_checked && nodes_checked + nodes_check_iter >= checked_limit) {
				return CHECKED_MET;
			}

			// if have found a goal
			if(env->GoalTest(children[j], goal)) {
				successors.clear();
				successors.push_back(new_node);
				best_path_cost = child_g;
				return 1;
			}

			// label node as stored in current layer
			nodes_in_current_layer[child_key] = 0;
		}
	}

	// sorts children
	sort(successors.begin(), successors.end());
	if(debug) {
		printf("Successors: \n");
		for(unsigned i = 0; i < successors.size(); i++) {
			std::cout << successors[i].my_state;
			printf(" G:%.0f H:%.0f Cost:%.0f\n", successors[i].g_value, successors[i].h_value, successors[i].cost);
		}
	}
	return 0; // no goal found and no limits hit
}

template <class state, class action, class environment>
void GeneralBulb<state, action, environment>::extract_goal(std::vector<state> &state_path) {
	state_path.resize(beams.size()); // properly resizes beam, last spot will have final node added elsewhere

	unsigned beam_layer = beams.size() - 1;
	unsigned current_index = beams[beam_layer].size() - 1;

	while(beam_layer > 0) { // extract states from beam
		state_path[beam_layer] = beams[beam_layer][current_index].my_state;
		current_index = beams[beam_layer][current_index].parent_index;
		beam_layer--;
	}

	state_path[beam_layer] = beams[beam_layer][current_index].my_state;
}

template <class state, class action, class environment>
void GeneralBulb<state, action, environment>::print_beams(bool last_only) {
	printf("\n\n");
	printf("Memory Used: %d\n", nodes_in_beam.size());
	unsigned i = 0;

	if(last_only) {
		i = beams.size() - 1;
	}
	for(; i < beams.size(); i++) {
		for(unsigned j = 0; j < beams[i].size(); j++) {

			if(!last_only) {
				for(unsigned k = 0; k < i; k++)
					printf(" ");
			}

			std::cout << beams[i][j].my_state;
			printf(" G:%.0f H:%.0f Cost:%.0f\n", beams[i][j].g_value, beams[i][j].h_value, beams[i][j].cost);
		}
	}
}

template <class state, class action, class environment>
bool GeneralBulb<state, action, environment>::SetExpandedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_expanded = true; expanded_limit = limit;
	}
	else {
		bound_expanded = false;
	}
	return true;
}

template <class state, class action, class environment>
bool GeneralBulb<state, action, environment>::SetTouchedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_touched = true; touched_limit = limit;
	}
	else {
		bound_touched = false;
	}
	return true;
}

template <class state, class action, class environment>
bool GeneralBulb<state, action, environment>::SetCheckedLimit(uint64_t limit) {
	if(limit > 0) {
		bound_checked = true; checked_limit = limit;
	}
	else {
		bound_checked = false;
	}
	return true;
}
#endif
