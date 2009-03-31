#ifndef TT_SIM_H
#define TT_SIM_H

#include "experiment_basics.h"

using namespace std;

class TurnTakingSimulation {

public:
	TurnTakingSimulation(const char *input_file);

	static bool parse_input_files(const char *input_file,
	                         vector <string> &, vector<std::vector<double> > &,
	                         vector <vector<uint64_t> > &,
	                         vector <vector<uint64_t> > &,
	                         vector <vector<uint64_t> > &);

	static int output_turntaking_input_file(const char *filename, int num_probs, const char * output_file);

	virtual bool input_new_file(const char *input_file);
	void output_solver_names(vector<unsigned> &solvers_of_interest);

	void simulate(vector<unsigned> &puzzles_of_interest, vector<unsigned> &solvers_of_interest, vector<unsigned> &set_sizes, vector<unsigned> num_per_size, bool print_all_stats);

	int best_of_combo(vector<unsigned> &, vector<unsigned> &, unsigned);

	void turn_taking_on_weight_set(vector<unsigned> &, vector<unsigned> &, vector<unsigned> &, double &, double &, unsigned &, bool);
protected:
	vector <string> solver_names;
	vector<vector<double> > costs;
	vector<vector<uint64_t> >nodes_expanded;
	vector<vector<uint64_t> >nodes_checked;
	vector<vector<uint64_t> >nodes_touched;

	unsigned num_solvers, num_problems;
};
#endif
