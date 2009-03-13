#ifndef EXP_BASICS
#define EXP_BASICS

#include <vector>
#include <cmath>
#include <map>
#include <algorithm>
#include <sstream>
#include <stdio.h>
#include "Common.h"
#include <iostream>
#include <sstream>
#include "MNPuzzle.h"
#include "GeneralIDA.h"
#include "GenericStepAlgorithm.h"

#define STATE_PATH 1
#define ACTION_PATH 2

void warnings();
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);

std::vector<std::string> split(const std::string &s, char delim);

typedef std::pair<double, double> Puzzle_Info;
int read_in_extra_puzz_info(const char *filename, std::vector<Puzzle_Info> &info, char delim, unsigned max_info);

template <class state, class action, class environment>
unsigned long long general_batch(environment *env, GenericStepAlgorithm<state, action, environment> *solver, std::vector<state> &puzzles, state &goal, bool print_all_stats, unsigned type);

unsigned long long general_batch_puzzles(unsigned num_cols, unsigned num_rows, GenericStepAlgorithm<MNPuzzleState, slideDir, MNPuzzle> *solver, std::vector<MNPuzzleState> &puzzles, std::vector<slideDir> &op_order, bool print_all_stats, unsigned type);

/*
double oracle_experiment(std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, bool print_all_stats);

void weight_gradient (std::vector<MNPuzzleState> &puzzles, std::vector<slideDir> &op_order);

unsigned independent_solvers_run(std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, MNPuzzle *mnp, MNPuzzleState start, MNPuzzleState goal);

void random_k_oracle(std::vector<std::pair<double, double> > &weights, std::vector<double> &solver_info, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, unsigned size, unsigned num, double &nodes, double &path_cost);

double ind_batch_puzzles(std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, bool print_all_stats);

void random_ind_puzzles(std::vector<GeneralIDA<MNPuzzleState, slideDir> *> solvers, std::vector<double> &solver_info, std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<slideDir> &op_order, unsigned size, unsigned num);*/

unsigned tt_simulation(const char *nodes_file, const char *cost_file, unsigned num_weights, unsigned puzzle_num, std::vector<unsigned> &puzzles_of_interest, std::vector<unsigned> &weights_of_interest, std::vector<unsigned> &set_sizes, std::vector<unsigned>&num_per_size, bool print_all_stats);

unsigned ind_approx(const char *nodes_file, const char *cost_file, unsigned num_weights, unsigned puzzle_num, unsigned start_w_index, unsigned end_w_index, unsigned start_size, unsigned end_size, unsigned num, bool print_all_stats);

void calculate_earned();

void get_standard_test_set(std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<double> &solver_info, unsigned num);

void get_big_4x4_test_set(std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<double> &solver_info, unsigned num);

void get_4x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_5x4_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_3x6_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_6x3_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_5x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);

//int output_exp_info(const char *filename, int num_probs, int num_solvers, const char *nodes_filename, const char *costs_filename);
int output_exp_info(const char *filename, int num_probs, const char *costs_filename, const char *ex_nodes_filename, const char *ch_nodes_filename, const char *t_nodes_filename);
int get_distribution(const char *filename, double num_buckets, double bucket_size);
#endif
