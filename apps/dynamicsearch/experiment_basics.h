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
#include "PancakePuzzle.h"
#include "GeneralIDA.h"
#include "GenericStepAlgorithm.h"
#include <stdlib.h>

#define STATE_PATH 1
#define ACTION_PATH 2

void warnings();
std::string int_to_string(int i);
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems);

std::vector<std::string> split(const std::string &s, char delim);

typedef std::pair<double, double> Puzzle_Info;
int read_in_extra_puzz_info(const char *filename, std::vector<Puzzle_Info> &info, char delim, unsigned max_info);

template <class state, class action, class environment>
uint64_t general_batch(environment *env, GenericStepAlgorithm<state, action, environment> *solver, std::vector<state> &puzzles, state &goal, bool print_all_stats, unsigned type);

uint64_t general_batch_puzzles(unsigned num_cols, unsigned num_rows, GenericStepAlgorithm<MNPuzzleState, slideDir, MNPuzzle> *solver, std::vector<MNPuzzleState> &puzzles, const std::vector<slideDir>, bool print_all_stats, unsigned type);

uint64_t general_batch_pancake_puzzles(PancakePuzzle &env, unsigned size, GenericStepAlgorithm<PancakePuzzleState, unsigned, PancakePuzzle> *solver, std::vector<PancakePuzzleState> &puzzles, bool print_all_stats, unsigned type);

/** This is the korf test set. It is of 4x4 puzle problems. Solver_info contains info on each problem that can be used. num is the number of problems you want appended to puzzles. num cannot exceed 100. **/
void get_standard_test_set(std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<double> &solver_info, unsigned num);

/** This is a set of 1000 randomly generated but solvable 4x4 problems. num is the number of problems you want appended to puzzles. num cannot exceed 1000.**/
void get_big_4x4_test_set(std::vector<MNPuzzleState> &puzzles, std::vector<Puzzle_Info> &info, std::vector<double> &solver_info, unsigned num);

/** These return various test sets of the type num_cols x num_rows. Each test set contains a 1000 problems. num is the number of problems you want appended to puzzles. num cannot exceed 1000.**/
void get_4x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_5x4_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_3x6_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_6x3_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_5x5_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_6x6_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);
void get_7x7_test_set(std::vector<MNPuzzleState> &puzzles, unsigned num);

void get_12pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_13pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_14pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_16pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_18pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);
void get_20pancake_test_set(std::vector<PancakePuzzleState> &puzzles, unsigned num);

int get_distribution(const char *filename, double num_buckets, double bucket_size);

bool get_next_combo(std::vector<unsigned> &current_combination, unsigned max_num, unsigned pos, bool set_as_smallest);

void print_combo(std::vector<unsigned> &current_combination);

void get_rand_combo(std::vector<unsigned> &combo, unsigned start_w_index, unsigned end_w_index, unsigned size);
#endif
