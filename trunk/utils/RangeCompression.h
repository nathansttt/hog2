#ifndef RANGE_COMPRESSION_H
#define RANGE_COMPRESSION_H

#include <vector>

/*
  Code by Malte Helmert
 
  Compute best values (= the ones maximizing the average
  heuristic value) to use in a range-compressed admissible
  heuristic.

  `distribution`: histogram of uncompressed heuristic values

  `num_values`: maximal number of values to use in the compressed
  heuristic; must be at least 1

  `result`: receives the result vector.

  The function computes up to `num_values` values, in sorted
  order. It will return fewer values if `distribution` contains
  fewer than `num_values` non-zero entries.
*/
void GetOptimizedBoundaries(const std::vector<uint64_t> &distribution,
                            int numValues, std::vector<int> &result);


// Debugging function.
void DumpOptimizedBoundaries(const std::vector<uint64_t> &distribution,
                             int numValues);

#endif
