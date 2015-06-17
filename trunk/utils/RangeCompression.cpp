#include "RangeCompression.h"

#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

using namespace std;


class HeuristicTableEntry {
    int pos;
    int h;
    uint64_t numStates;

public:
    HeuristicTableEntry(int pos, int h, uint64_t numStates)
        : pos(pos),
          h(h),
          numStates(numStates) {
    }

    int GetPos() const {
        return pos;
    }

    int GetH() const {
        return h;
    }

    uint64_t GetNumStates() const {
        return numStates;
    }

    uint64_t GetPenalty(const HeuristicTableEntry &compressedEntry) const {
        return (h - compressedEntry.h) * numStates;
    }
};


class HeuristicTable {
    /*
      Represents the heuristic histogram in a more convenient form
      than the vector<int> we get as an input.

      The representation excludes histogram entries with 0 states,
      which is important for correctness. (The optimization algorithm
      assumes that every admissible solution for a given subproblem
      must include the lowest h value for that subproblem. This is
      only true if there exists at least one entry with that h value.)
    */

    vector<HeuristicTableEntry> entries;

public:
    explicit HeuristicTable(const vector<uint64_t> &histogram) {
        for (size_t h = 0; h < histogram.size(); ++h) {
            uint64_t numStates = histogram[h];
            assert(numStates >= 0);
            if (numStates) {
                HeuristicTableEntry entry(entries.size(), h, numStates);
                entries.push_back(entry);
            }
        }
    }

    const HeuristicTableEntry &operator[](int pos) const {
        return entries[pos];
    }

    int GetSize() const {
        return entries.size();
    }

    uint64_t ComputeCostBetween(size_t startPos, size_t endPos) const {
        const auto &compressedEntry = entries[startPos];
        uint64_t cost = 0;
        for (size_t pos = startPos; pos < endPos; ++pos)
            cost += entries[pos].GetPenalty(compressedEntry);
        return cost;
    }
};


class Solution {
    const HeuristicTableEntry *firstEntry;
    shared_ptr<const Solution> subsolution;
    uint64_t cost;

    int GetFirstH() const {
        return firstEntry->GetH();
    }

    int GetNextH() const {
        if (subsolution)
            return subsolution->GetFirstH();
        else
            return numeric_limits<int>::max();
    }

public:
    Solution(const HeuristicTable &hTable,
             const HeuristicTableEntry &onlyEntry)
        : firstEntry(&onlyEntry),
          subsolution(nullptr),
          cost(hTable.ComputeCostBetween(onlyEntry.GetPos(),
                                         hTable.GetSize())) {
    }

    Solution(const HeuristicTable &hTable,
             const HeuristicTableEntry &firstEntry,
             const shared_ptr<const Solution> &subsolution)
        : firstEntry(&firstEntry),
          subsolution(subsolution),
          cost(hTable.ComputeCostBetween(firstEntry.GetPos(),
                                         subsolution->firstEntry->GetPos()) +
               subsolution->cost) {
    }

    uint64_t GetCost() const {
        return cost;
    }

    vector<int> GetHValues() const {
        vector<int> result;
        const Solution *restSolution = this;
        while (restSolution) {
            result.push_back(restSolution->GetFirstH());
            restSolution = restSolution->subsolution.get();
        }
        return result;
    }

    double GetAverageH(const vector<uint64_t> &histogram) const {
        // This method is only used for statistics in the test code.
        double weightedH = 0.0;
        double totalStates = 0.0;
        const Solution *currentSolution = this;
        for (size_t h = 0; h < histogram.size(); ++h) {
            uint64_t numStates = histogram[h];
            while (currentSolution->GetNextH() <= static_cast<int>(h))
                currentSolution = currentSolution->subsolution.get();
            weightedH += currentSolution->GetFirstH() * numStates;
            totalStates += numStates;
        }
        return weightedH / totalStates;
    }
};


static void DumpSolution(const Solution &solution,
                         const vector<uint64_t> &histogram) {
    auto hValues = solution.GetHValues();
    cout << "[";
    for (size_t i = 0; i < hValues.size(); ++i) {
        if (i != 0)
            cout << ", ";
        cout << hValues[i];
    }
    cout << "] with cost " << solution.GetCost()
         << " and average h value " << solution.GetAverageH(histogram)
         << endl;
}


static vector<int> Optimize(const vector<uint64_t> &histogram, int numValues,
                            bool dumpResult = false) {
    assert(numValues >= 1);

    HeuristicTable hTable(histogram);

    vector<shared_ptr<const Solution>> subsolutions;
    for (int iteration = 0; iteration < numValues; ++iteration) {
        /*
          Invariant: After k iterations, `subsolutions[pos]` contains
          the best solution for the subproblem of compressing the
          distribution that starts at position `pos` to at most k
          values.
        */
        vector<shared_ptr<const Solution>> solutions;
        solutions.reserve(hTable.GetSize());

        for (int pos = 0; pos < hTable.GetSize(); ++pos) {
            const auto &entry = hTable[pos];
            Solution bestSolution(hTable, entry);

            if (!subsolutions.empty()) {
                for (int boundaryPos = pos + 1;
                     boundaryPos < hTable.GetSize();
                     ++boundaryPos) {
                    auto subsolution = subsolutions[boundaryPos];
                    Solution solution(hTable, entry, subsolution);
                    if (solution.GetCost() < bestSolution.GetCost())
                        bestSolution = solution;
                }
            }

            solutions.push_back(make_shared<Solution>(bestSolution));
        }
        subsolutions = move(solutions);
    }

    auto solution = *subsolutions[0];

    if (dumpResult)
        DumpSolution(solution, histogram);

    return solution.GetHValues();
}


void GetOptimizedBoundaries(const vector<uint64_t> &distribution,
                            int numValues, vector<int> &result) {
    result = Optimize(distribution, numValues, false);
}


void DumpOptimizedBoundaries(const vector<uint64_t> &distribution, int numValues) {
    Optimize(distribution, numValues, true);
}
