#include "GraphEnvironment.h"
#include <stdlib.h>

#ifndef MAXIMUMNORMGRAPHMAPHEURISTIC_H
#define MAXIMUMNORMGRAPHMAPHEURISTIC_H

class MaximumNormGraphMapHeuristic : public GraphHeuristic {
public:
  MaximumNormGraphMapHeuristic(Graph *graph)
  :g(graph) {}
  double HCost(graphState &state1, graphState &state2)
  {
    int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
    int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
    int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
    int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);

		return max( abs(x1-x2), abs(y1-y2) );
  }
private:
  Graph *g;
};

#endif
