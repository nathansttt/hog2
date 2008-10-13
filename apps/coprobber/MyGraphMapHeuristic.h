#include "GraphEnvironment.h"
#include <stdlib.h>

#ifndef MYGRAPHMAPHEURISTIC_H
#define MYGRAPHMAPHEURISTIC_H

class MyGraphMapHeuristic : public GraphHeuristic {
public:
  MyGraphMapHeuristic(Map *map, Graph *graph)
  :m(map), g(graph) {}
  double HCost(graphState &state1, graphState &state2)
  {
    int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
    int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
    int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
    int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);

		return max( abs(x1-x2), abs(y1-y2) );
  }
private:
  Map *m;
  Graph *g;
};

#endif
