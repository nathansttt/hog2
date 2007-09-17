/*
 *  GraphEnvironment.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 5/29/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "GraphEnvironment.h"
#include "GLUtil.h"
#include "Heap.h"

using namespace GraphSearchConstants;

GraphEnvironment::GraphEnvironment(Graph *_g, GraphHeuristic *gh)
:g(_g), h(gh)
{
	directed = true;
}

//GraphEnvironment::GraphEnvironment(Map *m)
//{
//	h = 0;
//	directed = false;
//	g = GetMapGraph(m);
//	//BuildHeuristics(m, g);
//}

GraphEnvironment::~GraphEnvironment()
{
	// delete g; ??
}

void GraphEnvironment::GetSuccessors(graphState &stateID, std::vector<graphState> &neighbors)
{
	neighbors.resize(0);
	node *n = g->GetNode(stateID);
	if (directed)
	{
		edge_iterator ei = n->getOutgoingEdgeIter();
		for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
		{
			neighbors.push_back(e->getTo());
		}
	}
	else {
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			neighbors.push_back(e->getTo());
		}
	}
}

void GraphEnvironment::GetActions(graphState &stateID, std::vector<graphMove> &actions)
{
	actions.resize(0);
	node *n = g->GetNode(stateID);
	edge_iterator ei = n->getOutgoingEdgeIter();
	for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
	{
		actions.push_back(graphMove(e->getFrom(), e->getTo()));
	}
}

graphMove GraphEnvironment::GetAction(graphState &s1, graphState &s2)
{
	return graphMove(s1, s2);
}

void GraphEnvironment::ApplyAction(graphState &s, graphMove a)
{
	assert(s == a.from);
	s = a.to;
}

bool GraphEnvironment::InvertAction(graphMove &a)
{
	uint16_t tmp = a.from;
	a.from = a.to;
	a.to = tmp;
	if (g->findDirectedEdge(a.from, a.to))
		return true;
	return false;
}

double GraphEnvironment::HCost(graphState &state1, graphState &state2)
{
	if (h)
		return h->HCost(state1, state2);
	return 0;
}

double GraphEnvironment::GCost(graphState &state1, graphState &state2)
{
	edge *e = g->FindEdge(state1, state2);
	assert(e);
	return e->GetWeight();
}

bool GraphEnvironment::GoalTest(graphState &state, graphState &goal)
{
	return state == goal;
}

uint64_t GraphEnvironment::GetStateHash(graphState &state)
{
	return g->GetNode(state)->getUniqueID();
}

uint64_t GraphEnvironment::GetActionHash(graphMove act)
{
	return (g->GetNode(act.from)->getUniqueID()<<16)|
	(g->GetNode(act.to)->getUniqueID());
}

void GraphEnvironment::OpenGLDraw(int window)
{
	if ((g == 0) || (g->getNumNodes() == 0)) return;

	glBegin(GL_LINES);
	glNormal3f(0, 1, 0);
		
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		//int x, y;
		//double offsetx, offsety;
		node *n;
		n = g->GetNode(e->getFrom());
		
		glColor3f(1, 0, 0);
		if (e->getMarked())
			glColor3f(1, 1, 1);
		
		GLdouble x, y, z;
		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
		glVertex3f(x, y, z);
		
		n = g->GetNode(e->getTo());
		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
		
		glVertex3f(x, y, z);
	}
	glEnd();
}

void GraphEnvironment::OpenGLDraw(int, graphState &s)
{
	node *n = g->GetNode(s);
	DrawSphere((GLdouble)n->GetLabelF(GraphSearchConstants::kXCoordinate),
						 (GLdouble)n->GetLabelF(GraphSearchConstants::kYCoordinate),
						 (GLdouble)n->GetLabelF(GraphSearchConstants::kZCoordinate),
						 (GLdouble)2.0/(g->getNumNodes()*g->getNumNodes()));
}

void GraphEnvironment::OpenGLDraw(int, graphState &, graphMove &)
{
	// if we want to draw a set of moves we use this to do so
}

namespace GraphSearchConstants {

	/**
	* GetMapGraph(map)
	 *
	 * Given a map, this function uses the external map interfaces to turn it
	 * into a Graph, and sets the appropriate node numbers for that map. This
	 * function should not be called multiple times on the same map, because
	 * the original Graph map lose it's association with the map.
	 */
	Graph *GetGraph(Map *m)
	{
		char name[32];
		Graph *g = new Graph();
		node *n;
		for (int y = 0; y < m->getMapHeight(); y++)
		{
			for (int x = 0; x < m->getMapWidth(); x++)
			{
				Tile &currTile = m->getTile(x, y);
				currTile.tile1.node = kNoGraphNode;
				currTile.tile2.node = kNoGraphNode;
				
				if (m->adjacentEdges(x, y, kInternalEdge))
				{
					GLdouble xx, yy, zz, rr;
					m->getOpenGLCoord(x, y,xx, yy, zz, rr);
					if (m->getTerrainType(x, y) == kOutOfBounds)
						continue;
					sprintf(name, "(%d, %d)", x, y);
					currTile.tile1.node = g->AddNode(n = new node(name));
					n->SetLabelF(kXCoordinate, xx);
					n->SetLabelF(kYCoordinate, yy);
					n->SetLabelF(kZCoordinate, 00);
					n->SetLabelL(kMapX, x);
					n->SetLabelL(kMapY, y);
				}
				else {
					if (m->getTerrainType(x, y, kLeftEdge) != kOutOfBounds)
					{
						GLdouble xx, yy, zz, rr;
						m->getOpenGLCoord(x, y,xx, yy, zz, rr);
						if (m->getTerrainType(x, y) == kOutOfBounds)
							continue;
						sprintf(name, "(%d, %d)", x, y);
						currTile.tile1.node = g->AddNode(n = new node(name));
						n->SetLabelF(kXCoordinate, xx);
						n->SetLabelF(kYCoordinate, yy);
						n->SetLabelF(kZCoordinate, 00);
						n->SetLabelL(kMapX, x);
						n->SetLabelL(kMapY, y);
						//					if (currTile.split == kForwardSplit)
	//						n->SetLabelL(kFirstData+2, kTopLeft);
	//					else
	//						n->SetLabelL(kFirstData+2, kBottomLeft);
					}
					
					if (m->getTerrainType(x, y, kRightEdge) != kOutOfBounds)
					{
						GLdouble xx, yy, zz, rr;
						m->getOpenGLCoord(x, y,xx, yy, zz, rr);
						if (m->getTerrainType(x, y) == kOutOfBounds)
							continue;
						sprintf(name, "(%d, %d)", x, y);
						currTile.tile1.node = g->AddNode(n = new node(name));
						n->SetLabelF(kXCoordinate, xx);
						n->SetLabelF(kYCoordinate, yy);
						n->SetLabelF(kZCoordinate, 00);
						n->SetLabelL(kMapX, x);
						n->SetLabelL(kMapY, y);
	//					if (currTile.split == kForwardSplit)
	//						n->SetLabelL(kFirstData+2, kBottomRight);
	//					else
	//						n->SetLabelL(kFirstData+2, kTopRight);
					}
				}
			}
		}
		for (int y = 0; y < m->getMapHeight(); y++)
		{
			for (int x = 0; x < m->getMapWidth(); x++)
			{
				AddEdges(m, g, x, y);
			}
		}
		// printf("Done\n");
		
		// verify Graph is correct
	#if 0
		{
			node_iterator ni = g->getNodeIter();
			for (n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
			{
				int numEdges = n->getNumOutgoingEdges() + n->getNumIncomingEdges();
				
				edge *ee;
				edge_iterator eie = n->getEdgeIter();
				for (int x = 0; x < numEdges; x++)
				{
					ee = n->edgeIterNext(eie);
					if (ee == 0)
					{ cout << "**That's impossible; we were told we had " << numEdges << ":(" << n->getNumOutgoingEdges() << "+" << n->getNumIncomingEdges() <<
						") edges, we're on #" << x << " and we got nil!";
						cout << "(node " << n->GetNum() << ")" << endl;
						break;
					}
				}			
			}
		}
	#endif
		
		return g;
	}

	/**
	* AddEdges(map, Graph, x, y)
	 *
	 * This is a helper function for GetMapGraph that does the work of adding
	 * the Graph edges. Each edge is only added once, so while the Graph has
	 * directional edges, we treat them as being bidirectional.
	 */
	void AddEdges(Map *m, Graph *g, int x, int y,
				  double straigtEdgeCost,
				  double diagEdgeCost,
				  int straightEdgeProb,
				  int diagEdgeProb)
	{
		// check 4 surrounding edges
		// when we get two of them, we add the corresponding diagonal edge(?)...not yet
		// make sure the edge we add isn't there already!
		// make sure we get the right node # when we add the edge
		edge *e = 0;
		
		// left edge is always tile1, right edge is always tile 2
		if ((x >= 1) && (m->adjacentEdges(x, y, kLeftEdge)) && (m->getTile(x, y).tile1.node != kNoGraphNode))
		{
			if (m->adjacentEdges(x-1, y, kInternalEdge) && (m->getTile(x-1, y).tile1.node != kNoGraphNode))
			{
				if ((random()%100) < straightEdgeProb)
				{
					e = new edge(m->getTile(x, y).tile1.node, m->getTile(x-1, y).tile1.node, straigtEdgeCost);
					g->AddEdge(e);
					e = new edge(m->getTile(x-1, y).tile1.node, m->getTile(x, y).tile1.node, straigtEdgeCost);
					g->AddEdge(e);
				}
			}
			else if (m->getTile(x-1, y).tile2.node != kNoGraphNode)
			{
				if ((random()%100) < straightEdgeProb)
				{
					e = new edge(m->getTile(x, y).tile1.node, m->getTile(x-1, y).tile2.node, straigtEdgeCost);
					g->AddEdge(e);
					e = new edge(m->getTile(x-1, y).tile2.node, m->getTile(x, y).tile1.node, straigtEdgeCost);
					g->AddEdge(e);
				}
			}
		}

		// top edge (may be tile 1 or tile 2)
		if ((y >= 1) && (m->adjacentEdges(x, y, kTopEdge)))
		{
			if ((m->adjacentEdges(x, y, kInternalEdge)) || (m->getSplit(x, y) == kForwardSplit))
			{
				if (m->getTile(x, y).tile1.node != kNoGraphNode)
				{
					if (m->adjacentEdges(x, y-1, kInternalEdge) || (m->getSplit(x, y-1) == kBackwardSplit))
					{
						if (m->getTile(x, y-1).tile1.node != kNoGraphNode)
						{
							if ((random()%100) < straightEdgeProb)
							{
								e = new edge(m->getTile(x, y).tile1.node, m->getTile(x, y-1).tile1.node, straigtEdgeCost);
								g->AddEdge(e);
								e = new edge(m->getTile(x, y-1).tile1.node, m->getTile(x, y).tile1.node, straigtEdgeCost);
								g->AddEdge(e);
							}
						}
					}
					else if (m->getTile(x, y-1).tile2.node != kNoGraphNode)
					{
						if ((random()%100) < straightEdgeProb)
						{
							e = new edge(m->getTile(x, y).tile1.node, m->getTile(x, y-1).tile2.node, straigtEdgeCost);
							g->AddEdge(e);
							e = new edge(m->getTile(x, y-1).tile2.node, m->getTile(x, y).tile1.node, straigtEdgeCost);
							g->AddEdge(e);
						}
					}
				}
			}
			else {
				if (m->adjacentEdges(x, y-1, kInternalEdge) || (m->getSplit(x, y-1) == kBackwardSplit))
				{
					if ((m->getTile(x, y).tile2.node != kNoGraphNode) && (m->getTile(x, y-1).tile1.node != kNoGraphNode))
					{
						if ((random()%100) < straightEdgeProb)
						{
							e = new edge(m->getTile(x, y).tile2.node, m->getTile(x, y-1).tile1.node, straigtEdgeCost);
							g->AddEdge(e);
							e = new edge(m->getTile(x, y-1).tile1.node, m->getTile(x, y).tile2.node, straigtEdgeCost);
							g->AddEdge(e);
						}
					}
				}
				else if ((m->getTile(x, y).tile2.node != kNoGraphNode) && (m->getTile(x, y-1).tile2.node != kNoGraphNode))
				{
					if ((random()%100) < straightEdgeProb)
					{
						e = new edge(m->getTile(x, y).tile2.node, m->getTile(x, y-1).tile2.node, straigtEdgeCost);
						g->AddEdge(e);
						e = new edge(m->getTile(x, y-1).tile2.node, m->getTile(x, y).tile2.node, straigtEdgeCost);
						g->AddEdge(e);
					}
				}
			}
		}
		e = 0;
		// diagonal UpperLeft edge, always node 1...
		// (1) we can cross each of the boundaries between tiles
		if ((x >= 1) && (y >= 1) && (m->adjacentEdges(x, y, kLeftEdge)) && (m->adjacentEdges(x, y, kTopEdge)) &&
				(m->adjacentEdges(x, y-1, kLeftEdge)) && (m->adjacentEdges(x-1, y, kTopEdge)) &&
				(m->getTile(x, y).tile1.node != kNoGraphNode))
		{
			// (2) we can cross the inner tile boundaries
			if (((m->adjacentEdges(x-1, y, kInternalEdge)) || (m->getSplit(x-1, y) == kBackwardSplit)) &&
					((m->adjacentEdges(x, y-1, kInternalEdge)) || (m->getSplit(x, y-1) == kBackwardSplit)) &&
					((m->adjacentEdges(x-1, y-1, kInternalEdge)) || (m->getSplit(x-1, y-1) == kForwardSplit)) &&
					((m->adjacentEdges(x, y, kInternalEdge)) || (m->getSplit(x, y) == kForwardSplit)))
			{
				// (3) find what tiles to connect
				if (m->adjacentEdges(x-1, y-1, kInternalEdge))
				{
					if (m->getTile(x-1, y-1).tile1.node != kNoGraphNode)
					{
						if ((random()%100) < diagEdgeProb)
						{
							e = new edge(m->getTile(x, y).tile1.node, m->getTile(x-1, y-1).tile1.node, diagEdgeCost);
							g->AddEdge(e);
							e = new edge(m->getTile(x-1, y-1).tile1.node, m->getTile(x, y).tile1.node, diagEdgeCost);
							g->AddEdge(e);
						}
					}
				}
				else if (m->getTile(x-1, y-1).tile2.node != kNoGraphNode)
				{
					if ((random()%100) < diagEdgeProb)
					{
						e = new edge(m->getTile(x, y).tile1.node, m->getTile(x-1, y-1).tile2.node, diagEdgeCost);
						g->AddEdge(e);
						e = new edge(m->getTile(x-1, y-1).tile2.node, m->getTile(x, y).tile1.node, diagEdgeCost);
						g->AddEdge(e);
					}
				}
			}
		}

		// diagonal UpperRight edge
		// (1) we can cross each of the boundaries between tiles
		if ((y >= 1) && (x < m->getMapWidth()-1) && (m->adjacentEdges(x, y, kRightEdge)) && (m->adjacentEdges(x, y, kTopEdge)) &&
				(m->adjacentEdges(x, y-1, kRightEdge)) && (m->adjacentEdges(x+1, y, kTopEdge)) &&
				(m->getTile(x+1, y-1).tile1.node != kNoGraphNode))
		{
			// (2) we can cross the inner tile boundaries
			if (((m->adjacentEdges(x+1, y, kInternalEdge)) || (m->getSplit(x+1, y) == kForwardSplit)) &&
					((m->adjacentEdges(x, y-1, kInternalEdge)) || (m->getSplit(x, y-1) == kForwardSplit)) &&
					((m->adjacentEdges(x+1, y-1, kInternalEdge)) || (m->getSplit(x+1, y-1) == kBackwardSplit)) &&
					((m->adjacentEdges(x, y, kInternalEdge)) || (m->getSplit(x, y) == kBackwardSplit)))
			{
				// (3) connect
				if (m->adjacentEdges(x, y, kInternalEdge))
				{
					if (m->getTile(x, y).tile1.node != kNoGraphNode)
					{
						if ((random()%100) < diagEdgeProb)
						{
							e = new edge(m->getTile(x, y).tile1.node, m->getTile(x+1, y-1).tile1.node, diagEdgeCost);
							g->AddEdge(e);
							e = new edge(m->getTile(x+1, y-1).tile1.node, m->getTile(x, y).tile1.node, diagEdgeCost);
							g->AddEdge(e);
						}
					}
				}
				else if (m->getTile(x, y).tile2.node != kNoGraphNode)
				{
					if ((random()%100) < diagEdgeProb)
					{
						e = new edge(m->getTile(x, y).tile2.node, m->getTile(x+1, y-1).tile1.node, diagEdgeCost);
						g->AddEdge(e);
						e = new edge(m->getTile(x+1, y-1).tile1.node, m->getTile(x, y).tile2.node, diagEdgeCost);
						g->AddEdge(e);
					}
				}
			}
		}	
	}
}

GraphMapInconsistentHeuristic::GraphMapInconsistentHeuristic(Map *map, Graph *graph)
:m(map), g(graph)
{
	for (int x = 0; x < 10; x++)
	{
		node *n = g->GetRandomNode();
		graphState loc = n->GetNum();
		std::vector<double> values;
		GetOptimalDistances(n, values);
		AddHeuristic(values, loc);
	}
}

double GraphMapInconsistentHeuristic::HCost(graphState &state1, graphState &state2)
{
	int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
	int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
	int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
	int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);
	
	double a = ((x1>x2)?(x1-x2):(x2-x1));
	double b = ((y1>y2)?(y1-y2):(y2-y1));
	double val = (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
//	return val;
	//for (unsigned int x = 0; x < heuristics.size(); x++)
	int x = (x1+x2+y1+y2)%heuristics.size();
	{
		double hval = heuristics[x][state1]-heuristics[x][state2];
		if (hval < 0) hval = -hval;
		if (fgreater(hval, val))
			val = hval;
	}
	return val;
}

void GraphMapInconsistentHeuristic::AddHeuristic(std::vector<double> &values,
												 graphState location)
{
	heuristics.push_back(values);
	locations.push_back(location);
}


void GraphMapInconsistentHeuristic::GetOptimalDistances(node *n, std::vector<double> &values)
{
	values.resize(g->getNumNodes());
	for (unsigned int x = 0; x < values.size(); x++)
		values[x] = -1.0;
	n->SetLabelF(kTemporaryLabel, 0.0);
	n->SetKeyLabel(kTemporaryLabel);
	Heap h;
	h.Add(n);
	while (!h.Empty())
	{
		node *next = (node*)h.Remove();
//		printf("Heap size %d, working on node %d cost %f\n", h.size(), next->GetNum(),
//			   next->GetLabelF(kTemporaryLabel));
		double cost = next->GetLabelF(kTemporaryLabel);
		values[next->GetNum()] = next->GetLabelF(kTemporaryLabel);
		neighbor_iterator ni = next->getNeighborIter();
		for (long tmp = next->nodeNeighborNext(ni); tmp != -1; tmp = next->nodeNeighborNext(ni))
		{
			if (values[tmp] == -1)
			{
				node *nb = g->GetNode(tmp);
				if (h.IsIn(nb))
				{
					if (fgreater(nb->GetLabelF(kTemporaryLabel),
								 cost + g->FindEdge(next->GetNum(), tmp)->GetWeight()))
					{
						nb->SetLabelF(kTemporaryLabel,
									  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
						h.DecreaseKey(nb);
					}
				}
				else {
					nb->SetKeyLabel(kTemporaryLabel);
					nb->SetLabelF(kTemporaryLabel,
								  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
					h.Add(nb);
				}
			}
		}
	}
}
