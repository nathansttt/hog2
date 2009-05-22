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
#include "FloydWarshall.h"

using namespace GraphSearchConstants;

//int GraphMapInconsistentHeuristic::hmode=2;
//int GraphMapInconsistentHeuristic::HN=10;
//double GraphMapPerfectHeuristic::prob=0.5;

GraphEnvironment::GraphEnvironment(Graph *_g, GraphHeuristic *gh)
:g(_g), h(gh)
{
 	directed = false;
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
	//	delete g;
//	delete h;
}

void GraphEnvironment::GetSuccessors(graphState &stateID, std::vector<graphState> &neighbors) const
{
	neighbors.resize(0);
	node *n = g->GetNode(stateID);

	if(n == 0) {
		return;
	}

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
			if(stateID != e->getTo())
				neighbors.push_back(e->getTo());
			else
				neighbors.push_back(e->getFrom());
		}
	}
}

void GraphEnvironment::GetActions(graphState &stateID, std::vector<graphMove> &actions) const
{
	actions.resize(0);
	node *n = g->GetNode(stateID);

	if(n == 0) {
		return;
	}

	if (directed)
	{
		edge_iterator ei = n->getOutgoingEdgeIter();
		for (edge *e = n->edgeIterNextOutgoing(ei); e; e = n->edgeIterNextOutgoing(ei))
		{
			actions.push_back(graphMove(e->getFrom(),e->getTo()));
		}
	}
	else {
		edge_iterator ei = n->getEdgeIter();
		for (edge *e = n->edgeIterNext(ei); e; e = n->edgeIterNext(ei))
		{
			if(stateID != e->getTo())
				actions.push_back(graphMove(e->getFrom(),e->getTo()));
			else
				actions.push_back(graphMove(e->getTo(),e->getFrom()));
		}
	}

}

graphMove GraphEnvironment::GetAction(graphState &s1, graphState &s2) const
{
	return graphMove(s1, s2);
}

void GraphEnvironment::ApplyAction(graphState &s, graphMove a) const
{
	assert(s == a.from);
	s = a.to;
}

bool GraphEnvironment::InvertAction(graphMove &a) const
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

double GraphEnvironment::GCost(graphState &, graphMove &move)
{
	edge *e = g->FindEdge(move.from, move.to);
	assert(e);
	return e->GetWeight();
}

double GraphEnvironment::GCost(graphState &state1, graphState &state2)
{
	edge *e = g->FindEdge(state1, state2);
//	if (!e)
//		return -1000.0;
	assert(e);
	return e->GetWeight();
}

bool GraphEnvironment::GoalTest(graphState &state, graphState &goal)
{
	return state == goal;
}

uint64_t GraphEnvironment::GetStateHash(graphState &state) const
{
	return g->GetNode(state)->getUniqueID();
}

uint64_t GraphEnvironment::GetActionHash(graphMove act) const
{
	return (g->GetNode(act.from)->getUniqueID()<<16)|
	(g->GetNode(act.to)->getUniqueID());
}

void GraphEnvironment::OpenGLDraw() const
{
	if ((g == 0) || (g->GetNumNodes() == 0)) return;

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

void GraphEnvironment::OpenGLDraw(const graphState &s) const
{
	node *n = g->GetNode(s);
	DrawSphere((GLdouble)n->GetLabelF(GraphSearchConstants::kXCoordinate),
						 (GLdouble)n->GetLabelF(GraphSearchConstants::kYCoordinate),
						 (GLdouble)n->GetLabelF(GraphSearchConstants::kZCoordinate),
						 (GLdouble)2.0/(g->GetNumNodes()*g->GetNumNodes()));
}

void GraphEnvironment::OpenGLDraw(const graphState &, const graphMove &) const
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
				AddEdges(m, g, x, y);//, 1.0, 1.5);
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
:GraphDistanceHeuristic(graph), m(map)
{
	SetMode(kMax);
	SetNumUsedHeuristics(1000);
//	std::vector<std::vector<double> > values;
//	FloydWarshall(graph, values);
//	std::vector<int> randomizer;
//	randomizer.resize(values.size());
//	for (int x = 0; x < values.size(); x++)
//		randomizer[x] = x;
//	for (int x = values.size(); x > 0; x--)
//	{
//		int tmp = randomizer[x-1];
//		int switcher = random()%x;
//		randomizer[x-1] = randomizer[switcher];
//		randomizer[switcher] = tmp;
//	}
//	for (int x = 0; x < values.size(); x++)
//		AddHeuristic(values[randomizer[x]], x);
//	for (int x = 0; x < HN /*10*/; x++)
//	{
//		node *n = g->GetRandomNode();
//		graphState loc = n->GetNum();
//		std::vector<double> values;
//		GetOptimalDistances(n, values);
//		AddHeuristic(values, loc);
//	}
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

	if (hmode == kIgnore)
		return val;

	//for (unsigned int x = 0; x < heuristics.size(); x++)
	if (hmode == kRandom) {
		int x = (x1+x2+y1+y2)%heuristics.size();
		{
			double hval = heuristics[x][state1]-heuristics[x][state2];
			if (hval < 0) hval = -hval;
			if (fgreater(hval, val))
				val = hval;
		}
	}
	else if (hmode == kMax) { // hmode == 2, taking the max
		for (unsigned int i = 0; i < heuristics.size() && i < (unsigned int)numHeuristics; i++)
		{
			double hval = heuristics[i][state1]-heuristics[i][state2];
			if (hval < 0)
				hval = -hval;
			if (fgreater(hval,val))
				val = hval;
		}
	}
	else if (hmode == kGridMax) {  // hmode == 3, return max at grid points, otherwise 0
		if( (x1+x2) % 4 == 0 && (y1+y2) % 4 == 0) {
			for(unsigned int i=0;i<heuristics.size();i++) {
				double hval = heuristics[i][state1]-heuristics[i][state2];
				if(hval < 0)
					hval = -hval;
				if(fgreater(hval,val))
					val = hval;
			}
		}
		else
			val = 0;
	}

	return val;
}

void GraphDistanceHeuristic::OpenGLDraw() const
{
	static int counter = 0;
	counter = (counter+1);
	if (heuristics.size() == 0)
		return;

	for (unsigned int a = 0; a < locations.size(); a++)
	{
		GLdouble x, y, z;
		node *n = g->GetNode(locations[a]);
		x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
		y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
		z = n->GetLabelF(GraphSearchConstants::kZCoordinate);

		recColor r = getColor((counter+locations[a])%100, 0, 100, 4);

		glColor3f(r.r, r.g, r.b);
		DrawSphere(x, y, z, 0.05);
	}
}

double GraphDistanceHeuristic::HCost(graphState &state1, graphState &state2)
{
	double val = 0;
	for (unsigned int i = 0; i < heuristics.size(); i++)
	{
		double hval = heuristics[i][state1]-heuristics[i][state2];
		if (hval < 0)
			hval = -hval;
		if (fgreater(hval,val))
			val = hval;
	}
	return val;
}

void GraphDistanceHeuristic::ChooseStartGoal(graphState &start, graphState &goal)
{
	if (heuristics.size() == 0)
		return;
	double minStart=-1, minGoal=-1;

	minStart = heuristics[0][start];
	minGoal = heuristics[0][goal];
	for (unsigned int x = 1; x < heuristics.size(); x++)
	{
		if (heuristics[x][start] < minStart)
			minStart = heuristics[x][start];
		if (heuristics[x][goal] < minGoal)
			minGoal = heuristics[x][goal];
	}
	if (minStart < minGoal)
	{
		graphState tmp;
		tmp = start;
		start = goal;
		goal = tmp;
	}
}

void GraphDistanceHeuristic::AddHeuristic(node *n)
{
	if (smartPlacement)
		n = FindFarNode(n);
	else if (n == 0)
		n = g->GetRandomNode();
	
	std::vector<double> values;
	GetOptimalDistances(n, values);
	AddHeuristic(values, n->GetNum());
}

void GraphDistanceHeuristic::AddHeuristic(std::vector<double> &values, graphState location)
{
	heuristics.push_back(values);
	locations.push_back(location);
}



void GraphDistanceHeuristic::GetOptimalDistances(node *n, std::vector<double> &values)
{
	values.resize(g->GetNumNodes());
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

node *GraphDistanceHeuristic::FindFarNode(node *n)
{
	std::vector<double> values;
	values.resize(g->GetNumNodes());
	for (unsigned int x = 0; x < values.size(); x++)
		values[x] = -1;

	Heap h;
	for (unsigned int x = 0; x < locations.size(); x++)
	{
		n = g->GetNode(locations[x]);
		n->SetLabelF(kTemporaryLabel, 0.0);
		n->SetKeyLabel(kTemporaryLabel);
		h.Add(n);
	}
	if (locations.size() == 0)
	{
		if (n == 0)
			n = g->GetRandomNode();
		n->SetLabelF(kTemporaryLabel, 0.0);
		n->SetKeyLabel(kTemporaryLabel);
		h.Add(n);
	}

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
		if (h.Empty())
		{
//			printf("Selecting node at (%ld, %ld)\n", next->GetLabelL(GraphSearchConstants::kMapX),
//				   next->GetLabelL(GraphSearchConstants::kMapY));
			return next;
		}
	}
	return 0;
}


//GraphMapInconsistentHeuristic::GraphMapInconsistentHeuristic(Map *map, Graph *graph)
//:m(map), g(graph)
//{
//	for (int x = 0; x < HN /*10*/; x++)
//	{
//		node *n = g->GetRandomNode();
//		graphState loc = n->GetNum();
//		std::vector<double> values;
//		GetOptimalDistances(n, values);
//		AddHeuristic(values, loc);
//	}
//} 
//
//double GraphMapInconsistentHeuristic::HCost(graphState &state1, graphState &state2)
//{
//	int x1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapX);
//	int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
//	int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
//	int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);
//	
//	double a = ((x1>x2)?(x1-x2):(x2-x1));
//	double b = ((y1>y2)?(y1-y2):(y2-y1));
//	double val = (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
//
//	if(hmode == 0)
//		return val;
//
//	//for (unsigned int x = 0; x < heuristics.size(); x++)
//	if(hmode == 1) {
//		int x = (x1+x2+y1+y2)%heuristics.size();
//		{
//			double hval = heuristics[x][state1]-heuristics[x][state2];
//			if (hval < 0) hval = -hval;
//			if (fgreater(hval, val))
//				val = hval;
//		}
//	}
//	else if(hmode == 2) { // hmode == 2, taking the max
//		for(unsigned int i=0;i<heuristics.size();i++) {
//			double hval = heuristics[i][state1]-heuristics[i][state2];
//			if(hval < 0)
//				hval = -hval;
//			if(fgreater(hval,val))
//				val = hval;
//		}
//	}
//	else {  // hmode == 3, return max at grid points, otherwise 0
//		if( (x1+x2) % 4 == 0 && (y1+y2) % 4 == 0) {
//			for(unsigned int i=0;i<heuristics.size();i++) {
//				double hval = heuristics[i][state1]-heuristics[i][state2];
//				if(hval < 0)
//					hval = -hval;
//				if(fgreater(hval,val))
//					val = hval;
//			}
//		}
//		else
//			val = 0;
//	}
//
//	return val;
//}
//
//void GraphMapInconsistentHeuristic::AddHeuristic(std::vector<double> &values,
//												 graphState location)
//{
//	heuristics.push_back(values);
//	locations.push_back(location);
//}
//
//
//void GraphMapInconsistentHeuristic::GetOptimalDistances(node *n, std::vector<double> &values)
//{
//	values.resize(g->GetNumNodes());
//	for (unsigned int x = 0; x < values.size(); x++)
//		values[x] = -1.0;
//	n->SetLabelF(kTemporaryLabel, 0.0);
//	n->SetKeyLabel(kTemporaryLabel);
//	Heap h;
//	h.Add(n);
//	while (!h.Empty())
//	{
//		node *next = (node*)h.Remove();
////		printf("Heap size %d, working on node %d cost %f\n", h.size(), next->GetNum(),
////			   next->GetLabelF(kTemporaryLabel));
//		double cost = next->GetLabelF(kTemporaryLabel);
//		values[next->GetNum()] = next->GetLabelF(kTemporaryLabel);
//		neighbor_iterator ni = next->getNeighborIter();
//		for (long tmp = next->nodeNeighborNext(ni); tmp != -1; tmp = next->nodeNeighborNext(ni))
//		{
//			if (values[tmp] == -1)
//			{
//				node *nb = g->GetNode(tmp);
//				if (h.IsIn(nb))
//				{
//					if (fgreater(nb->GetLabelF(kTemporaryLabel),
//								 cost + g->FindEdge(next->GetNum(), tmp)->GetWeight()))
//					{
//						nb->SetLabelF(kTemporaryLabel,
//									  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
//						h.DecreaseKey(nb);
//					}
//				}
//				else {
//					nb->SetKeyLabel(kTemporaryLabel);
//					nb->SetLabelF(kTemporaryLabel,
//								  cost+g->FindEdge(next->GetNum(), tmp)->GetWeight());
//					h.Add(nb);
//				}
//			}
//		}
//	}
//}



AbstractionGraphEnvironment::AbstractionGraphEnvironment( GraphAbstraction *_gabs, unsigned int level, GraphHeuristic *gh ):
	GraphEnvironment( _gabs->GetAbstractGraph(level), gh ), gabs(_gabs)
{
	// compute graph scale
	node_iterator ni = g->getNodeIter();
	node *n;
	double min_x = DBL_MAX, max_x = DBL_MIN;
	double min_y = DBL_MAX, max_y = DBL_MIN;
	double min_z = DBL_MAX, max_z = DBL_MIN;
	n = g->nodeIterNext( ni );
	while( n != NULL ) {
		double x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
		double y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
		double z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
		if( x < min_x ) min_x = x;
		if( x > max_x ) max_x = x;
		if( y < min_y ) min_y = y;
		if( y > max_y ) max_y = y;
		if( z < min_z ) min_z = z;
		if( z > max_z ) max_z = z;
		n = g->nodeIterNext( ni );
	}
	graphscale = 0.;
	int count = 0;
	if( fgreater(max_x, min_x) ) { graphscale += max_x-min_x; count++; }
	if( fgreater(max_y, min_y) ) { graphscale += max_y-min_y; count++; }
	if( fgreater(max_z, min_z) ) { graphscale += max_z-min_z; count++; }
	// average over all the dimensions and divide by root(nodecount,dimensions)
	graphscale /= count * pow( (double)g->GetNumNodes(), 1./(double)count );
};

AbstractionGraphEnvironment::~AbstractionGraphEnvironment() {
	//~GraphEnvironment();
};

void AbstractionGraphEnvironment::OpenGLDraw() const {
	if ((g == 0) || (g->GetNumNodes() == 0)) return;

	glBegin(GL_LINES);

	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		//int x, y;
		//double offsetx, offsety;
		node *n;
		n = g->GetNode(e->getFrom());
		
		glColor3f( 0,0,0 );
		//glColor3f(1, 0, 0);
		//if (e->getMarked())
		//	glColor3f(1, 1, 1);
		
		GLdouble x, y, z;
		x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
		y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
		z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
		glVertex3f(x, y, z);
		
		n = g->GetNode(e->getTo());
		x = n->GetLabelF(GraphAbstractionConstants::kXCoordinate);
		y = n->GetLabelF(GraphAbstractionConstants::kYCoordinate);
		z = n->GetLabelF(GraphAbstractionConstants::kZCoordinate);
		
		glVertex3f(x, y, z);
	}
	glEnd();
};
