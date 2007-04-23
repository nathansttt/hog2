/*
 * $Id: mapAbstraction.cpp,v 1.21 2007/03/24 23:34:11 nathanst Exp $
 *
 *  mapAbstraction.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/3/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "mapAbstraction.h"
#include "constants.h"

mapAbstraction::~mapAbstraction()
{ 
	delete m;
}

void mapAbstraction::getRandomTileFromNode(node *n, int &x, int &y)
{
	while (getAbstractionLevel(n) != 0)
		n = getNthChild(n, random()%getNumChildren(n));
	x = n->getLabelL(kFirstData);
	y = n->getLabelL(kFirstData+1);
}

void mapAbstraction::getTileFromNode(node *n, int &x, int &y)
{
	if (getAbstractionLevel(n) != 0)
	{
		getTileUnderLoc(x, y, getNodeLoc(n));
		node *t = getNodeFromMap(x, y);
		if (getNthParent(t, getAbstractionLevel(n)) == n)
		{
			//			printf("Nth parent is %d (%d), not %d (%d)\n",
			//						 t->getNum(), getAbstractionLevel(t),
			//						 n->getNum(), getAbstractionLevel(n));
			return;
		}
		while (getAbstractionLevel(n) != 0)
			n = getNthChild(n, 0);
	}
	x = n->getLabelL(kFirstData);
	y = n->getLabelL(kFirstData+1);
	//printf("(%d, %d) in x/y\n", x, y);
}

void mapAbstraction::toggleDrawAbstraction(int which)
{
	bool drawThis = ((levelDraw>>which)&0x1);
	if (!drawThis)
		levelDraw |= (1<<which);
	else
		levelDraw = levelDraw&(~(1<<which));
}

void mapAbstraction::OpenGLDraw()
{
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		if ((levelDraw >> x) & 1) drawGraph(abstractions[x]);
		//glCallList(displayLists[x]);
	}
}


void mapAbstraction::drawGraph(graph *g)
{
	if ((g == 0) || (g->getNumNodes() == 0)) return;
	
	int abLevel = g->getNode(0)->getLabelL(kAbstractionLevel);	
	
	//  if (verbose&kBuildGraph) printf("Drawing graph abstraction %d!\n", abLevel);
	glBegin(GL_LINES);
	glNormal3f(0, 1, 0);
	
	//	glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), .25+((GLfloat)(abLevel%15-8)/30.0), 1);
	//	switch (g->getNode(0)->getLabelL(kAbstractionLevel)%3)
	//	{
	//		case 0: glColor4f(1, 0, 0, 1); break;
	//		case 1: glColor4f(0, 1, 0, 1); break;
	//		case 2: glColor4f(1, 1, 0, 1); break;
	//		case 3: glColor4f(0, 0, 1, .5); break;
	//		case 4: glColor4f(1, 0, 1, .5); break;
	//		case 5: glColor4f(1, 1, 0, .5); break;
	//		case 6: glColor4f(0, 1, 0, .5); break;
	//		case 7: glColor4f(0, 1, 0, .5); break;
	//		default:glColor4f(0, 0, 1, .5); break;
	//	}
	node_iterator ni;
	//	ni = g->getNodeIter();
	//	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	//	{
	//		if (n->getLabelL(kNodeBlocked) > 0)
	//		{
	//			glColor3f(1, 1, 1);
	//			recVec rv = getNodeLoc(n);
	//			glVertex3f(rv.x, 0, rv.z);
	//			glVertex3f(rv.x, rv.y, rv.z);
	//		}
	//	}
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		//int x, y;
		//double offsetx, offsety;
		node *n;
		n = g->getNode(e->getFrom());
		
		if (e->getLabelL(kEdgeCapacity) == 0)      glColor4f(.5, .5, .5, 1);
		else if (e->getLabelL(kEdgeCapacity) <= 0) glColor4f(.2, .2, .2, 1);
		else if (e->getMarked())                  glColor4f(1, 1, 1, 1);
		else if (abLevel%2)
			glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), 0, 1);
		else glColor4f(((GLfloat)(abLevel%15)/15.0), 1-((GLfloat)(abLevel%15)/15.0), 0, 1);
		//glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), .5+((GLfloat)(abLevel%15-8)/30.0), 1);
		
		// Color the edge widths
		// 		else if (e->getWidth() <= 0.0)
		// 			glColor4f(1, 0, 0, 1);
		// 		else if (e->getWidth() <= 0.8)
		// 			glColor4f(1, 0, 1, 1);
		// 		else if (e->getWidth() <= 1.1)
		// 			glColor4f(1, 1, 0, 1);
		// 		else if (e->getWidth() <= 1.5)
		// 			glColor4f(0, 1, 1, 1);
		// 		else if (e->getWidth() <= 2.1)
		// 			glColor4f(0, 1, 0, 1);
		//if (e->getMarked()) {	
		recVec rv = getNodeLoc(n);
		glVertex3f(rv.x, rv.y, rv.z);
		
		n = g->getNode(e->getTo());
		rv = getNodeLoc(n);
		
		glVertex3f(rv.x, rv.y, rv.z);
		//}
	}
//node_iterator
ni = g->getNodeIter();
for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
drawLevelConnections(n);
glEnd();
//  if (verbose&kBuildGraph) printf("Done\n");
}

void mapAbstraction::drawLevelConnections(node *n)
{
	//	int x, y;
	//	double offsetx, offsety;
	//	recVec ans;
	//if (n->getNumOutgoingEdges()+n->getNumIncomingEdges() == 0) return;
	
	if (n->getLabelL(kAbstractionLevel) == 0) return;
	else {
		glColor4f(.6, .6, .6, .6);
		recVec v = getNodeLoc(n);
		for (int cnt = 0; cnt < n->getLabelL(kNumAbstractedNodes); cnt++)
		{
			recVec v1 = getNodeLoc(abstractions[n->getLabelL(kAbstractionLevel)-1]->getNode(n->getLabelL(kFirstData+cnt)));
			glVertex3f(v.x, v.y, v.z);
			glVertex3f(v1.x, v1.y, v1.z);
		}
	}
	//return ans;
}

void mapAbstraction::getTileUnderLoc(int &x, int &y, const recVec &v)
{
	double width = (getMap()->getMapWidth()+1)/2.0;
	double height = (getMap()->getMapHeight()+1)/2.0;
	double offsetx, offsety;
	offsetx = offsety = .5;
	x = (int)(width*(v.x+1.0));
	y = (int)(height*(v.y+1.0));
	//	printf("(%1.2f, %1.2f) in openGL converted to (%d, %d) in x/y\n",
	//				 v.x, v.y, x, y);
}

recVec mapAbstraction::getNodeLoc(node *n)
{
	int x, y;
	//  double offsetx, offsety;
	recVec ans;
	
	if (n->getLabelF(kXCoordinate) != kUnknownPosition)
	{
		ans.x = n->getLabelF(kXCoordinate);
		ans.y = n->getLabelF(kYCoordinate);
		ans.z = n->getLabelF(kZCoordinate);
		return ans;
	}
	
	//	double width = getMap()->getMapWidth();
	//	double height = getMap()->getMapHeight();
	
	if (n->getLabelL(kAbstractionLevel) == 0)
	{
		x = n->getLabelL(kFirstData);
		y = n->getLabelL(kFirstData+1);
		
		Map *mp = getMap();
		double r;
		mp->getOpenGLCoord(x,y,ans.x,ans.y,ans.z,r);
		//    switch (n->getLabelL(kFirstData+2)) {
		//			case kTopLeft: offsetx = .3; offsety = .3; break;
		//			case kTopRight: offsetx = .6; offsety = .3; break;
		//			case kBottomLeft: offsetx = .3; offsety = .6; break;
		//			case kBottomRight: offsetx = .6; offsety = .6; break;
		//			case kNone:
		//			default: offsetx = .5; offsety = .5; break;
		//    }
		//    ans.x = (double)x/width+offsetx/width-.5;
		//    ans.z = (double)y/height+offsety/width-.5;
	}
	else {
		int totNodes = 0;
		ans.x = ans.y = ans.z = 0;
		for (int cnt = 0; cnt < n->getLabelL(kNumAbstractedNodes); cnt++)
		{
			int absLevel = n->getLabelL(kAbstractionLevel)-1;
			node *nextChild = abstractions[absLevel]->getNode(n->getLabelL(kFirstData+cnt));
			recVec tmp = getNodeLoc(nextChild);
			int weight = nextChild->getLabelL(kNumAbstractedNodes);
			totNodes += weight;
			ans.x += weight*tmp.x;
			ans.y += weight*tmp.y;
			ans.z += weight*tmp.z;
		}
		ans.x /= totNodes;//n->getLabelL(kNumAbstractedNodes); 
			ans.y /= totNodes;//n->getLabelL(kNumAbstractedNodes); 
				ans.z /= totNodes;//n->getLabelL(kNumAbstractedNodes); 
	}
	Map *mp = getMap();
	double r, a, b, c;
	mp->getOpenGLCoord(0,0,a,b,c,r);
	// height
	ans.z -= 5.0*r;
	//ans.z = -(double)5.0*r*(n->getLabelL(kAbstractionLevel)+1.1);
	
	n->setLabelF(kXCoordinate, ans.x);
	n->setLabelF(kYCoordinate, ans.y);
	n->setLabelF(kZCoordinate, ans.z);
	return ans;
}

void mapAbstraction::clearMarkedNodes()
{
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		edge_iterator ei = abstractions[x]->getEdgeIter();
		for (edge *e = abstractions[x]->edgeIterNext(ei); e; e = abstractions[x]->edgeIterNext(ei))
			e->setMarked(false);
	}
}

// estimate the cost from a to b
double mapAbstraction::h(node *a, node *b)
{
	if ((a == 0) || (b == 0))
		return 999999999.99;
	
	recVec rv1 = getNodeLoc(a);
	recVec rv2 = getNodeLoc(b);
	double answer = octileDistance((double)rv1.x,(double)rv1.y,(double)rv2.x,(double)rv2.y);
	//		if (fabs(rv1.x-rv2.x) < fabs(rv1.y-rv2.y)) {
	//			answer = root2m1*fabs(rv1.x-rv2.x)+fabs(rv1.y-rv2.y);
	//		} else {
	//			answer = root2m1*fabs(rv1.y-rv2.y)+fabs(rv1.x-rv2.x);
	//		}
	return answer*getMap()->getCoordinateScale();
}

/** Computes octile distance. No scaling */
double mapAbstraction::octileDistance(double x1, double y1, double x2, double y2)
{
	double answer = 0.0;
	const double root2m1 = ROOT_TWO-1;//sqrt(2.0)-1;
		if (fabs(x1-x2) < fabs(y1-y2))
			answer = root2m1*fabs(x1-x2)+fabs(y1-y2);
	else
		answer = root2m1*fabs(y1-y2)+fabs(x1-x2);
	return answer;
}




/**
* getMapGraph(map)
 *
 * Given a map, this function uses the external map interfaces to turn it
 * into a graph, and sets the appropriate node numbers for that map. This
 * function should not be called multiple times on the same map, because
 * the original graph map lose it's association with the map.
 */
graph *getMapGraph(Map *m)
{
	// printf("Getting graph representation of world\n");
	char name[32];
	graph *g = new graph();
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
				if (m->getTerrainType(x, y) == kOutOfBounds)
					continue;
				sprintf(name, "(%d, %d)", x, y);
				currTile.tile1.node = g->addNode(n = new node(name));
				n->setLabelL(kAbstractionLevel, 0); // level in abstraction tree
				n->setLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
				n->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
				n->setLabelF(kXCoordinate, kUnknownPosition);
				n->setLabelL(kNodeBlocked, 0);
				n->setLabelL(kFirstData, x);
				n->setLabelL(kFirstData+1, y);
				n->setLabelL(kFirstData+2, kNone);
			}
			else {
				if (m->getTerrainType(x, y, kLeftEdge) != kOutOfBounds)
				{
					sprintf(name, "(%d/%d)", x, y);
					currTile.tile1.node = g->addNode(n = new node(name));
					n->setLabelL(kAbstractionLevel, 0); // level in abstraction tree
					n->setLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
					n->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
					n->setLabelF(kXCoordinate, kUnknownPosition);
					n->setLabelL(kNodeBlocked, 0);
					n->setLabelL(kFirstData, x);
					n->setLabelL(kFirstData+1, y);
					if (currTile.split == kForwardSplit)
						n->setLabelL(kFirstData+2, kTopLeft);
					else
						n->setLabelL(kFirstData+2, kBottomLeft);
				}
				
				if (m->getTerrainType(x, y, kRightEdge) != kOutOfBounds)
				{
					sprintf(name, "(%d\\%d)", x, y);
					currTile.tile2.node = g->addNode(n = new node(name));
					n->setLabelL(kAbstractionLevel, 0); // level in abstraction tree
					n->setLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
					n->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
					n->setLabelF(kXCoordinate, kUnknownPosition);
					n->setLabelL(kNodeBlocked, 0);
					n->setLabelL(kFirstData, x);
					n->setLabelL(kFirstData+1, y);
					if (currTile.split == kForwardSplit)
						n->setLabelL(kFirstData+2, kBottomRight);
					else
						n->setLabelL(kFirstData+2, kTopRight);
				}
			}
		}
	}
	for (int y = 0; y < m->getMapHeight(); y++)
	{
		for (int x = 0; x < m->getMapWidth(); x++)
		{
			//cout << "Trying (x, y) = (" << x << ", " << y << ")" << endl;
			addMapEdges(m, g, x, y);
			//			if (!g->verifyGraph())
			//			{
			//				cerr << "Broken at (x, y) = (" << x << ", " << y << ")" << endl;
			//			}
		}
	}
	// printf("Done\n");
	
	// verify graph is correct
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
					cout << "(node " << n->getNum() << ")" << endl;
					break;
				}
			}			
		}
	}
#endif
	
	return g;
}

/**
* addMapEdges(map, graph, x, y)
 *
 * This is a helper function for getMapGraph that does the work of adding
 * the graph edges. Each edge is only added once, so while the graph has
 * directional edges, we treat them as being bidirectional.
 */
static const int gEdgeProb = 100;
static const int gStraightEdgeProb = 100;

void addMapEdges(Map *m, graph *g, int x, int y)
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
			if ((random()%100) < gStraightEdgeProb)
			{
				e = new edge(m->getTile(x, y).tile1.node, m->getTile(x-1, y).tile1.node, 1);
				g->addEdge(e);
			}
		}
		else if (m->getTile(x-1, y).tile2.node != kNoGraphNode)
		{
			if ((random()%100) < gStraightEdgeProb)
			{
				e = new edge(m->getTile(x, y).tile1.node, m->getTile(x-1, y).tile2.node, 1);
				g->addEdge(e);
			}
		}
		if (e)
			e->setLabelL(kEdgeCapacity, 1);
	}
	e = 0;
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
						if ((random()%100) < gStraightEdgeProb)
						{
							e = new edge(m->getTile(x, y).tile1.node, m->getTile(x, y-1).tile1.node, 1);
							g->addEdge(e);
						}
					}
				}
				else if (m->getTile(x, y-1).tile2.node != kNoGraphNode)
				{
					if ((random()%100) < gStraightEdgeProb)
					{
						e = new edge(m->getTile(x, y).tile1.node, m->getTile(x, y-1).tile2.node, 1);
						g->addEdge(e);
					}
				}
			}
		}
		else {
			if (m->adjacentEdges(x, y-1, kInternalEdge) || (m->getSplit(x, y-1) == kBackwardSplit))
			{
				if ((m->getTile(x, y).tile2.node != kNoGraphNode) && (m->getTile(x, y-1).tile1.node != kNoGraphNode))
				{
					if ((random()%100) < gStraightEdgeProb)
					{
						e = new edge(m->getTile(x, y).tile2.node, m->getTile(x, y-1).tile1.node, 1);
						g->addEdge(e);
					}
				}
			}
			else if ((m->getTile(x, y).tile2.node != kNoGraphNode) && (m->getTile(x, y-1).tile2.node != kNoGraphNode))
			{
				if ((random()%100) < gStraightEdgeProb)
				{
					e = new edge(m->getTile(x, y).tile2.node, m->getTile(x, y-1).tile2.node, 1);
					g->addEdge(e);
				}
			}
		}
		if (e)
			e->setLabelL(kEdgeCapacity, 1);
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
					if ((random()%100) < gEdgeProb)
					{
						e = new edge(m->getTile(x, y).tile1.node, m->getTile(x-1, y-1).tile1.node, ROOT_TWO);
						g->addEdge(e);
					}
				}
			}
			else if (m->getTile(x-1, y-1).tile2.node != kNoGraphNode)
			{
				if ((random()%100) < gEdgeProb)
				{
					e = new edge(m->getTile(x, y).tile1.node, m->getTile(x-1, y-1).tile2.node, ROOT_TWO);
					g->addEdge(e);
				}
			}
			if (e)
				e->setLabelL(kEdgeCapacity, 1);
		}
	}
	e = 0;
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
					if ((random()%100) < gEdgeProb)
					{
						e = new edge(m->getTile(x, y).tile1.node, m->getTile(x+1, y-1).tile1.node, ROOT_TWO);
						g->addEdge(e);
					}
				}
			}
			else if (m->getTile(x, y).tile2.node != kNoGraphNode)
			{
				if ((random()%100) < gEdgeProb)
				{
					e = new edge(m->getTile(x, y).tile2.node, m->getTile(x+1, y-1).tile1.node, ROOT_TWO);
					g->addEdge(e);
				}
			}
			if (e)
				e->setLabelL(kEdgeCapacity, 1);
		}
	}	
}
