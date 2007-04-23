/*
 *  loadedBBAbstraction.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 4/12/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
 *
 */

using namespace std;

#include "loadedBBAbstraction.h"
#include "fpUtil.h"
#include <math.h>
enum {
	kQuiet = 0x00,
	kBuildGraph = 0x01,
	kRepairGraph = 0x02,
	kMiscMessages = 0x04
};

const int verbose = kMiscMessages;//kMiscMessages;//kRepairGraph;

const double unknownPosition = -5.0;

bool BoundingBox::pointInBox(double x, double y, double z)
{
//	printf("checking (%lf, %lf, %lf) in (%lf, %lf, %lf)<=>(%lf, %lf, %lf)\n",
//				 x, y, z, x1, y1, z1, x2, y2, z2);
	return (((fless(x, x1) && (!fless(x, x2))) || (fless(x, x2) && (!fless(x, x1)))) &&
					((fless(y, y1) && (!fless(y, y2))) || (fless(y, y2) && (!fless(y, y1)))) &&
					((fless(z, z1) && (!fless(z, z2))) || (fless(z, z2) && (!fless(z, z1)))));
}

void BoundingBox::OpenGLDraw()
{
	glBegin(GL_LINE_LOOP);
	glVertex3f(x1, y1, z1);
	glVertex3f(x2, y1, z1);
	glVertex3f(x2, y2, z1);
	glVertex3f(x1, y2, z1);
	glEnd();
	glBegin(GL_LINE_LOOP);
	glVertex3f(x2, y2, z2);
	glVertex3f(x1, y2, z2);
	glVertex3f(x1, y1, z2);
	glVertex3f(x2, y1, z2);
	glEnd();
	glBegin(GL_LINES);
	glVertex3f(x1, y1, z1);
	glVertex3f(x1, y1, z2);
	glVertex3f(x2, y1, z1);
	glVertex3f(x2, y1, z2);
	glVertex3f(x1, y2, z1);
	glVertex3f(x1, y2, z2);
	glVertex3f(x2, y2, z1);
	glVertex3f(x2, y2, z2);
	glEnd();
}
//double x1, x2, y1, y2, z1, z2;

/**
 * Construct a new graph hierarchy.
 *
 * Constructions a new graph abstraction hierarchy from the graph using the
 * designated abstraction method.
 */
loadedBBAbstraction::loadedBBAbstraction(char *fname, char *boxFile)
:graphAbstraction()
{
	levelDraw = 1;

	loadBoxes(boxFile);
	buildAbstractions(loadGraph(fname));
}

loadedBBAbstraction::~loadedBBAbstraction()
{
  cleanMemory();
}

void loadedBBAbstraction::loadBoxes(char *boxFile)
{
	FILE *f = fopen(boxFile, "r");
	if (f)
	{
		while (!feof(f))
		{
			//printf("Loading bounding box %u\n", boxes.size());
			BoundingBox b;
			int num;
			fscanf(f, "%d %lf %lf %lf %lf %lf %lf\n",
						 &num, &b.x1, &b.y1, &b.z1, &b.x2, &b.y2, &b.z2);
			boxes.push_back(b);
		}
		fclose(f);
	}
	else {
		printf("Unable to open box file %s\n", boxFile);
		exit(2);
	}
}

double loadedBBAbstraction::h(node *a, node *b)
{
	double d1 = a->getLabelF(kXCoordinate)-b->getLabelF(kXCoordinate);
	double d2 = a->getLabelF(kYCoordinate)-b->getLabelF(kYCoordinate);
	double d3 = a->getLabelF(kZCoordinate)-b->getLabelF(kZCoordinate);
	return sqrt(d1*d1+d2*d2+d3*d3);
}

void loadedBBAbstraction::toggleDrawAbstraction(int which)
{
  bool drawThis = ((levelDraw>>which)&0x1);
  if (!drawThis)
    levelDraw |= (1<<which);
  else
    levelDraw = levelDraw&(~(1<<which));
}

void loadedBBAbstraction::OpenGLDraw()
{
	glDisable(GL_LIGHTING);
  for (unsigned int x = 0; x < abstractions.size(); x++)
	{
    if ((levelDraw >> x) & 1) drawGraph(abstractions[x]);
    //glCallList(displayLists[x]);
  }
	if (levelDraw&1)
	{
		for (unsigned int x = 0; x < boxes.size(); x++)
		{
			glColor3f(0, 0, 1);
			boxes[x].OpenGLDraw();
		}
	}
	glEnable(GL_LIGHTING);
}

void loadedBBAbstraction::drawGraph(graph *g)
{
		if ((g == 0) || (g->getNumNodes() == 0)) return;
		
		int abLevel = g->getNode(0)->getLabelL(kAbstractionLevel);	
		
		glBegin(GL_LINES);
		glNormal3f(0, 1, 0);
		node_iterator ni;
		edge_iterator ei = g->getEdgeIter();
		for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei)) {
			node *n;
			n = g->getNode(e->getFrom());
			
			if (e->getLabelL(kEdgeCapacity) == 0)      glColor4f(.5, .5, .5, 1);
			else if (e->getLabelL(kEdgeCapacity) <= 0) glColor4f(.2, .2, .2, 1);
			else if (e->getMarked())                  glColor4f(1, 1, 1, 1);
			else if (abLevel%2)
				glColor4f(1-((GLfloat)(abLevel%15)/15.0), ((GLfloat)(abLevel%15)/15.0), 0, 1);
			else glColor4f(((GLfloat)(abLevel%15)/15.0), 1-((GLfloat)(abLevel%15)/15.0), 0, 1);
			recVec rv = getNodeLoc(n);
			glVertex3f(rv.x, rv.y, rv.z);
			
			n = g->getNode(e->getTo());
			rv = getNodeLoc(n);
			
			glVertex3f(rv.x, rv.y, rv.z);
		}
		ni = g->getNodeIter();
		for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
			drawLevelConnections(n);
		glEnd();
		//  if (verbose&kBuildGraph) printf("Done\n");
}

void loadedBBAbstraction::drawLevelConnections(node *n)
{
  //	int x, y;
  //	double offsetx, offsety;
  //	recVec ans;
  //if (n->getNumOutgoingEdges()+n->getNumIncomingEdges() == 0) return;
	
  if (n->getLabelL(kAbstractionLevel) == 0) return;
  else {
    glColor4f(.6, .6, .6, .6);
    recVec v = getNodeLoc(n);
    for (int cnt = 0; cnt < n->getLabelL(kNumAbstractedNodes); cnt++) {
      recVec v1 = getNodeLoc(abstractions[n->getLabelL(kAbstractionLevel)-1]->getNode(n->getLabelL(kFirstData+cnt)));
      glVertex3f(v.x, v.y, v.z);
      glVertex3f(v1.x, v1.y, v1.z);
    }
  }
  //return ans;
}

recVec loadedBBAbstraction::getNodeLoc(node *n)
{
	//  double offsetx, offsety;
  recVec ans;
	
  if (n->getLabelF(kXCoordinate) != unknownPosition) {
    ans.x = n->getLabelF(kXCoordinate);
    ans.y = n->getLabelF(kYCoordinate);
    ans.z = n->getLabelF(kZCoordinate);
    return ans;
  }
	
	int totNodes = 0;
	ans.x = ans.y = ans.z = 0;
	for (int cnt = 0; cnt < n->getLabelL(kNumAbstractedNodes); cnt++) {
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

  n->setLabelF(kXCoordinate, ans.x);
  n->setLabelF(kYCoordinate, ans.y);
  n->setLabelF(kZCoordinate, ans.z);
  return ans;
}


graph *loadedBBAbstraction::loadGraph(char *fname)
{
	FILE *f = fopen(fname, "r");
	if (!f)
	{
		printf("Error opening %s for loading\n", fname);
		exit(1);
	}
	char nextLine[255];
	std::vector<unsigned int> IDS;
	bool nodes = true;
	graph *g = new graph();
	fgets(nextLine, 255, f);
	const double VERSION = 1.0;
	double version;
	sscanf(nextLine, "VERSION %lf", &version);
	if (version != VERSION)
	{
		printf("Got %lf; code can only handle version 1.0\n", version);
		exit(1);
	}
	
	while ((!feof(f)) && fgets(nextLine, 255, f))
	{
		if (nextLine[0] == '#')
			continue;
		if (strncmp("edges", nextLine, 5) == 0)
		{
			nodes = false;
			continue;
		}
		if (nodes)
		{
			// ID x y z
			int ID;
			double x, y, z;
			sscanf(nextLine, "%d %lf %lf %lf", &ID, &x, &y, &z);
			//printf("Loaded node %d\n", ID);
			if (IDS.size() <= (unsigned int)ID)
				IDS.resize(ID+1);
			node *n;
			IDS[ID] = g->addNode(n = new node("l1"));
			n->setLabelL(kAbstractionLevel, 0); // level in abstraction tree
			n->setLabelL(kNumAbstractedNodes, 1); // number of abstracted nodes
			n->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			n->setLabelF(kXCoordinate, x);
			n->setLabelF(kYCoordinate, y);
			n->setLabelF(kZCoordinate, z);
			n->setLabelL(kNodeBlocked, 0);
		}
		else {
			int ID1, ID2;
			sscanf(nextLine, "%d %d", &ID1, &ID2);
			g->addEdge(new edge(IDS[ID1], IDS[ID2], h(g->getNode(ID1), g->getNode(ID2))));
			//printf("Loaded edge between %d and %d\n", ID1, ID2);
		}
	}
	return g;
}

void loadedBBAbstraction::verifyHierarchy()
{
	cout << "VERIFY START" << endl;
	for (unsigned int x = 0; x < abstractions.size(); x++)
	{
		// first make sure graph is ok
		abstractions[x]->verifyGraph();
		// then make sure abstraction is ok
	  node_iterator ni = abstractions[x]->getNodeIter();
		for (node *n = abstractions[x]->nodeIterNext(ni); n; n = abstractions[x]->nodeIterNext(ni))
		{
			// verify graph, because there may be issues...
			if (n->getLabelL(kParent) != -1)
			{
				graph *g = abstractions[x+1];
				node *parent = g->getNode(n->getLabelL(kParent));
				bool found = false;
				for (int y = 0; y < parent->getLabelL(kNumAbstractedNodes); y++)
				{
					if (parent->getLabelL(kFirstData+y) == (long)n->getNum())
					{ found = true; break; }
				}
				if (!found)
				{
					cout << "VERIFY: Graph doesn't verify; child:" << endl << *n << endl;
					cout << "VERIFY: Graph doesn't verify; parent:" << endl << *parent << endl;
				}
			}
			if (x > 0)
			{
				graph *g = abstractions[x-1];
				for (int y = 0; y < n->getLabelL(kNumAbstractedNodes); y++)
				{
					node *child = g->getNode(n->getLabelL(kFirstData+y));
					if (!child)
					{
						cout << "VERIFY: Graph doesn't verify; CHILD is null, parent:" << endl << *n << endl;
					}
					else if (child->getLabelL(kParent) != (long)n->getNum())
					{
						cout << "VERIFY: Graph doesn't verify; parent:" << endl << *n << endl;
						cout << "VERIFY: Graph doesn't verify; child:" << endl << *child << endl;
					}
				}
			}
			else {
//				int x1, y1;
//				getTileFromNode(n, x1, y1);
//				if (n != getNodeFromMap(x1, y1))
//					cout << "VERIFY: node doesn't correspond to underlying map" << endl << *n << endl;
			}
		}
		// verify edges
		edge_iterator ei = abstractions[x]->getEdgeIter();
		for (edge *e = abstractions[x]->edgeIterNext(ei); e; e = abstractions[x]->edgeIterNext(ei))
		{
			node *p1, *p2;
			p1 = findNodeParent(abstractions[x]->getNode(e->getFrom()));
			p2 = findNodeParent(abstractions[x]->getNode(e->getTo()));
			if (p1 == p2)
				continue;
			if ((p1 == 0) || (p2 == 0))
			{
				cout << "VERIFY: One edge parent is null, and the other isn't " << *e << endl << *p1 << endl << *p2 << endl;
				continue;
			}
			if (!abstractions[x+1]->findEdge(p1->getNum(), p2->getNum()))
			{
				cout << "Didn't find parent edge of " << *e << " at abslevel " << x << endl;
				cout << *p1 << endl << *p2 << endl;
			}
			// VERIFY edge weights
			if (e->getLabelL(kEdgeCapacity) == 0)
				cout << "VERIFY: Edge capacity is 0?!? " << e << endl;
			if (x > 0) // we can verify the capacity
			{
				int count = 0;
				// we should find kEdgeCapacity edges between the children of p1 and p2
				p1 = abstractions[x]->getNode(e->getFrom());
				p2 = abstractions[x]->getNode(e->getTo());
				for (int c1 = 0; c1 < p1->getLabelL(kNumAbstractedNodes); c1++)
				{
					for (int c2 = 0; c2 < p2->getLabelL(kNumAbstractedNodes); c2++)
					{
						if (abstractions[x-1]->findEdge(p1->getLabelL(kFirstData+c1),
																						p2->getLabelL(kFirstData+c2)))
						{
							count++;
						}
					}
				}
				if (count != e->getLabelL(kEdgeCapacity))
				{
					cout << "VERIFY: Edge capactiy of " << *e << " is "
					<< e->getLabelL(kEdgeCapacity) << " but we only found " << count
					<< " edges below that abstract into it." << endl;
				}
			}
		}
	}
	cout << "VERIFY END" << endl;
}

void loadedBBAbstraction::cleanMemory()
{
  while (abstractions.size() > 0) {
    delete abstractions.back();
    abstractions.pop_back();
  }
//	clearDisplayLists();
//	while (displayLists.size() > 0)
//		displayLists.pop_back();
}

//void loadedBBAbstraction::clearDisplayLists()
//{
//  for (unsigned int x = 0; x < displayLists.size(); x++) {
//    if (displayLists[x] != 0) glDeleteLists(displayLists[x], 1);
//    displayLists[x] = 0;
//  }
//}

void loadedBBAbstraction::buildAbstractions(graph *_g)
{
	int totalNodes = 0;
  cleanMemory();
	abstractions.push_back(_g);
  graph *g = abstractions[0];
//	if (displayLists.size() != 1)
//		displayLists.push_back(0);
  //if (verbose)
      printf("Base graph (0) has %d nodes\n", g->getNumNodes());
			g->printStats();
	
  for (int x = 1; ; x++)
	{
    if (g->getNumEdges() == 0) break;

    if (verbose&kMiscMessages) printf("Building abstraction #%2d\n", x);

    abstractions.push_back(abstractGraph(g));
		g = abstractions.back();

    if (verbose&kMiscMessages)
		{
      printf("Abstract graph #%2d has %d nodes\n", x, g->getNumNodes());
			g->printStats();
		}
		totalNodes += g->getNumNodes();
//    displayLists.push_back(0);
  }
	// printf("%d nodes, excluding bottom level", totalNodes);
}

graph *loadedBBAbstraction::abstractGraph(graph *g)
{
	// for each node:
	// 1. find bounding box
	// 2. add node and all neighbors which are in the bounding box
	node_iterator ni = g->getNodeIter();
	node *newNode;
	graph *aGraph = new graph();

	ni = g->getNodeIter();
	for (node *n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	{
		if (n->getLabelL(kParent) != -1) continue;
		int which;
		if (n->getLabelL(kAbstractionLevel) == 0)
			which = findBoundingBox(n);
		else
			which = -1;
		//printf("%u abstracted into box %d\n", n->getNum(), which);
		newNode = createNewParent(aGraph, n);
		//printf("%u created as parent of %u\n", newNode->getNum(), n->getNum());
		if ((which == -1) && (n->getLabelL(kAbstractionLevel) == 0))
			addNodeToParent(n, newNode);
		else
			addNeighborsInBox(g, n, which, newNode);
	}
	
	// now add all the edges
	edge_iterator ei = g->getEdgeIter();
	for (edge *e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		int from = g->getNode(e->getFrom())->getLabelL(kParent);
		int to = g->getNode(e->getTo())->getLabelL(kParent);
		edge *f=0;
		
		if ((from != to) && (!(f = aGraph->findEdge(to, from))))
		{
			double weight = h(aGraph->getNode(from), aGraph->getNode(to));
			f = new edge(from, to, weight);
			f->setLabelL(kEdgeCapacity, 1);
			aGraph->addEdge(f);
			//printf("Adding edge to graph!\n");
		}
		else if (f) f->setLabelL(kEdgeCapacity, f->getLabelL(kEdgeCapacity)+1);
		//		else if (g)
		//			g->setLabel(kEdgeCapacity, g->getLabelL(kEdgeCapacity)+1);
	}
	 
	return aGraph;
}

int loadedBBAbstraction::findBoundingBox(node *n)
{
	for (unsigned int x = 0; x < boxes.size(); x++)
	{
		if (boxes[x].pointInBox(n->getLabelF(kXCoordinate),
																					 n->getLabelF(kYCoordinate),
																					 n->getLabelF(kZCoordinate)))
			return x;
//		if (boxes[boxes.size()-x-1].pointInBox(n->getLabelF(kXCoordinate),
//																					 n->getLabelF(kYCoordinate),
//																					 n->getLabelF(kZCoordinate)))
//			return boxes.size()-x-1;
	}
	return -1;
}

void loadedBBAbstraction::addNeighborsInBox(graph *g, node *n, int which, node *parent)
{
	//printf("Thinking about adding %d to parent %d\n", n->getNum(), parent->getNum());
	if (n->getLabelL(kParent) != -1)
		return;
	if ((which != -1) && (!boxes[which].pointInBox(n->getLabelF(kXCoordinate),
																								 n->getLabelF(kYCoordinate),
																								 n->getLabelF(kZCoordinate))))
		return;

	addNodeToParent(n, parent);
	neighbor_iterator nbi = n->getNeighborIter();
	for (node *next = g->getNode(n->nodeNeighborNext(nbi));
			 next; next = g->getNode(n->nodeNeighborNext(nbi)))
		addNeighborsInBox(g, next, which, parent);
}

void loadedBBAbstraction::addNodeToParent(node *n, node *parent)
{
	//printf("Adding %d to parent %d\n", n->getNum(), parent->getNum());
	n->setLabelL(kParent, parent->getNum());
	parent->setLabelL(kFirstData+parent->getLabelL(kNumAbstractedNodes), n->getNum());
	parent->setLabelL(kNumAbstractedNodes, parent->getLabelL(kNumAbstractedNodes)+1); // number of abstracted nodes
}

node *loadedBBAbstraction::createNewParent(graph *g, node *n)
{
	node *newNode = new node("l1");
	g->addNode(newNode);
	newNode->setLabelL(kAbstractionLevel, n->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
	newNode->setLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
	newNode->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
	newNode->setLabelF(kXCoordinate, unknownPosition);
	newNode->setLabelL(kNodeBlocked, 0);

	return newNode;
}

bool loadedBBAbstraction::pathable(unsigned int from, unsigned int to)
{
	return pathable(abstractions[0]->getNode(from), abstractions[0]->getNode(to));
}

bool loadedBBAbstraction::pathable(node *from, node *to)
{
  //printf("At nodes #%d and %d\n", from->getNum(), to->getNum());
  while (from != to) {
    if ((!from) || (!to) ||
				(abstractions[from->getLabelL(kAbstractionLevel)]->getNumEdges() == 0))
      return false;
		
    from = abstractions[from->getLabelL(kAbstractionLevel)+1]->
      getNode(from->getLabelL(kParent));
    to = abstractions[to->getLabelL(kAbstractionLevel)+1]->
      getNode(to->getLabelL(kParent));
  }
	if ((from == 0) || (to == 0))
		return false;
	return true;
}


void loadedBBAbstraction::addNode(node *n)
{
}

void loadedBBAbstraction::addEdge(edge *e, unsigned int absLevel)
{
}

//	// for now we'll immediately handle splits, but in the future we should queue up splits
//// and process them in batch form(?)
//void loadedBBAbstraction::removeEdge(edge *e, unsigned int absLevel)
//{
//  return;
//}
//

node *loadedBBAbstraction::findNodeParent(node *n)
{
  unsigned int absLevel = n->getLabelL(kAbstractionLevel);
  if (absLevel < abstractions.size()-1)
    return abstractions[absLevel+1]->getNode(n->getLabelL(kParent));
  return 0;
}
