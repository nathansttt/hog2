/*
 *  LoadedBBAbstraction.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 4/12/06.
 *  Copyright 2006 Nathan Sturtevant. All rights reserved.
 *
 */

#include "GraphAbstraction.h"
#include "GLUtil.h"

#ifndef LOADEDBBABSTRACTION_H
#define LOADEDBBABSTRACTION_H

class BoundingBox {
public:
	bool pointInBox(double x, double y, double z);
	void OpenGLDraw();
	double x1, x2, y1, y2, z1, z2;
};

class LoadedBBAbstraction : public GraphAbstraction {
public:
  LoadedBBAbstraction(char *, char *);
  virtual ~LoadedBBAbstraction();
	virtual void verifyHierarchy();
  double h(node *a, node *b);
	//  virtual void draw();
	//  virtual void rebuild();
	//	virtual int getRevision() { return 0; }
	
  bool Pathable(node *from, node *to);
  bool Pathable(unsigned int from, unsigned int to);
	//path *getQuickPath(node *from, node *to);
  void getParentHierarchy(node *from, node *to, std::vector<node *> &fromChain, std::vector<node *> &toChain);
  //node *getNthParent(node *, int n);
  
  //void clearMarkedNodes();
  void clearDisplayLists();
	void OpenGLDraw();
	void toggleDrawAbstraction(int which);
	void drawLevelConnections(node *n);
	void drawGraph(graph *g);
	recVec getNodeLoc(node *n);
	
  // first step to allowing simple loaded adjustments...
//  virtual void removeNode(node *n);
//  void removeEdge(edge *e, unsigned int absLevel);
	virtual void addNode(node *n);
	virtual void addEdge(edge *e, unsigned int absLevel);
  // this must be called after removeEdge, but removeNode will take care of itself
  //void repairAbstraction();
  node *findNodeParent(node *n);
  edge *findEdgeParent(edge *e, unsigned int absLevel);
private:
		std::vector<BoundingBox> boxes;
	graph *abstractGraph(graph *g);
	void cleanMemory();
	void buildAbstractions(graph *g);
	void loadBoxes(char *boxFile);
	graph *loadGraph(char *fname);
	node *createNewParent(graph *, node *n);
		void addNodeToParent(node *n, node *parent);
		void addNeighborsInBox(graph *g, node *n, int which, node *parent);
		int findBoundingBox(node *n);
		unsigned long levelDraw;

		virtual void removeNode(node *) { assert(false); }
		/** remove edge from abstraction */
		virtual void removeEdge(edge *, unsigned int) { assert(false); }
		virtual void repairAbstraction() { }
};

#endif
