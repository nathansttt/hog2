/*
 * $Id: loadedCliqueAbstraction.h,v 1.4 2006/09/18 06:22:14 nathanst Exp $
 *
 *  loadedCliqueAbstraction.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 3/10/06.
 *  Copyright 2006 Nathan Sturtevant, University of Alberta. All rights reserved.
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

#include "graphAbstraction.h"
#include "glUtil.h"

#ifndef loadedCLIQUEABSTRACTION_H
#define loadedCLIQUEABSTRACTION_H

/**
* A loaded abstraction based on the reduction of cliques.
 */

class loadedCliqueAbstraction : public graphAbstraction {
public:
  loadedCliqueAbstraction(char *);
  virtual ~loadedCliqueAbstraction();
	virtual void verifyHierarchy();
  double h(node *a, node *b);
	//  virtual void draw();
	//  virtual void rebuild();
	//	virtual int getRevision() { return 0; }
	
  bool pathable(node *from, node *to);
  bool pathable(unsigned int from, unsigned int to);
	//path *getQuickPath(node *from, node *to);
  void getParentHierarchy(node *from, node *to, std::vector<node *> &fromChain, std::vector<node *> &toChain);
  //node *getNthParent(node *, int n);
  
  void clearDisplayLists();
	void openGLDraw();
	void toggleDrawAbstraction(int which);
	void drawLevelConnections(node *n);
	void drawGraph(graph *g);
	recVec getNodeLoc(node *n);

  // first step to allowing simple loaded adjustments...
  virtual void removeNode(node *n);
  void removeEdge(edge *e, unsigned int absLevel);
	virtual void addNode(node *n);
	virtual void addEdge(edge *e, unsigned int absLevel);
  // this must be called after removeEdge, but removeNode will take care of itself
  void repairAbstraction();
  node *findNodeParent(node *n);
  edge *findEdgeParent(edge *e, unsigned int absLevel);
	
	//	virtual int getNumPrimitiveActions() = 0;
	//	virtual node *getNodeForPrimitiveAction(node *, int) = 0;
protected:
	graph *loadGraph(char *fname);
	virtual graph *abstractGraph(graph *g);
	virtual graph *cliqueAbstractGraph(graph *g);
	virtual graph *neighborAbstractGraph(graph *g, int width = 1);
	void addNodesToParent(graph *g, node *n, node *parent, int width);
  void addTunnel(node *n, graph *g, node *newNode);
  void renameNodeInAbstraction(node *which, unsigned int oldID);
  int getChildGroups(node *which);
  void splitNode(node *which, int numGroups);
	void addNodeToRepairQ(node *n);
	void removeNodeFromRepairQ(node *n);
	//void removeDiagonalEdgesAroundNode(node *n);
	
  // helper functions for splitting nodes
	void resetLocationCache(node *n);
	void checkAndCreateParent(node *which);
	void insertNodeIntoHierarchy(node *newNode);
  //void transferNode(node *which, node *oldParent, node *newParent, int whichChild);
	void transferGroup(int group, node *oldParent, node *newParent);
	void abstractUpEdge(unsigned int absLevel, edge *e);
  void extractGroupIntoNewNode(node *parent, int group);
  void mergeGroupIntoNeighbor(node *parent, int group, node *neighbor = 0);
  bool checkNeighborClique(node *child, node *neighbor);
  node *findNeighborCliques(node *parent, int group);
  node *findNeighborCliques(node *);
  int getGroupSize(node *parent, int group);
  node *getNodeInGroup(node *parent, int group);
private:
		unsigned long levelDraw;

		void buildAbstractions(graph*);
  void cleanMemory();
	
  std::vector<GLuint> displayLists;
  // temporary Q for nodes that are modified in the abstraction
  // we can then process them for changes all at once.
  std::vector<node *> modifiedNodeQ;
};

#endif
