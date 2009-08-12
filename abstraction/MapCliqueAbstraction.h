/*
 * $Id: MapCliqueAbstraction.h,v 1.10 2007/03/07 21:53:55 nathanst Exp $
 *
 *  MapCliqueAbstraction.h
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

#include "MapAbstraction.h"

#ifndef MAPCLIQUEABSTRACTION_H
#define MAPCLIQUEABSTRACTION_H

/**
 * A map abstraction based on the reduction of cliques.
 */

class MapCliqueAbstraction : public MapAbstraction {
public:
  MapCliqueAbstraction(Map *, bool uniform = true);
  virtual ~MapCliqueAbstraction();
	/** return a new abstraction map of the same type as this map abstraction */
	virtual MapAbstraction *Clone(Map *_m) { return new MapCliqueAbstraction(_m); }
	virtual void VerifyHierarchy();
//  virtual void draw();
//  virtual void rebuild();
//	virtual int GetRevision() { return 0; }
	
  bool Pathable(node *from, node *to);
  bool Pathable(unsigned int from, unsigned int to);
	//path *getQuickPath(node *from, node *to);
  void GetNumAbstractGraphs(node *from, node *to, std::vector<node *> &fromChain, std::vector<node *> &toChain);
  //node *GetNthParent(node *, int n);
  
  void ClearMarkedNodes();
  void clearDisplayLists();
	
  // first step to allowing simple map adjustments...
  virtual void RemoveNode(node *n);
  void RemoveEdge(edge *e, unsigned int absLevel);
	virtual void AddNode(node *n);
	virtual void AddEdge(edge *e, unsigned int absLevel);
  // this must be called after RemoveEdge, but RemoveNode will take care of itself
  void RepairAbstraction();
  node *findNodeParent(node *n);
  edge *findEdgeParent(edge *e, unsigned int absLevel);
	
	int MeasureRepairHits();
//	virtual int getNumPrimitiveActions() = 0;
//	virtual node *getNodeForPrimitiveAction(node *, int) = 0;
protected:
		virtual Graph *abstractGraph(Graph *g);
	virtual Graph *cliqueAbstractGraph(Graph *g);
	//virtual Graph *neighborAbstractGraph(Graph *g, int width = 1);
	void addNodesToParent(Graph *g, node *n, node *parent, int width);
  void addTunnel(node *n, Graph *g, node *newNode);
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
	void buildAbstractions();
  void cleanMemory();
	
#ifdef INSTRUMENT_REPAIR
	std::vector<int> hit;
#endif
  std::vector<GLuint> displayLists;
  // temporary Q for nodes that are modified in the abstraction
  // we can then process them for changes all at once.
  std::vector<node *> modifiedNodeQ;
	bool abstractUniformly;
};

#endif
