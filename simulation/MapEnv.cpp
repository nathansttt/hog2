///**
// * $Id: mapenv.cpp,v 1.2 2006/09/18 06:20:50 nathanst Exp $
// * 
// * Using the pathfinding library, this class will extend the environment
// * class so that the various pathfinding methods can be used.
// *
// * Uses Nathan's Graph class as a base.
// *
// * Copyright (c) Brandon Blanck
// * May 7, 2004
// */
//
//#include "Mapenv.h"
//#include "Pathfind.h"
//
//using namespace std;
//
//#define COST_ONE       10000000
//#define COST_SQRT2     14142135
//#define MULTIPLY_CONST 10000000
//
///**
// * Checks whether or not the given node is an obstacle
// *
// * Returns true if it's an obstacle, false otherwise.
// */
//bool MapEnv::isObstacle(node* n) const
//{
//	int x = n->GetLabelL(kFirstData);
//	int y = n->GetLabelL(kFirstData+1);
//	
//	int terrain = m_map->getTerrainType(x, y);
//	if (terrain == kGround)
//		return false;
//		
//	terrain = m_map->getTerrainType(x, y, kLeftSide);
//	if (terrain == kGround)
//		return false;
//	
//	terrain = m_map->getTerrainType(x, y, kRightSide);
//		if (terrain == kGround)
//			return false;
//
//	return true;
//}
//
//
///**
// * When finished, start and target will hold two valid, possibly
// * Pathable positions.  That is, the positions will not be an obstacle.
// */
//void MapEnv::getStartTarget(int* start, int* target) const
//{
//	node* s;
//	node* t;
//	
//	do {
//		s = m_graph->getRandomNode();
//		t = m_graph->getRandomNode();
//		
//	} while(isObstacle(s) || isObstacle(t) 
//		|| !isValidNodeId(s->GetNum())
//		|| !isValidNodeId(t->GetNum())
//		|| s->GetNum() == t->GetNum());
//	
//	*start = s->GetNum();
//	*target = t->GetNum();
//}
//
//	
///**
// * Returns the heuristic of the Graph.
// *
// * This is based off of an Octile Graph.
// */
//int MapEnv::getHeuristic(int start, int target) const 
//{ 
//	if (m_abstrMap) {
//		node* a = m_graph->GetNode(start);
//		node* b = m_graph->GetNode(target);
//		
//		return (int) (m_abstrMap->h(a, b) * MULTIPLY_CONST);
//	}
//	
//	int colStart = start % m_columns;
//    	int colTarget = target % m_columns;
//    	int rowStart = start / m_columns;
//    	int rowTarget = target / m_columns;
//    	int diffCol = abs(colTarget - colStart);
//    	int diffRow = abs(rowTarget - rowStart);
//
//        int maxDiff;
//        int minDiff;
//        if (diffCol > diffRow)
//        {
//            maxDiff = diffCol;
//            minDiff = diffRow;
//        }
//        else
//        {
//            maxDiff = diffRow;
//            minDiff = diffCol;
//        }
//        return minDiff * COST_SQRT2 + (maxDiff - minDiff) * COST_ONE;
//}
//		
///**
// * Returns the maximum cost of an edge.
// *
// * At the moment, this is COST_SQRT2, since the terrain isn't taken into count.
// * If terrain is taken into effect, this will change depending on terrain type.
// */
//int MapEnv::getMaxCost() const 
//{ 
//	return COST_SQRT2;
//}
//
//
///**
// * Returns the minimum cost of an edge.
// *
// * At the moment, this is COST_ONE, since the terrain isn't taken into count.
// * If terrain is taken into effect, this will change depending on terrain type.
// */		
//int MapEnv::getMinCost() const 
//{ 
//	return COST_ONE;
//}
//		
//
///**
// * Returns the number of nodes in the Graph.
// */
//int MapEnv::getNumberNodes() const 
//{
//	return m_graph->getNumNodes();
//}
//
//			
//void MapEnv::getSuccessors(int nodeId, int lastNodeId, vector<Successor>& result,
//			   bool /*usePruning*/) const 
//{
//	//result.reserve(8); // maximum 8 edges
//	result.clear();
//	node* n = m_graph->GetNode(nodeId);
//	
//	if (isObstacle(n))
//	{
//		assert(result.size() == 0);
//		return;
//	}
//	
//// 	edge_iterator i = n->getOutgoingEdgeIter();
//// 	
//// 	for (edge* e = n->edgeIterNextOutgoing(i); e; e = n->edgeIterNextOutgoing(i))
////     	{
////         	int targetNodeId = e->getTo();
////         	assert(isValidNodeId(targetNodeId));
////         	node* targetNode = m_graph->GetNode(targetNodeId);
//// 
////         	if ((targetNodeId == lastNodeId) || // don't revisit departed node.
//// 	 	    (isObstacle(targetNode))) // Obstacles are not passable
////            		continue;
//// 		
////         	result.push_back(Successor(targetNodeId, (int)(e->getWidth() * MULTIPLY_CONST)));
////     	}
//	
//	edge_iterator i = n->getEdgeIter();
//	
//	for (edge* e = n->edgeIterNext(i); e; e = n->edgeIterNext(i))
//    	{
//        	int targetNodeId;
//		if (e->getTo() == n->GetNum())
//			targetNodeId = e->getFrom();
//		else
//			targetNodeId = e->getTo();
//		
//		//int targetNodeId = e->getTo();
//        	assert(isValidNodeId(targetNodeId));
//        	node* targetNode = m_graph->GetNode(targetNodeId);
//
//        	if ((targetNodeId == lastNodeId) || // don't revisit departed node.
//	 	    (isObstacle(targetNode))) // Obstacles are not passable
//           		continue;
//		
//        	result.push_back(Successor(targetNodeId, (int)(e->getWeight() * MULTIPLY_CONST)));
//    	}
//	
//	
//	
//	//if (result.size() <= 0)
//	//	printf("Successor size: %d, Number outgoing edges: %d, Incoming: %d\n", result.size(), n->getNumOutgoingEdges(), n->getNumIncomingEdges());
//}
//
//	
//bool MapEnv::isValidNodeId(int nodeId) const 
//{ 
//	return m_graph->GetNode(nodeId);
//}
//
//
//
///**
// * A series of functions to print out the map/path.
// * These are not perfect, and should really only
// * be used for debugging.
// */
//vector<char> MapEnv::getCharVector() const
//{
//    vector<char> result;
//    int numberNodes = getNumberNodes();
//    result.reserve(numberNodes);
//    for (int i = 0; i < numberNodes; ++i)
//    {
//        if (isObstacle(m_graph->GetNode(i)))
//            result.push_back('@');
//	else
//            result.push_back('.');
//    }
//    return result;
//}
//
//
//void MapEnv::printFormatted(ostream& o, const vector<char>& chars) const
//{
//	int i = 0;
//
//    for (int row = 0; row < m_rows; ++row)
//    {
//        for (int col = 0; col < m_columns; ++col)
//        {
//            //int nodeId = getNodeId(row, col);
//            //o << chars[nodeId];
//	    o << chars[i];
//	    i++;
//        }
//        o << '\n';
//    }
//}
//
//
//void MapEnv::printFormatted(ostream& o, const vector<int>& path) const
//{
//    vector<char> chars = getCharVector();
//    if (path.size() > 0)
//    {
//        for (vector<int>::const_iterator i = path.begin();
//             i != path.end(); ++i)
//            chars[*i] = 'x';
//        chars[*path.begin()] = 'T';
//        chars[*(path.end() - 1)] = 'S';
//    }
//    printFormatted(o, chars);
//}
