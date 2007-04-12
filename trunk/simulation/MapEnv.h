///**
// * $Id: mapenv.h,v 1.2 2005/06/07 00:00:46 nathanst Exp $
// *
// * HOG File
// *
// * Using the pathfinding library, this class will extend the environment
// * class so that the various pathfinding methods can be used.
// *
// * Uses Nathan's map and graph classes as a base.
// *
// * Copyright (c) Brandon Blanck
// * May 7, 2004
// */
//
//#ifndef MAPENV_H
//#define MAPENV_H
//
//#include <vector>
//#include "environment.h"
//#include "mapAbstraction.h"
//
//
//class MapEnv : public PathFind::Environment
//{
//public:
//  /*MapEnv(Map* m, int rows, int columns)
//    : m_map(m), m_graph(m->getGraph()), m_rows(rows), m_columns(columns) {}
//  */
//  MapEnv(Map* m)
//    : m_abstrMap(0), m_map(m), m_graph(getMapGraph(m)), m_rows(m->getMapHeight()),
//      m_columns(m->getMapWidth()) {}
//	
//  MapEnv(Map* m, graph* g)
//    : m_abstrMap(0), m_map(m), m_graph(g), m_rows(m->getMapHeight()),
//      m_columns(m->getMapWidth()) {}
//	
//  MapEnv(mapAbstraction* am, Map* m, graph* g)
//    : m_abstrMap(am), m_map(m), m_graph(g), m_rows(m->getMapHeight()),
//      m_columns(m->getMapWidth()) {}
//	
//  ~MapEnv() { }
//	
//  bool isObstacle(node* n) const;
//	
//  void getStartTarget(int* start, int* target) const;
//	
//  graph* getGraph() { return m_graph; }
//	
//  std::vector<char> getCharVector() const;
//	
//  void printFormatted(std::ostream& o, const std::vector<char>& chars) const;
//	
//  void printFormatted(std::ostream& o, const std::vector<int>& path) const;
//	
//  // From the Environment class
//  int getHeuristic(int start, int target) const;
//       
//  int getMaxCost() const;
//        
//  int getMinCost() const;
//        
//  int getNumberNodes() const;
//		
//  void getSuccessors(int nodeId, int lastNodeId,
//		     std::vector<Successor>& result,
//		     bool usePruning = false) const;
//        
//  bool isValidNodeId(int nodeId) const;
//
//	
//private:
//
//  mapAbstraction* m_abstrMap;
//  Map* m_map;
//  graph* m_graph;
//  int m_rows;
//  int m_columns;
//};
//
//
//#endif
