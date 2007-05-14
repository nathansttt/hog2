/* 
 * $Id: ClusterAbstraction.cpp,v 1.12 2007/03/06 22:29:40 nathanst Exp $
 *
 * ClusterAbstraction.cpp
 * hog 
 *
 * Created by Renee Jansen on 06/06/06
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
#include "ClusterAbstraction.h"
#include "GenericAStar.h"
#include <cfloat>
#include <cmath>

class ClusterSearchEnvironment : public SearchEnvironment
{
public:
	ClusterSearchEnvironment(GraphAbstraction *_aMap, int _level)
	:aMap(_aMap), level(_level) {}
	
	void getNeighbors(uint32_t nodeID, std::vector<uint32_t> &neighbors)
	{
		node *n = aMap->GetAbstractGraph(level)->getNode(nodeID);
		neighbor_iterator ni = n->getNeighborIter();
		for (long tmp = n->nodeNeighborNext(ni); tmp != -1; tmp = n->nodeNeighborNext(ni))
		{
			if (inCorridor(tmp))
				neighbors.push_back(tmp);
		}
	}
	
	double heuristic(uint32_t node1, uint32_t node2)
	{
		return aMap->h(aMap->GetAbstractGraph(level)->getNode(node1),
									 aMap->GetAbstractGraph(level)->getNode(node2));
	}

	double gcost(uint32_t node1, uint32_t node2)
  {
		return heuristic(node1, node2);
	}

	void setCorridor(std::vector<node *> &corr)
	{
		corridor = corr;
		if (corr.size() > 0)
		{
			corridorLevel = aMap->GetAbstractionLevel(corr[0]);
			for (unsigned int x = 0; x < corridor.size(); x++)
				corridor[x]->setLabelL(kTemporaryLabel, x);
		}
	}
	
	bool inCorridor(uint32_t nodeID)
	{
		if (corridor.size() == 0)
			return true;
		node *n = aMap->GetAbstractGraph(level)->getNode(nodeID);
		node *parent = aMap->GetNthParent(n, corridorLevel);
		if (parent)
		{
			unsigned int loc = parent->getLabelL(kTemporaryLabel);
			if ((loc < corridor.size()) && (corridor[loc] == parent))
				return true;
		}
		return false;
	}
private:
	GraphAbstraction *aMap;
	int level;
	int corridorLevel;
	std::vector<node *> corridor;
};


const static int verbose = 0;

/**
* Add an abstract node corresponding to an entrance to the cluster
 */
void Cluster::AddNode(int n)
{
	nodes.push_back(n);
}

/**
* create a cluster abstraction for the given map. Clusters are square, 
 * with height = width = clustersize. 
 */ 
ClusterAbstraction::ClusterAbstraction(Map *map, int _clusterSize)
:MapAbstraction(map),clusterSize(_clusterSize)
{
	abstractions.push_back(GetMapGraph(map));
	createClustersAndEntrances();
	linkEntrancesAndClusters();
	createAbstractGraph();
}

ClusterAbstraction::~ClusterAbstraction()
{
	graph* g = abstractions[1];
	edge_iterator ei = g->getEdgeIter();
	edge* e = g->edgeIterNext(ei);
	while(e)
	{
		path* p = temp[e];
		temp.erase(e);
		delete p;
		
		//	ei = g->getEdgeIter();
		e = g->edgeIterNext(ei);
	}
	paths.clear();
}

/**
* create clusters on the abstract level and create corresponding 
 * entrances. 
 */ 
void ClusterAbstraction::createClustersAndEntrances()
{
	Map* map = MapAbstraction::GetMap();
	int row=0, col=0, clusterId=0;
	int horizSize,vertSize;
	
	if (verbose) std::cout<<"Creating clusters and entrances\n";
	
	for (int j = 0; j < map->getMapHeight(); j+=clusterSize)
	{
		col=0; 
		for (int i = 0; i< map->getMapWidth(); i+=clusterSize)
		{
			horizSize = min(clusterSize, map->getMapWidth()-i);
			vertSize = min(clusterSize, map->getMapHeight()-j);
			Cluster cluster(clusterId++,row,col,i,j,horizSize,vertSize);
			
			addCluster(cluster);
			if (j > 0 && j < map->getMapHeight())
			{
				
				createHorizEntrances(i,i+horizSize-1,j-1,row-1,col);
			}
			if (i > 0 && i < map->getMapWidth())
			{
				
				createVertEntrances(j,j+vertSize-1,i-1,row,col-1);
			}
			col++;
		}
		row++;
	}  
	
	rows = row;
	columns = col;
	
	if (verbose) std::cout<<"map is "<<map->getMapHeight()<<" x "<<map->getMapWidth()<<std::endl
		<<"rows of clusters: "<<rows<<"\ncolumns of clusters: "<<columns<<std::endl;
}

/*
 * return the minimum of two integers (get from some library?)
 */
int ClusterAbstraction::min(int i, int j)
{
	
	if (i<j) return i;
	else return j;
}

/*
 * create the abstract graph: create a node for each entrance and connect entrances with their 
 * "counterparts" in adjacent clusters, then set up the parent/child relationship between these
 * abstract nodes in the map graph, then create intra edges. 
 */
void ClusterAbstraction::createAbstractGraph()
{
	abstractions.push_back(new graph());
	graph *g = abstractions[1];
	
	addAbsNodes(g);
	setUpParents(g);
	computeClusterPaths(g);
	
	// 	std::cout<<"1st level of abstraction\n";
	// 	g->Print(std::cout);
	// 	std::cout<<"end of graph\n\n";
	createConnectivityGraph();
	//		abstractions[2]->Print(std::cout);
	
	if (verbose) std::cout<<"abstract graph has "<<abstractions[1]->getNumNodes()<<" nodes\n";
	if (verbose) std::cout<<"abstract graphs has "<<abstractions[1]->getNumEdges()<<" edges\n";
}

/* 
* add a cluster to the abstraction
 */
void ClusterAbstraction::addCluster(Cluster c)
{
	clusters.push_back(c);
}

/* 
* add an entrance to the abstraction
 */
void ClusterAbstraction::addEntrance(Entrance e)
{
	entrances.push_back(e);
}

/**
* based on the function from HPA* abswizard.cpp with the same name
 *
 * Create horizontal entrances. Between 2 obstacles, there is either one or two entrances,
 * depending on how many tiles there are between these obstacles. 
 */ 
void ClusterAbstraction::createHorizEntrances(int start, int end, int latitude, int row, int col)
{
	Map* map = MapAbstraction::GetMap();
	graph* g = abstractions[0];
	
	for (int i=start; i<=end; i++)
	{
		
		
		while((i<=end) && (!GetNodeFromMap(i,latitude) || !GetNodeFromMap(i,latitude+1) 
											 ||!(g->findEdge(GetNodeFromMap(i,latitude)->getNum(), GetNodeFromMap(i,latitude+1)->getNum()))))
		{
			i++;
		}
		
		if (i>end)
			return;
		
		int begin = i;
		i++;
		//int begin = i;
		//		std::cout<<GetNodeFromMap(i,latitude)<<" "<<GetNodeFromMap(i,latitude+1)<<std::endl;
		while((i<=end)&&(GetNodeFromMap(i,latitude)) && (GetNodeFromMap(i,latitude+1))
					&& (g->findEdge(GetNodeFromMap(i,latitude)->getNum(), GetNodeFromMap(i,latitude+1)->getNum()))
					&& ((i==start) || ((g->findEdge(GetNodeFromMap(i-1,latitude)->getNum(), GetNodeFromMap(i,latitude)->getNum()))
														 && (g->findEdge(GetNodeFromMap(i-1,latitude+1)->getNum(), GetNodeFromMap(i,latitude+1)->getNum())))))
		{
			i++;
		}
		
		//add entrance(s)
		if ((i - begin) >= MAX_ENTRANCE_WIDTH)
		{
			// create two new entrances, one for each end
			Entrance entrance1(latitude, begin,-1,-1,
												 map->getNodeNum(begin,latitude),
												 map->getNodeNum(begin,latitude+1),
												 row, col, 1, HORIZONTAL);
			addEntrance(entrance1);
			Entrance entrance2(latitude, (i - 1),-1,-1,
												 map->getNodeNum(i-1,latitude),
												 map->getNodeNum(i-1,latitude+1),
												 row, col, 1, HORIZONTAL);
			addEntrance(entrance2);
		}
		else 
		{
			// create one entrance in the middle 
			Entrance entrance(latitude, ((i - 1) + begin)/2,-1,-1,
												map->getNodeNum(((i - 1) + begin)/2,latitude),
												map->getNodeNum(((i - 1) + begin)/2,latitude+1),
												row, col, (i - begin), HORIZONTAL);
			addEntrance(entrance);
		}
		i--;
	}
}

/**
* based on the function from HPA* abstiling.cpp with the same name
 *
 * create vertical entrances 
 */
void ClusterAbstraction::createVertEntrances(int start, int end, int meridian, int row, int col)
{
	Map* map = MapAbstraction::GetMap();
	
	graph* g = abstractions[0];
	for (int i=start; i<=end; i++)
	{
		
		
		while((i<=end)&& (!GetNodeFromMap(meridian,i) || !GetNodeFromMap(meridian+1,i)
											|| (!(g->findEdge(GetNodeFromMap(meridian,i)->getNum(), GetNodeFromMap(meridian+1,i)->getNum())))))
		{
			i++;
		}
		
		
		
		if (i>end)
			return;
		
		int begin = i;
		
		i++;
		
		while((i<=end) && (GetNodeFromMap(meridian,i)) && (GetNodeFromMap(meridian+1,i))
					&& (g->findEdge(GetNodeFromMap(meridian,i)->getNum(), GetNodeFromMap(meridian+1,i)->getNum()))
					&& ((i==start) || ((g->findEdge(GetNodeFromMap(meridian,i-1)->getNum(), GetNodeFromMap(meridian,i)->getNum()))
														 && (g->findEdge(GetNodeFromMap(meridian+1,i-1)->getNum(), GetNodeFromMap(meridian+1,i)->getNum())))))
		{
			i++;
		}
		
		//add entrance(s)
		if ((i - begin) >= MAX_ENTRANCE_WIDTH)
		{
			// create two entrances, one for each end
			Entrance entrance1(begin, meridian,-1,-1,
												 map->getNodeNum(meridian,begin),
												 map->getNodeNum(meridian+1,begin),
												 row, col, 1, VERTICAL);
			addEntrance(entrance1);
			Entrance entrance2((i - 1), meridian,-1,-1,
												 map->getNodeNum(meridian,i-1),
												 map->getNodeNum(meridian+1,i-1),
												 row, col, 1, VERTICAL);
			addEntrance(entrance2);
		}
		else
		{
			// create one entrance
			Entrance entrance(((i - 1) + begin)/2, meridian,-1,-1,
												map->getNodeNum(meridian,((i - 1) + begin)/2),
												map->getNodeNum(meridian+1,((i - 1) + begin)/2),
												row, col, (i - begin), VERTICAL);
			addEntrance(entrance);
		}
		
		i--;
		
	}
	
	
}

/** 
* from original HPA* abstiling.cpp
*/ 
void ClusterAbstraction::linkEntrancesAndClusters()
{
	if (verbose) std::cout<<"linking entrances and clusters\n";
	
	int cluster1Id;
	int cluster2Id;
	for (unsigned int i = 0; i < entrances.size(); i++)
	{
		
		Entrance &entrance = entrances[i];
		//   std::cout<<"entrance getrow: "<<entrance.getRow()<<" entrance.getcol: "<<entrance.getCol()<<std::endl;
		switch (entrance.getOrientation())
		{
			case HORIZONTAL:
			{
				cluster1Id = getClusterId(entrance.getRow(), entrance.getCol());
				cluster2Id = getClusterId(entrance.getRow() + 1, entrance.getCol());
				
				// update entrance
				entrance.setCluster1Id(cluster1Id);
				entrance.setCluster2Id(cluster2Id);
			}
				break;
			case VERTICAL:
			{
				cluster1Id = getClusterId(entrance.getRow(), entrance.getCol());
				cluster2Id = getClusterId(entrance.getRow(), entrance.getCol() + 1);
				entrance.setCluster1Id(cluster1Id);
				entrance.setCluster2Id(cluster2Id);
			}
				break;
			default:
				assert(false);
				break;
		}
	}
} 

/**
* Add a node for cluster for each entrance
 */ 
void ClusterAbstraction::addAbsNodes(graph* g)
{
	Map* map = MapAbstraction::GetMap();
	if (verbose) std::cout<<"adding abstract nodes\n";
	int newnode1=-1;
	int newnode2=-1;
	
	recVec ans;
	double r;
	int num=-1;
	
	for (unsigned int i=0; i<entrances.size(); i++)
	{
		
		Entrance &entrance = entrances[i];
		int cluster1Id = entrance.getCluster1Id();
		int cluster2Id = entrance.getCluster2Id();
		switch (entrance.getOrientation())
		{
			case HORIZONTAL:
			{
				// create node for 1st cluster
				map->getOpenGLCoord(entrance.getCenter1Col(), entrance.getCenter1Row(), ans.x,ans.y,ans.z,r);
				
				Cluster& c1 = getCluster(entrance.getCluster1Id());
				num = nodeExists(c1,ans.x,ans.y,g);
				
				if (num==-1)
				{
					
					node* n1 = new node("");
					newnode1 = g->AddNode(n1);
					n1->setLabelL(kParent,-1);
					n1->setLabelL(kAbstractionLevel,1);
					n1->setLabelF(kXCoordinate,ans.x);
					n1->setLabelF(kYCoordinate,ans.y);
					n1->setLabelF(kZCoordinate,-10*r);
					n1->setLabelL(kNumAbstractedNodes,0);
					c1.AddNode(newnode1);
				}
				else{
					newnode1 = num;
				}
				
				map->getOpenGLCoord(entrance.getCenter1Col(), entrance.getCenter1Row()+1, ans.x,ans.y,ans.z,r);
				Cluster& c2 = getCluster(cluster2Id);	  
				
				num = nodeExists(c2,ans.x,ans.y,g);
				if (num==-1)
				{
					
					node* n2= new node("");
					newnode2 = g->AddNode(n2);
					n2->setLabelL(kParent,-1);
					n2->setLabelL(kAbstractionLevel,1);
					n2->setLabelF(kXCoordinate,ans.x);
					n2->setLabelF(kYCoordinate,ans.y);
					n2->setLabelF(kZCoordinate,-10*r);
					n2->setLabelL(kNumAbstractedNodes,0);
					
					c2.AddNode(newnode2);
				}
				else{
					newnode2=num;
				}
				//add inter-edge
				g->AddEdge(new edge(newnode1,newnode2,1));
				
			}
				break;
			case VERTICAL:
			{
				map->getOpenGLCoord(entrance.getCenter1Col(), entrance.getCenter1Row(), ans.x,ans.y,ans.z,r);
				
				Cluster& c1 = getCluster(cluster1Id);
				num = nodeExists(c1,ans.x,ans.y,g);
				if (num==-1)
				{
					
					node* n1 = new node("");
					newnode1 = g->AddNode(n1);
					n1->setLabelL(kParent,-1);
					n1->setLabelL(kAbstractionLevel,1);
					n1->setLabelF(kXCoordinate,ans.x);
					n1->setLabelF(kYCoordinate,ans.y);
					n1->setLabelF(kZCoordinate,-10*r);
					n1->setLabelL(kNumAbstractedNodes,0);
					
					c1.AddNode(newnode1);
				}
				else{
					newnode1 = num;
				}
				
				
				map->getOpenGLCoord(entrance.getCenter1Col()+1, entrance.getCenter1Row(),ans.x,ans.y,ans.z,r);
				
				Cluster& c2 = getCluster(cluster2Id);
				num = nodeExists(c2,ans.x,ans.y,g);
				if (num==-1)
				{
					
					node* n2 = new node("");
					
					newnode2 = g->AddNode(n2);
					n2->setLabelL(kParent,-1);
					n2->setLabelL(kAbstractionLevel,1);
					n2->setLabelF(kXCoordinate,ans.x);
					n2->setLabelF(kYCoordinate,ans.y);
					n2->setLabelF(kZCoordinate,-10*r);
					n2->setLabelL(kNumAbstractedNodes,0);
					
					c2.AddNode(newnode2);
				}
				else{
					newnode2=num;
				}
				//add inter-edge
				g->AddEdge(new edge(newnode1, newnode2, 1));
				
			}	     
				break;
			default:
				assert(false);
				break;
		}
	}
}

/*
 * Compute the paths inside each cluster. For each pair of entrances inside a cluster, find out if 
 * there is a path that only uses nodes inside the cluster. If there is, add an edge to the abstract
 * graph with the path distance as its weight and cache the path in the hash map.  
 * 
 * can make this more efficient?
 */
void ClusterAbstraction::computeClusterPaths(graph* g)
{//int total=0;
	Map* map = MapAbstraction::GetMap();
	if (verbose) std::cout<<"computing cluster paths\n";
	for (unsigned int i=0; i<clusters.size(); i++)
	{
		Cluster& c = clusters[i];
		
		std::vector<node*> corridor; 
		for (int l=0; l<c.getNumNodes(); l++)
		{
			corridor.push_back(g->getNode(c.getIthNodeNum(l)));
		}
		for (unsigned int j=0; j < c.parents.size(); j++)
			corridor.push_back(c.parents[j]);
		
		//int num = 0;
		for (int j=0; j<c.getNumNodes(); j++)
		{
			for (int k=j+1; k<c.getNumNodes();k++)
			{
				
				// find bottom level nodes
				node* absStart = g->getNode(c.getIthNodeNum(j));
				node* absGoal = g->getNode(c.getIthNodeNum(k));
				
				// find start/end coordinates (same in abstract and bottom level)
				double startx = absStart->getLabelF(kXCoordinate);	
				double starty = absStart->getLabelF(kYCoordinate);
				double startz = absStart->getLabelF(kZCoordinate);
				
				double goalx = absGoal->getLabelF(kXCoordinate);
				double goaly = absGoal->getLabelF(kYCoordinate);
				double goalz = absGoal->getLabelF(kZCoordinate);
				
				point3d s(startx,starty,startz);
				point3d gl(goalx,goaly,goalz);
				
				int px;
				int py;
				
				map->getPointFromCoordinate(s,px,py);
				
				node* start = GetNodeFromMap(px,py);
				
				map->getPointFromCoordinate(gl,px,py);
				
				node* goal = GetNodeFromMap(px,py);
				
				int startnum = c.getIthNodeNum(j);
				int goalnum = c.getIthNodeNum(k);
				
				//find path
//				corridorAStar astar;
//				astar.setCorridor(&corridor);
//				path* p = astar.getPath(static_cast<MapAbstraction*>(this), start, goal);

				GenericAStar astar;
				ClusterSearchEnvironment cse(this, GetAbstractionLevel(start));
				cse.setCorridor(corridor);
				std::vector<uint32_t> resultPath;
				astar.getPath(&cse, start->getNum(), goal->getNum(),
											resultPath);
				path *p = 0;
				for (unsigned int x = 0; x < resultPath.size(); x++)
					p = new path(GetAbstractGraph(start)->getNode(resultPath[x]), p);

				if (p!=0)
				{
					//get its length
					double dist = distance(p); 
					
					//create edge
					edge* newedge = new edge(startnum, goalnum, dist);
					g->AddEdge(newedge);
					
					//store path
					paths[newedge] = p;
				}
			}
		}
	}
}

/**
* given a cluster row and column (NOT map row/column), return the cluster's ID.
 */
int ClusterAbstraction::getClusterId(int row, int col) const
{
	// assert? 
	return row * columns + col;
}

/**
* given a node's coordinates, find the cluster it's in
 */ 
int ClusterAbstraction::getClusterIdFromCoord(int row, int col) const
{
	// put in assert?
	int crow = row / clusterSize;
	int ccol = col / clusterSize; 
	return getClusterId(crow, ccol);
}

/**
* get the cluster from it's ID
 */
Cluster& ClusterAbstraction::getCluster(int id)
{
	assert (0 <= id && id < (int)clusters.size());
	return clusters[id];
}

/**
* check if a node already exists in a certain cluster with a particular set of 
 * coordinates
 */

//this could probably be done more efficiently. Reference to the node in the cluster? 
//(instead of node number) What about storing more efficiently ? hash map inside 
//createabsnodes? 
int ClusterAbstraction::nodeExists(const Cluster& c,double x,double y, graph* g)
{
	int n=-1;
	for (int i=0; i< c.getNumNodes(); i++)
	{
		
		n = c.getIthNodeNum(i);
		node* no = g->getNode(n);
		if ((no->getLabelF(kXCoordinate)==x) && (no->getLabelF(kYCoordinate)==y))
		{
			
			return n;
		}
	}
	return -1;
}

/**
* Given a map node, return the ID of the cluster it's in. 
 *
 * Only works for map 
 */
int ClusterAbstraction::getClusterIdFromNode(node* n)
{
	
	Map* map = MapAbstraction::GetMap();
	if (n->getLabelL(kAbstractionLevel) == 0)
		return getClusterIdFromCoord(n->getLabelL(kFirstData+1),n->getLabelL(kFirstData));	
	else{
		point3d s(n->getLabelF(kXCoordinate),n->getLabelF(kYCoordinate),-1);
		int px;
		int py;
		
		map->getPointFromCoordinate(s,px,py);
		return getClusterIdFromCoord(py,px);
	}
}


/**
* Nodes are assigned the closest entrance node in the abstract graph as their parent.
 * Connected components that cannot reach any entrance nodes will be assigned their own
 * parent node. 
 *
 * Connected component code borrowed from MapQuadTreeAbstraction.cpp
 */
void ClusterAbstraction::setUpParents(graph* g)
{
	
	
	if (verbose)	std::cout<<"Setting up parents\n";
	Map* map = MapAbstraction::GetMap();
	
	std::vector<node*> dummies; 
	int numOrigNodes = g->getNumNodes();
	
	// create ALL DUMMY PARENTS first
	for (unsigned int i=0; i<clusters.size(); i++)
	{
		
		Cluster& c = clusters[i];
		
		node* dummyParent = new node("");
		dummyParent->setLabelL(kAbstractionLevel, 1);
		dummyParent->setLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
		dummyParent->setLabelL(kParent, -1); 
		dummyParent->setLabelF(kXCoordinate, kUnknownPosition);
		dummyParent->setLabelL(kNodeBlocked, 0);
		
		
		g->AddNode(dummyParent);
		dummies.push_back(dummyParent);
		
		for (int x=c.getHOrig(); x<c.getHOrig()+c.getWidth(); x++)
		{
			
			for (int y=c.getVOrig(); y<c.getVOrig()+c.getHeight(); y++)
			{
				
				if (map->getNodeNum(x,y) >= 0)
				{
					node* n = GetNodeFromMap(x,y);
					buildNodeIntoParent(n, dummyParent);
				}
			}
		}
	}
	
	int numNodesAfter = g->getNumNodes();
	
	for (unsigned int i=0; i<clusters.size(); i++)
	{
		
		Cluster& c = clusters[i];
		//Create the corridor
		std::vector<node*> corridor; 
		corridor.push_back(dummies[i]);
		
		for (int l=0; l<c.getNumNodes(); l++)
		{
			corridor.push_back(g->getNode(c.getIthNodeNum(l)));
		}
		
//		corridorAStar astar;
//		astar.setCorridor(&corridor);	
		
		// 		std::cout<<"corridor has: "<<std::endl;
		// 		for (unsigned int bla = 0; bla<corridor.size(); bla++)
		// 			std::cout<<corridor[bla]<<" ("<<corridor[bla]->getLabelL(kNumAbstractedNodes)<<") ";
		// 		std::cout<<std::endl;
		
		//Find parent for each node 
		for (int x=c.getHOrig(); x<c.getHOrig()+c.getWidth(); x++)
		{
			for (int y=c.getVOrig(); y<c.getVOrig()+c.getHeight(); y++)
			{
				if (map->getNodeNum(x,y) >= 0)
				{
					node* mnode = GetNodeFromMap(x,y);
					
					// reset minimum 
					double minDist = DBL_MAX;
					node* entrance = 0;
					
					//for every abstract (entrance node) in this cluster
					for (int k=0; k<c.getNumNodes(); k++)
					{
						//get the entrance
						int nodenum = c.getIthNodeNum(k);
						
						node* n = g->getNode(nodenum);						
						node* low = getLowLevelNode(n);	
						
						if (low==mnode)
						{
							entrance = n;
							break;
						}
						
						//See if there's a path within this cluster
//						astar.setCorridor(&corridor);
//						path* p = astar.getPath(static_cast<MapAbstraction*>(this),low,mnode);
						GenericAStar astar;
						ClusterSearchEnvironment cse(this, GetAbstractionLevel(low));
						cse.setCorridor(corridor);
						std::vector<uint32_t> resultPath;
						astar.getPath(&cse, low->getNum(), mnode->getNum(),
													resultPath);
						path *p = 0;
						for (unsigned int t = 0; t < resultPath.size(); t++)
							p = new path(GetAbstractGraph(low)->getNode(resultPath[t]), p);
						
						if (p!=0)
						{
							// 								std::cout<<"corridor has: "<<std::endl;
							// 								for (unsigned int bla = 0; bla<corridor.size(); bla++)
							// 									std::cout<<corridor[bla]<<" ("<<corridor[bla]->getLabelL(kNumAbstractedNodes)<<") ";
							// 								std::cout<<std::endl;
							// 							}
							
							// find distance to this point 
							point3d s(n->getLabelF(kXCoordinate),n->getLabelF(kYCoordinate),-1);
							int px;
							int py;
							map->getPointFromCoordinate(s,px,py);
							
							// calculate the distance to this entrance
							
							double dist = distance(p);
							
							if (dist<minDist)
							{
								
								minDist=dist;  
								entrance=n;
								
							}
							delete p;
					}
				}
					
					//std::cout<<entrance<<std::endl;
					if (entrance)
					{
						
						point3d s(entrance->getLabelF(kXCoordinate),entrance->getLabelF(kYCoordinate),-1);
						int px;
						int py;
						map->getPointFromCoordinate(s,px,py);
						buildNodeIntoParent(mnode, entrance);
					}
					//					else
					//std::cout<<" no parent\n";
			}//end if (not out of bounds)
		}    
	}
}
// 	for (unsigned int i=0;i<dummies.size(); i++){
// 		// make sure no node has a dummy for a parent
// 		for (int j=0; j<dummies[i]->getLabelL(kNumAbstractedNodes); j++){
// 			node* child = abstractions[0]->getNode(dummies[i]->getLabelL(kFirstData+j));
// 			if (dummies[i]->getNum()==5289)
// 				std::cout<<"going to check\n";
// 			if (child->getLabelL(kParent) == dummies[i]->getNum()){
// 				if (dummies[i]->getNum()==5289)
// 					std::cout<<"removing "<<child->getNum()<<std::endl;
// 				child->setLabelL(kParent,-1);
// 			}
// 		}

// 		g->RemoveNode(dummies[i]);
// 		delete dummies[i];
// 	}

//finish by giving not-yet-abstracted nodes a parent

node_iterator ni = abstractions[0]->getNodeIter();
for (node *next = abstractions[0]->nodeIterNext(ni); next;
		 next = abstractions[0]->nodeIterNext(ni))
{
	// if it isn't abstracted, do a bfs according to the cluster and abstract these nodes together
	if ((next->getLabelL(kParent) == -1) || ((next->getLabelL(kParent) >= numOrigNodes) && next->getLabelL(kParent) < numNodesAfter))
	{
		node *parent;
		g->AddNode(parent = new node("??"));
		parent->setLabelL(kAbstractionLevel, next->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
		parent->setLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
		parent->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
		parent->setLabelF(kXCoordinate, kUnknownPosition);
		parent->setLabelL(kNodeBlocked, 0);
		abstractionBFS(next, parent, getClusterIdFromNode(next),numOrigNodes,numNodesAfter);
	}
}
for (unsigned int i=dummies.size()-1;i>0; --i)
{
	
	
	int num = dummies[i]->getNum();
	
	
	g->RemoveNode(dummies[i]);
	node* n = g->getNode(num); 
	
	if (n)
	{
		
		//update children
		int numnodes = n->getLabelL(kNumAbstractedNodes);
		for (int j=0; j<numnodes; j++)
		{
			
			(abstractions[0]->getNode(n->getLabelL(kFirstData+j)))->setLabelL(kParent, num);
		}
	}
	
	delete dummies[i];
}
}

/**
* 'borrowed' from MapQuadTreeAbstraction.cpp
 */
void ClusterAbstraction::abstractionBFS(node *which, node *parent, int cluster, int numOrigNodes, int numNodesAfter)
{
	if ((which == 0) || (getClusterIdFromNode(which) != cluster) ||
			((which->getLabelL(kParent) < numOrigNodes) && (which->getLabelL(kParent>=0))) ||
			((which->getLabelL(kParent) >= numNodesAfter) && (which->getLabelL(kParent)>=0)))
		return;
	
	buildNodeIntoParent(which, parent);
	
	getCluster(cluster).addParent(parent);
	
	neighbor_iterator ni = which->getNeighborIter();
	for (long tmp = which->nodeNeighborNext(ni); tmp != -1; tmp = which->nodeNeighborNext(ni))
	{
		abstractionBFS(abstractions[0]->getNode(tmp), parent, cluster,numOrigNodes,numNodesAfter);
	}
}

/**
* Add a 2nd level to the graph. If two nodes in the abstract (level 1) graph have the 
 * same parent, there is a path between them. 
 */
void ClusterAbstraction::createConnectivityGraph()
{
	// 	std::cout<<" begin\n";
	// 	abstractions[1]->Print(std::cout);
	// 	std::cout<<"end of this\n";
	graph* g = new graph; 
	abstractions.push_back(g); 
	
	node_iterator ni = abstractions[1]->getNodeIter();
	for (node *next = abstractions[1]->nodeIterNext(ni); next;
			 next = abstractions[1]->nodeIterNext(ni))
	{
		//		std::cout<<"looking at "<<next->getNum()<<std::endl;
		//std::cout<<"parent: "<<next->getLabelL(kParent)<<std::endl;
		
		// if it isn't abstracted, do a bfs according to the cluster and abstract these nodes together
		if (next->getLabelL(kParent) == -1)
		{
			//		std::cout<<"here inside if\n";
			node *parent;
			g->AddNode(parent = new node("??"));
			parent->setLabelL(kAbstractionLevel, next->getLabelL(kAbstractionLevel)+1); // level in abstraction tree
			parent->setLabelL(kNumAbstractedNodes, 0); // number of abstracted nodes
			parent->setLabelL(kParent, -1); // parent of this node in abstraction hierarchy
			parent->setLabelF(kXCoordinate, kUnknownPosition);
			parent->setLabelL(kNodeBlocked, 0);
			connectedBFS(next, parent);
		}
	}
	//	g->Print(std::cout);
}

void ClusterAbstraction::connectedBFS(node *which, node *parent)
{
	if ((which == 0) || (which->getLabelL(kParent) != -1))
	{
		
		return;
	}
	buildNodeIntoParent(which, parent);
	
	neighbor_iterator ni = which->getNeighborIter();
	for (long tmp = which->nodeNeighborNext(ni); tmp != -1; tmp = which->nodeNeighborNext(ni))
	{
		connectedBFS(abstractions[1]->getNode(tmp), parent);
	}
}

/**
* Check if there is a path between two nodes
 */
bool ClusterAbstraction::Pathable(node* from, node* to)
{
	node* fParent = abstractions[1]->getNode(from->getLabelL(kParent));
	node* tParent = abstractions[1]->getNode(to->getLabelL(kParent));
	
	if (fParent->getLabelL(kParent) == tParent->getLabelL(kParent))
		return true;
	else
		return false;
}


/*
 * 'borrowed' from quadTreeAbstraction.cpp 
 */ 
void ClusterAbstraction::buildNodeIntoParent(node *n, node *parent)
{
	assert((n->getLabelL(kAbstractionLevel)+1 == parent->getLabelL(kAbstractionLevel))&& parent);
	
	n->setLabelL(kParent, parent->getNum());
	parent->setLabelL(kFirstData+parent->getLabelL(kNumAbstractedNodes), n->getNum());
	parent->setLabelL(kNumAbstractedNodes, parent->getLabelL(kNumAbstractedNodes)+1);
}

/**
* insert a start or goal node in to the abstract graph (for searching purposes). 
 * The node is inserted in the same location as it is in on the map-graph. Edges are
 * added between this node and all entrance nodes in its cluster when there is a path 
 * between the nodes. The weight of the edge is set to the distance of the map-level
 * path 
 */
node* ClusterAbstraction::insertNode(node* n, int& expanded, int& touched)
{
	
	expanded = 0;
	touched = 0;
	
	if (verbose)std::cout<<"in insert node\n";
	Map* map = MapAbstraction::GetMap();
	graph* g = abstractions[1];
	
	point3d s(n->getLabelF(kXCoordinate),n->getLabelF(kYCoordinate),-1);
	int px;
	int py;
	
	map->getPointFromCoordinate(s,px,py);
	
	// 	Cluster& c = getCluster(getClusterIdFromCoord(py,px));
	Cluster& c = getCluster(getClusterIdFromCoord(n->getLabelL(kFirstData+1), n->getLabelL(kFirstData)));
	//	std::cout<<"cluster: "<<&c<<std::endl;
	//	std::cout<<"id: "<<getClusterIdFromCoord(py,px)<<std::endl;
	// 		for (int l=0; l<c.getNumNodes(); l++){
	// 			std::cout<<g->getNode(c.getIthNodeNum(l))<<std::endl;
	// 			//	corridor.push_back(g->getNode(c.getIthNodeNum(l)));
	// 		}
	recVec ans;
	double r;
	map->getOpenGLCoord(n->getLabelL(kFirstData),n->getLabelL(kFirstData+1) , ans.x,ans.y,ans.z,r);
	
	//	std::cout<<"cluster num: "<<getClusterIdFromCoord(py,px)<<std::endl;
	//	std::cout<<"level "<<n->getLabelL(kAbstractionLevel)<<std::endl;
	//	int nnum = nodeExists(c,n->getLabelF(kXCoordinate),n->getLabelF(kYCoordinate),g);
	
	int nnum = nodeExists(c,ans.x,ans.y,g);
	
	//		std::cout<<"nnum is "<<nnum<<std::endl;
	if (nnum==-1)
	{
		node* newnode = new node("");
		int nodenum = g->AddNode(newnode);
		newnode->setLabelF(kXCoordinate, n->getLabelF(kXCoordinate));
		newnode->setLabelF(kYCoordinate, n->getLabelF(kYCoordinate));
		newnode->setLabelF(kZCoordinate, n->getLabelF(kZCoordinate)*2); //should never have to draw this so it doesn't matter
		newnode->setLabelL(kParent,-1); 
		newnode->setLabelL(kNumAbstractedNodes,0);
		newnode->setLabelL(kAbstractionLevel,1);

		//  		std::vector<node*> corridor; 
		//  		for (int l=0; l<c.getNumNodes(); l++){
		//  			std::cout<<g->getNode(c.getIthNodeNum(l))<<std::endl;
		//  			corridor.push_back(g->getNode(c.getIthNodeNum(l)));
		//  		}
		
		//		std::cout<<"nodes: "<<c.getNumNodes()<<std::endl;
		for (int k=0; k<c.getNumNodes(); k++)
		{
			//get the entrance
			int num = c.getIthNodeNum(k);
			
			node* tempgoal = g->getNode(num);
			
			double goalx = tempgoal->getLabelF(kXCoordinate);
			double goaly = tempgoal->getLabelF(kYCoordinate);
			double goalz = tempgoal->getLabelF(kZCoordinate);
			
			point3d gl(goalx,goaly,goalz);
			
			map->getPointFromCoordinate(gl,px,py);
			node* goal = GetNodeFromMap(px,py);
			
			//  		std::cout<<"corridor has: "<<std::endl;
			//  		for (unsigned int bla = 0; bla<corridor.size(); bla++){
			// 			printMapCoord(corridor[bla]);
			// 		std::cout<<" "<<corridor[bla]<<" ("<<corridor[bla]->getLabelL(kNumAbstractedNodes)<<") ";
			// 		}
			//  		std::cout<<std::endl;
			
			std::vector<node*> corridor;
			for (unsigned int i=0; i<c.parents.size(); i++)
				corridor.push_back(c.parents[i]);
			for (int j=0; j<c.getNumNodes(); j++)
				corridor.push_back(g->getNode(c.getIthNodeNum(j)));
			
			//find path
//			corridorAStar astar;
//			astar.setCorridor(&corridor);
//			path* p = astar.getPath(static_cast<MapAbstraction*>(this),n , goal);
			GenericAStar astar;
			ClusterSearchEnvironment cse(this, GetAbstractionLevel(n));
			cse.setCorridor(corridor);
			std::vector<uint32_t> resultPath;
			astar.getPath(&cse, n->getNum(), goal->getNum(),
										resultPath);
			path *p = 0;
			for (unsigned int x = 0; x < resultPath.size(); x++)
				p = new path(GetAbstractGraph(n)->getNode(resultPath[x]), p);
			
			expanded += astar.getNodesExpanded();
			touched += astar.getNodesTouched();
			
			if (p!=0)
			{
				
				//get its length
				double dist = distance(p);
				//create edge
				edge* newedge = new edge(nodenum,num,dist);
				if (newnode->getLabelL(kParent)==-1)
				{
					
					newnode->setLabelL(kParent,(g->getNode(num))->getLabelL(kParent));
				}
				
				g->AddEdge(newedge);
				
				//store path
				path* p2 = p;
				temp[newedge] = p;
				
				newPaths.push_back(p2);
			}
		}
		
		if (newnode->getLabelL(kParent)==-1)
		{
			
			// This node has its own parent in the connectivity graph
			node* par = new node("");
			abstractions[2]->AddNode(par);
			par->setLabelF(kXCoordinate, kUnknownPosition);
			par->setLabelL(kParent,-1); 
			par->setLabelL(kNumAbstractedNodes,0);
			par->setLabelL(kAbstractionLevel,2);		
			
			buildNodeIntoParent(newnode,par);
		}
		
		
		//		std::cout<<"inserted\n";
		return newnode;
	}
	else{
		//		std::cout<<"no need to insert\n";
		return g->getNode(nnum);
	}
	
}



/**
* remove a start or goal node after the search has been completed. The node is
 * removed, as well as all the edges that were added to connect it to the cluster. 
 * The edges are also removed from the cache.
 */ 
void ClusterAbstraction::removeNodes(node* start, node* goal)
{
	if (verbose) std::cout<<"in remove node\n";
	
	Map* map = MapAbstraction::GetMap();
	
	// Check cluster if it's one of the original nodes
	// Only delete if the node is not part of the original abstraction
	point3d s(start->getLabelF(kXCoordinate),start->getLabelF(kYCoordinate),-1);
	int px;
	int py;
	map->getPointFromCoordinate(s,px,py);
	
	Cluster& c = getCluster(getClusterIdFromCoord(py,px));
	graph* g = abstractions[1];
	int num = nodeExists(c,start->getLabelF(kXCoordinate),start->getLabelF(kYCoordinate),g);
	if (num==-1)
	{
		
		if (start->getNumEdges() == 0)
		{
			
			abstractions[2]->RemoveNode(start->getLabelL(kParent));
		}
		edge_iterator ei = start->getEdgeIter();
		edge* e =start->edgeIterNext(ei); 
		
		while(e)
		{
			
			temp.erase(e);
			g->RemoveEdge(e);
			delete e;
			ei = start->getEdgeIter();
			e = start->edgeIterNext(ei);
		}
		int snum = start->getNum();
		assert((snum == g->getNumNodes()-1) || (snum == g->getNumNodes()-2));
		g->RemoveNode(start);
		delete start;
		//	std::cout<<"removing node\n";
	}
	//	else
	//	std::cout<<"don't need to remove this\n";
	
	point3d gl(goal->getLabelF(kXCoordinate),goal->getLabelF(kYCoordinate),-1);
	int px2;
	int py2;
	map->getPointFromCoordinate(gl,px2,py2);
	
	Cluster& c2 = getCluster(getClusterIdFromCoord(py2,px2));
	num = nodeExists(c2,goal->getLabelF(kXCoordinate),goal->getLabelF(kYCoordinate),g);
	if (num==-1)
	{
		
		if (goal->getNumEdges() == 0)
		{
			
			abstractions[2]->RemoveNode(goal->getLabelL(kParent));
			
		}
		edge_iterator ei = goal->getEdgeIter();
		//				std::cout<<"goal->getNumEdges "<<goal->getNumEdges()<<std::endl;
		edge* e = goal->edgeIterNext(ei); 
		while(e)
		{
			
			temp.erase(e);
			int gnum = goal->getNum();
			assert((gnum == g->getNumNodes()-1) || (gnum == g->getNumNodes()-2));
			g->RemoveEdge(e);
			delete e;
			ei = goal->getEdgeIter();
			e = goal->edgeIterNext(ei);
		}
		
		g->RemoveNode(goal);
		delete goal;
		//		std::cout<<"removing goal node\n";
	}
	//	else std::cout<<"don't need to remove goal\n";
	
	temp.clear();
	for (unsigned int i=0; i<newPaths.size(); i++)
	{
		
		delete newPaths[i];
	}
	newPaths.clear();
}

/**
* get the map-level path that corresponds to abstract edge e
 */ 
path* ClusterAbstraction::getCachedPath(edge* e) 
{
	if (temp[e]) return temp[e]->clone();
	else if (paths[e]) return paths[e]->clone();
	else return 0;
}

/**
* get the map-level node that is at the (x,y) coordinates of the abstract
 * node
 */
node* ClusterAbstraction::getLowLevelNode(node* abstract) 
{
	Map* map = MapAbstraction::GetMap();
	
	point3d s(abstract->getLabelF(kXCoordinate),abstract->getLabelF(kYCoordinate),-1);
	int px;
	int py;
	map->getPointFromCoordinate(s,px,py);
	
	return MapAbstraction::GetNodeFromMap(px,py);
}

void ClusterAbstraction::printMapCoord(node* n)
{
	
	Map* map = MapAbstraction::GetMap();
	graph* g = abstractions[1];
	point3d s(n->getLabelF(kXCoordinate),n->getLabelF(kYCoordinate),-1);
	int px;
	int py;
	map->getPointFromCoordinate(s,px,py);
	
	node* parent = g->getNode(n->getLabelL(kParent));
	
	point3d s2(parent->getLabelF(kXCoordinate),parent->getLabelF(kYCoordinate),-1);
	int px2;
	int py2;
	map->getPointFromCoordinate(s2,px2,py2);
	
	std::cout<<"("<<px<<","<<py<<")"; // parent "<<parent<<"\n";
																		//("<<px2<<","<<py2<<")\n";
}

void ClusterAbstraction::printPathAsCoord(path* p)
{
	if (p->n)
	{
		
		path* curr = p;
		printMapCoord(curr->n);
		while(curr->next)
		{
			
			curr = curr->next;
			printMapCoord(curr->n);
		}
	}
}

void ClusterAbstraction::OpenGLDraw(int window)
{
	MapAbstraction::OpenGLDraw(window);
	GLdouble xx, yy, zz, rr;
	glColor3f(0.25, 0.0, 0.75);
	Map *map = GetMap();
	map->getOpenGLCoord(0, 0, xx, yy, zz, rr);
	glBegin(GL_LINES);
	int numXSectors = (map->getMapWidth()+clusterSize-1)/clusterSize;
	int numYSectors = (map->getMapHeight()+clusterSize-1)/clusterSize;
	for (int y = 0; y <= numYSectors; y++)
	{
		glVertex3f(xx-rr, yy-rr+2*y*rr*clusterSize, zz-5*rr);
		glVertex3f(xx+2*numXSectors*rr*clusterSize, yy-rr+2*y*clusterSize*rr, zz-5*rr);
	}
	for (int x = 0; x <= numXSectors; x++)
	{
		glVertex3f(xx-rr+2*x*rr*clusterSize, yy-rr, zz-5*rr);
		glVertex3f(xx-rr+2*x*rr*clusterSize, yy-rr+2*numYSectors*rr*clusterSize, zz-5*rr);
	}
	glEnd();		
}
