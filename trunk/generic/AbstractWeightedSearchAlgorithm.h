/**
* @file AbstractWeightedSearchAlgorithm.h
* @package hog2
* @brief A search algorithm for DMs with abstraction
* @author Renee Jansen
* @date 01/16/2008
*
* This file is part of HOG2.
* HOG : http://www.cs.ualberta.ca/~nathanst/hog.html
* HOG2: http://code.google.com/p/hog2/
*
* HOG2 is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* 
* HOG2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with HOG2; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef ABSTRACTWEIGHTEDSEARCHALGORITHM_H
#define ABSTRACTWEIGHTEDSEARCHALGORITHM_H

#include "GraphEnvironment.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include "GraphAbstraction.h"

/** Occupancy interface which works with graphState and graphMove
* A wrapper to use with an exisitng BaseMapOccupancyInterface. It converts
* graphState to xyLoc, and performs the computation on the BaseMapOccupancyInterface.
*/
class GraphOccupancyInterface : public OccupancyInterface<graphState,graphMove>
{
public:
	GraphOccupancyInterface(BaseMapOccupancyInterface *b, Graph *_g){boi=b; g=_g;}
	virtual ~GraphOccupancyInterface(){}
	virtual void SetStateOccupied(graphState &gs, bool b )
	{ xyLoc s = Convert(gs); boi->SetStateOccupied(s,b); }
	virtual bool GetStateOccupied(graphState &gs)
	{ xyLoc s = Convert(gs); if(boi->GetStateOccupied(s)) return boi->GetStateOccupied(s);}
	virtual bool CanMove(graphState &gs1, graphState &gs2)
	{ xyLoc s1 = Convert(gs1); xyLoc s2 = Convert(gs2);return boi->CanMove(s1,s2);}
	virtual void MoveUnitOccupancy(graphState &gs1, graphState &gs2)
	{ xyLoc s1 = Convert(gs1); xyLoc s2 = Convert(gs2); boi->MoveUnitOccupancy(s1, s2);}
	
private:
	xyLoc Convert(graphState &gs)
	{
		node* n = g->GetNode(gs);
		
		xyLoc returnme;
		returnme.x = n->GetLabelL(GraphAbstractionConstants::kFirstData);
		returnme.y = n->GetLabelL(GraphAbstractionConstants::kFirstData+1);
		return returnme;
	}

	BaseMapOccupancyInterface *boi;
	//bitVector *bitvec; /// For each map position, set if occupied
	
	//long mapWidth; /// Used to compute index into bitvector
	//long mapHeight; /// used to compute index into bitvector
	
	long CalculateIndex(uint16_t x, uint16_t y);
	
	Graph* g;
};

/** A graph environment to use with the a graph abstraction. 
*/
class AbsGraphEnvironment : public GraphEnvironment 
{
public:
	AbsGraphEnvironment(Graph *_g, GraphHeuristic *gh, AbsMapEnvironment *me, BaseMapOccupancyInterface *bmoi)
		:GraphEnvironment(_g,gh){g = _g; h = gh;wenv = 0; regenv = me; goi = new GraphOccupancyInterface(bmoi,g); exactGoal = false; SetDirected(false);}
	void SetAbsGraph(Graph *_g){abs=_g;};
	void SetWeightedEnvironment(WeightedMap2DEnvironment *w){ wenv = w; }
	~AbsGraphEnvironment() {};
	//void SetBaseMapOccupancyInterface(BaseMapOccupancyInterface *bmoi){
	GraphOccupancyInterface *GetOccupancyInfo() { return goi; }
	
	/** Set to true when we want to find the exact goal rather than any child of the parent of the goal*/ 
 	void SetFindExactGoal(bool b) { exactGoal = true; }
	
	/** If exactGoal is set, GoalTest returns true when state is the same as goal. 
	If exactGoal is not set, GoalTest returns true if state and goal have the same parent*/
	bool GoalTest(graphState &state, graphState &goal)
	{
		if(exactGoal)
			return (state == goal);
		//std::cout<<"state "<<state<<" goal "<<goal<<std::endl;
		node *n = g->GetNode(state);
		//std::cout<<"g # nodes: "<<g->GetNumNodes()<<std::endl;
		//std::cout<<"n: "<<n<<std::endl;
		//std::cout<<"here\n";
		graphState nParent = n->GetLabelL(GraphAbstractionConstants::kParent);
		//std::cout<<"here\n";
		node *gn = g->GetNode(goal);
		graphState goalParent = gn->GetLabelL(GraphAbstractionConstants::kParent);
		
 		// if parent of node & goal are the same, return true
		if(nParent == goalParent)
			return true;
		
		return false;
	}
	
	/** GCost returns a weighted GCost from the WeightedMap2DEnvironment */
	double GCost(graphState &state1, graphState &state2)
	{		
		//std::cout<<"Using GCost in new env\n";
		// Get the GCost from the weighted environment
		node* s1 = g->GetNode(state1);
		node* s2 = g->GetNode(state2);
		
		assert((s1->GetLabelL(GraphAbstractionConstants::kAbstractionLevel)==0)&&(s2->GetLabelL(GraphAbstractionConstants::kAbstractionLevel)==0));
		xyLoc from, to;
		from.x = s1->GetLabelL(GraphAbstractionConstants::kFirstData);
		from.y = s1->GetLabelL(GraphAbstractionConstants::kFirstData+1);
		
		to.x = s2->GetLabelL(GraphAbstractionConstants::kFirstData);
		to.y = s2->GetLabelL(GraphAbstractionConstants::kFirstData+1);
		//std::cout<<to<<" "<<from<<std::endl;
		//std::cout<<"wenv "<<wenv<<std::endl;
		
		return wenv->GCost(from,to);
	
		
	}
	
	
private:
	Graph *g;
	Graph *abs;	
	WeightedMap2DEnvironment *wenv;
	GraphHeuristic *h;
	AbsMapEnvironment *regenv;
	GraphOccupancyInterface *goi;
	bool exactGoal;
};

class GraphStraightLineHeuristic : public GraphHeuristic {
public:
	GraphStraightLineHeuristic(Map *map, Graph *graph, Graph *mg)
	:m(map), g(graph), mapgraph(mg) {}
	double HCost(graphState &state1, graphState &state2)
	{
	//std::cout<<"state1 "<<state1<<" state2 "<<state2<<std::endl;
	//modified for abstract nodes
	
	// Get child nodes and find distance between them. Sigh. 
	node* from = g->GetNode(state1);
	node* fChild = mapgraph->GetNode(from->GetLabelL(GraphAbstractionConstants::kFirstData));
	
	node* to = g->GetNode(state2);
	node* tChild = mapgraph->GetNode(to->GetLabelL(GraphAbstractionConstants::kFirstData));
	
	int x1 = fChild->GetLabelL(GraphAbstractionConstants::kFirstData);
	int y1 = fChild->GetLabelL(GraphAbstractionConstants::kFirstData+1);
	int x2 = tChild->GetLabelL(GraphAbstractionConstants::kFirstData);
	int y2 = tChild->GetLabelL(GraphAbstractionConstants::kFirstData+1);	
		
	//int x1 = g->GetNode(state1)->GetLabelL(GraphAbstractionConstants::kXCoordinate);
	//int y1 = g->GetNode(state1)->GetLabelL(GraphSearchConstants::kMapY);
	//int x2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapX);
	//int y2 = g->GetNode(state2)->GetLabelL(GraphSearchConstants::kMapY);
		
	//std::cout<<"x1 "<<x1<<" y1 "<<y1<<" x2 "<<x2<<" y2 "<<y2<<std::endl;
		
	//std::cout<<"returning "<<sqrt(pow(x1-x2,2) + pow(y1-y2,2))<<std::endl;
	return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
	}
private:
	Map *m;
	Graph *g;
	Graph *mapgraph;
};

class OctileHeuristic : public GraphHeuristic {
public:
	OctileHeuristic(Map *map, Graph *graph)
	:m(map), g(graph) {}
	double HCost(graphState &state1, graphState &state2)
	{
		int x1 = g->GetNode(state1)->GetLabelL(GraphAbstractionConstants::kFirstData);
		int y1 = g->GetNode(state1)->GetLabelL(GraphAbstractionConstants::kFirstData+1);
		int x2 = g->GetNode(state2)->GetLabelL(GraphAbstractionConstants::kFirstData);
		int y2 = g->GetNode(state2)->GetLabelL(GraphAbstractionConstants::kFirstData+1);

		double a = ((x1>x2)?(x1-x2):(x2-x1));
		double b = ((y1>y2)?(y1-y2):(y2-y1));
		
		//std::cout<<"returning "<<((a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a))<<std::endl;
		return (a>b)?(b*ROOT_TWO+a-b):(a*ROOT_TWO+b-a);
	}
private:
	Map *m;
	Graph *g;
};

/** A search algorithm which combines direction maps with abstraction
*
* This algorithm requires an abstraction environment with a MapQuadTreeAbstraction as well
* as a WeightedMap2DEnvironment.
*/
template <class state, class action, class environment>
class AbstractWeightedSearchAlgorithm : public GenericSearchAlgorithm<state,action,environment>
{
	public:
		
		AbstractWeightedSearchAlgorithm();
		virtual ~AbstractWeightedSearchAlgorithm();
		bool InitializeSearch(environment *env, state& from, state& to, std::vector<state> &thePath);
		virtual void GetPath(environment *env, state& from, state& to, std::vector<state> &thePath);
		virtual void GetPath(environment *env, state &from, state &to, std::vector<action> &path) {}
		virtual const char *GetName() {return "AbstractWeightedSearchAlgorihm";}
		virtual int GetNodesExpanded() {return nodesExpanded;}
		virtual int GetNodesTouched()  {return nodesTouched;}
		virtual void LogFinalStats(StatCollection *stats) {}
		
		/** Set the weighted environment
		* This must be set for the algorithm to work
		*/
		void SetWeightedEnvironment(WeightedMap2DEnvironment *w){ wenv = w; }
		
		/** Set skip abstract nodes, and partial path refinement
		* If set to 'true', the algorithm plans to the one-after-next abstract node, and cuts off
		* the path after pathPerc. 0 \< pathPerc <= 1
		*/
		void SetSkipAbsNode(double pathPerc) { assert((refinePart > 0) && (refinePart <= 1)); partialSkip = true; refinePart = pathPerc; }
		
	private:
		int nodesExpanded;
		int nodesTouched;
		WeightedMap2DEnvironment *wenv;
		
		bool partialSkip;
		double refinePart;
};

template<class state, class action, class environment>
AbstractWeightedSearchAlgorithm<state,action,environment>::AbstractWeightedSearchAlgorithm()
{
	partialSkip = false;
}

template<class state, class action, class environment>
AbstractWeightedSearchAlgorithm<state,action,environment>::~AbstractWeightedSearchAlgorithm()
{

}

template<class state, class action, class environment>
bool AbstractWeightedSearchAlgorithm<state,action,environment>::InitializeSearch(environment *env, state& from, state& to, std::vector<state> &thePath)
{
	nodesExpanded = 0;
	nodesTouched = 0;
	return true;
}

template<class state, class action, class environment>
void AbstractWeightedSearchAlgorithm<state,action,environment>::GetPath(environment *env, state& from, state& to, std::vector<state> &thePath)
{	

	// get abs path
	// if neighbour is child of next abs node, skip this one
	
	InitializeSearch(env, from, to, thePath);
	//std::cout<<"NodesExpanded before starting "<<nodesExpanded<<std::endl;
	thePath.clear();
	
	MapAbstraction* ma = env->GetMapAbstraction();
	Graph *abs = ma->GetAbstractGraph(1);
	Graph *g = ma->GetAbstractGraph(0);
	
	//Find parent of "from"
	node* fromnode = ma->GetNodeFromMap(from.x, from.y);
	graphState fromPar = fromnode->GetLabelL(GraphAbstractionConstants::kParent);
	graphState fromGs = fromnode->GetNum();
	
	//Find parent of "to"
	node* tonode = env->GetMapAbstraction()->GetNodeFromMap(to.x,to.y);
	graphState toPar = tonode->GetLabelL(GraphAbstractionConstants::kParent);
	graphState toGs = tonode->GetNum();

	// Check if we're already in the same abstract node
	if(fromPar == toPar) 
	{
		// Do straight planning on lower level, using the direction map
		std::vector<graphState> refpath;
		OctileHeuristic *heur = new OctileHeuristic(ma->GetMap(),g);

		AbsGraphEnvironment *graphenv = new AbsGraphEnvironment(g,heur,env,env->GetOccupancyInfo());
		graphenv->SetFindExactGoal(true); // We don't want just any child of the parent of the goal state
		graphenv->SetWeightedEnvironment(wenv); 
		graphenv->SetAbsGraph(abs);	
		
		TemplateAStar<graphState,graphMove,GraphEnvironment> *astar = new TemplateAStar<graphState, graphMove,GraphEnvironment>();
		astar->GetPath(graphenv,fromGs,toGs,refpath);
		
		nodesExpanded += astar->GetNodesExpanded();
		nodesTouched += astar->GetNodesTouched();
		//std::cout<<"No abs path. "<<fromGs<<" to "<<toGs<<" required "<<astar->GetNodesExpanded()<<" nodes exp.\n";
		
		// Copy the path into thePath, as xyLoc
		for(unsigned int j=0; j<refpath.size(); j++)
		{
			node* newnode = g->GetNode(refpath[j]);
			xyLoc newloc; 
			newloc.x = newnode->GetLabelL(GraphAbstractionConstants::kFirstData);
			newloc.y = newnode->GetLabelL(GraphAbstractionConstants::kFirstData+1);
			thePath.push_back(newloc);
		}
	}
	else // fromPar != toPar
	{
		//Find abstract path
		std::vector<graphState> abspath;
	
		GraphStraightLineHeuristic *heur = new GraphStraightLineHeuristic(ma->GetMap(),abs, g);

		GraphEnvironment *graphenv = new GraphEnvironment(abs,heur);
		graphenv->SetDirected(false);
		TemplateAStar<graphState,graphMove,GraphEnvironment> *astar = new TemplateAStar<graphState, graphMove,GraphEnvironment>();
		astar->DoAbstractSearch(); // Don't want to use radius & occupancy interface at abstract level
		astar->GetPath(graphenv,fromPar,toPar,abspath);
		
		nodesExpanded += astar->GetNodesExpanded();
		nodesTouched += astar->GetNodesTouched();
		//std::cout<<"Abs path "<<fromPar<<" to "<<toPar<<" required "<<astar->GetNodesExpanded()<<" nodes exp.\n";
		//for(int i=0; i<abspath.size(); i++)
		//{ 
		//		std::cout<<abspath[i]<<" ";
		//}
		//	std::cout<<std::endl;
	 
		//Refinement 
		//GraphStraightLineHeuristic *heur2 = new GraphStraightLineHeuristic(ma->GetMap(),g);
		
		OctileHeuristic *heur2 = new OctileHeuristic(ma->GetMap(),g);
		AbsGraphEnvironment *age = new AbsGraphEnvironment(g,heur2,env,env->GetOccupancyInfo()); 
		age->SetWeightedEnvironment(wenv);
		age->SetAbsGraph(abs);		
		
		graphState start = fromGs; 
	
		TemplateAStar<graphState,tDirection,AbsGraphEnvironment> *astar2 = new TemplateAStar<graphState, tDirection,AbsGraphEnvironment>();
	
		graphState lastloc;
	
		std::vector<graphState> refpath;
		assert(abspath.size() != 0);

		xyLoc nl;
		nl.x = fromnode->GetLabelL(GraphAbstractionConstants::kFirstData);
		nl.y = fromnode->GetLabelL(GraphAbstractionConstants::kFirstData+1);
				//std::cout<<newloc<<" ";
				thePath.push_back(nl);
				
		// if partialSkip is true, we find a path to the one-after-next abstract node, and cut off the path
		// after some percentage (skipPerc is a value between 0 and 1, indicating the proportion to keep)
		if(partialSkip)
 		{
 			//if absPath's length is 2, want to do straight planning to goal (no cut off)
 			if(abspath.size()==2)
 			{
				// Do straight planning on lower level, using the direction map
				std::vector<graphState> refpath;
				OctileHeuristic *heur = new OctileHeuristic(ma->GetMap(),g);

				AbsGraphEnvironment *graphenv = new AbsGraphEnvironment(g,heur,env,env->GetOccupancyInfo());
				graphenv->SetFindExactGoal(true); // We don't want just any child of the parent of the goal state
				graphenv->SetWeightedEnvironment(wenv); 
				graphenv->SetAbsGraph(abs);	
		
				TemplateAStar<graphState,graphMove,GraphEnvironment> *astar = new TemplateAStar<graphState, graphMove,GraphEnvironment>();
				astar->GetPath(graphenv,fromGs,toGs,refpath);
		
				nodesExpanded += astar->GetNodesExpanded();
				nodesTouched += astar->GetNodesTouched();
				
				// Copy the path into thePath, as xyLoc
				for(unsigned int j=0; j<refpath.size(); j++)
				{
					node* newnode = g->GetNode(refpath[j]);
					xyLoc newloc; 
					newloc.x = newnode->GetLabelL(GraphAbstractionConstants::kFirstData);
					newloc.y = newnode->GetLabelL(GraphAbstractionConstants::kFirstData+1);
					thePath.push_back(newloc);
				}
	 		} // abs path not length 2
 			else
 			{
 				// path to second next abstract node and cut off the path
				graphState end = abs->GetNode(abspath[2])->GetLabelL(GraphAbstractionConstants::kFirstData);
 				
 				refpath.clear();
 		
 				astar2->GetPath(age,start,end,refpath);
 				nodesExpanded += astar2->GetNodesExpanded();
 				nodesTouched += astar2->GetNodesTouched();
 				
 				if(refpath.size()>0)
 				{
 					for(int j=1; j<refpath.size(); j++)
 					{
 						node* newnode = g->GetNode(refpath[j]);
 	
 						xyLoc newloc; 
 						newloc.x = 	newnode->GetLabelL(GraphAbstractionConstants::kFirstData);
 						newloc.y = newnode->GetLabelL(GraphAbstractionConstants::kFirstData+1);
 						thePath.push_back(newloc);
 					}
 					start = refpath[refpath.size()-1];
 				}
 			
 	
 				// Do the last one separate - need to get to exact goal
 	
	 			OctileHeuristic *heur3 = new OctileHeuristic(ma->GetMap(),g);
 
 				AbsGraphEnvironment *absgraphenv2 = new AbsGraphEnvironment(g,heur3,env,env->GetOccupancyInfo());
 				absgraphenv2->SetFindExactGoal(true);
 				absgraphenv2->SetWeightedEnvironment(wenv);
 				absgraphenv2->SetAbsGraph(abs);	
 	
	 			refpath.clear();
 
 				TemplateAStar<graphState,graphMove,GraphEnvironment> *astar3 = new TemplateAStar<graphState, graphMove,GraphEnvironment>();
 				astar3->GetPath(absgraphenv2,start,toGs,refpath);
 				nodesExpanded += astar3->GetNodesExpanded();
 				nodesTouched += astar3->GetNodesTouched();
 			
 				for(int j=1; j<refpath.size(); j++)
 				{
 					node* newnode = g->GetNode(refpath[j]);
 					xyLoc newloc; 
 					newloc.x = newnode->GetLabelL(GraphAbstractionConstants::kFirstData);
 					newloc.y = newnode->GetLabelL(GraphAbstractionConstants::kFirstData+1);
 					thePath.push_back(newloc);
 				}
 				// Chop off the path 
 				int desiredLength = refinePart * thePath.size();
				
 				for(int i=desiredLength; i<thePath.size(); i++)
 				{
 					thePath.pop_back();
 				}
 				
			} // end else (length of abs path > 2)
		} // end if(partialSkip)
		else
		{		
			for(unsigned int i=1; i<abspath.size()-1; i++) 
			//for(unsigned int i=1; i<2; i++)
			{
				//if any of my neighbours is a child of this abs node - skip it
				std::vector<graphState> nb;
				age->GetSuccessors(start, nb);
			
				bool skip = false;
				for(int k=0; k<nb.size(); k++)
				{
					node* neigh = g->GetNode(nb[k]);
					if(neigh->GetLabelL(GraphAbstractionConstants::kParent) == abspath[i])
						skip = true;
				}
					
				if (skip)
				{
					continue;
				}
				
				//Refine the path
				//Path to first any child - will get 'fixed' in GoalTest in environment
				graphState end = abs->GetNode(abspath[i])->GetLabelL(GraphAbstractionConstants::kFirstData);
				//ma->GetAbstractGraph(0)->GetNode(abspath[i]);
		
				refpath.clear();
		
				astar2->GetPath(age,start,end,refpath);
				nodesExpanded += astar2->GetNodesExpanded();
				nodesTouched += astar2->GetNodesTouched();
				
				//std::cout<<start<<" to "<<end<<" required "<<astar2->GetNodesExpanded()<<" nodes exp.\n";
				//std::cout<<"Refining nodes expanded now "<<nodesExpanded<<std::endl;
			
				if(refpath.size()>0)
				{
					for(int j=1; j<refpath.size(); j++)
					{
						node* newnode = g->GetNode(refpath[j]);
	
						xyLoc newloc; 
						newloc.x = 	newnode->GetLabelL(GraphAbstractionConstants::kFirstData);
						newloc.y = newnode->GetLabelL(GraphAbstractionConstants::kFirstData+1);
						//std::cout<<newloc<<" ";
						thePath.push_back(newloc);
					}
					//std::cout<<std::endl;	
					//std::cout<<"Refpath size is zero!!!\n";
					start = refpath[refpath.size()-1];
				}
			}
	
			// Do the last one separate - need to get to exact goal
	
			OctileHeuristic *heur3 = new OctileHeuristic(ma->GetMap(),g);

			AbsGraphEnvironment *absgraphenv2 = new AbsGraphEnvironment(g,heur3,env,env->GetOccupancyInfo());
			absgraphenv2->SetFindExactGoal(true);
			absgraphenv2->SetWeightedEnvironment(wenv);
			absgraphenv2->SetAbsGraph(abs);	
	
			refpath.clear();

			TemplateAStar<graphState,graphMove,GraphEnvironment> *astar3 = new TemplateAStar<graphState, graphMove,GraphEnvironment>();
			astar3->GetPath(absgraphenv2,start,toGs,refpath);
			nodesExpanded += astar3->GetNodesExpanded();
			nodesTouched += astar3->GetNodesTouched();
		//std::cout<<start<<" to "<<toGs<<" required "<<astar3->GetNodesExpanded()<<" nodes exp.\n";
			//Transfer the path to thePath, as series of xyLoc
			for(int j=1; j<refpath.size(); j++)
			{
				node* newnode = g->GetNode(refpath[j]);
				xyLoc newloc; 
				newloc.x = newnode->GetLabelL(GraphAbstractionConstants::kFirstData);
				newloc.y = newnode->GetLabelL(GraphAbstractionConstants::kFirstData+1);
				thePath.push_back(newloc);
			}
		}// end else --> not partialSkip
	
	//for (int i=0; i<thePath.size(); i++)
	//{
	//	std::cout<<thePath[i]<<" ";
	//}
	//std::cout<<std::endl<<std::endl;
	
	} // end else --> start and goal not same parent
}

#endif
