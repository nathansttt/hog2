/*
 *  Propagation.cpp
 *  hog2
 *
 *  by Zhifu Zhang 
 *
 *  This code emphasizes on correctness.
 */

#include <sys/time.h>
#include <math.h>
#include "Propagation.h"

using namespace PropUtil;

//#define MAXINT 2147483648

const static bool verbose = false;

void Prop::GetPath(GraphEnvironment *_env, Graph *_g, graphState from, graphState to, std::vector<graphState> &thePath) {
	if(!InitializeSearch(_env,_g,from,to,thePath))
		return;

	struct timeval t0,t1;

    gettimeofday(&t0,0);
	while(!DoSingleSearchStep(thePath)) 
		{}
	gettimeofday(&t1,0);

	double usedtime = t1.tv_sec-t0.tv_sec + (t1.tv_usec-t0.tv_usec)/1000000.0;

	if(thePath.size() > 0)
		printf("\nNodes expanded=%ld, Nodes touched=%ld, Reopenings=%ld.\n",GetNodesExpanded(),GetNodesTouched(),NodesReopened);

	//char algname[20];
	

	printf("Algorithm %s, time used=%lf sec, solution cost=%lf, solution edges=%d.\n", algname,usedtime,solutionCost,(int)thePath.size());
}

bool Prop::InitializeSearch(GraphEnvironment *_env, Graph *_g, graphState from, graphState to, std::vector<graphState> &thePath) {
	env = _env;
	grp = _g;
	nodesTouched = nodesExpanded = 0;
	NodesReopened = 0;
	start = from;
	goal = to;
	justExpanded = from;

	closedList.clear();
	openQueue.reset();
	FCache.reset();

	thePath.clear();

	if(verID==PROP_A)
		strcpy(algname,"A*");
	else if(verID==PROP_B)
		strcpy(algname,"B");
	else if(verID==PROP_BP)
		strcpy(algname,"B'");
	else if(verID==PROP_APPROX)
		strcpy(algname,"Approx");
	else if(verID==PROP_BFS)
		strcpy(algname,"BFSRepair");
	else if(verID==PROP_DELAY)
		strcpy(algname,"Delay");
	else if(verID==PROP_DP)
		strcpy(algname,"Dual Prop");
	else {
		printf("I don't know what to do.\n");
		exit(-1);
	}

	if ((from == UINT32_MAX) || (to == UINT32_MAX) || (from == to))
	{
		thePath.resize(0);
		return false;
	}

	// step (1)
	SearchNode first(env->HCost(start, goal), 0, start, start);
	openQueue.Add(first);

	//if(verID == MB_B || verID == MB_BP)
		F = 0;

	return true;
}

bool Prop::DoSingleSearchStep(std::vector<graphState> &thePath) {
	if(verID == PROP_A)
		return DoSingleStepA(thePath);
	if(verID == PROP_B)
		return DoSingleStepB(thePath);
	else if(verID == PROP_BP) 
		return DoSingleStepBP(thePath);
	else if(verID == PROP_APPROX)
		return DoSingleStepApprox(thePath);
	else if(verID == PROP_BFS)
		return DoSingleStepBFS(thePath);
	else  if(verID == PROP_DELAY)
		return DoSingleStepDelay(thePath);
	else // PROP_DP
		return DoSingleStepDP(thePath);
}

void Prop::ComputeNewHMero3a(double &h, double &h_tmp, graphState neighbor, double hTop, double edgeWeight)
{	// this is necessary because the modified h is stored in the node, not the environment
	if(openQueue.IsIn(SearchNode(neighbor))) 
	{ 
		SearchNode nb = openQueue.find(SearchNode(neighbor));
		h = max( nb.fCost - nb.gCost, hTop - edgeWeight);

		h_tmp = nb.fCost - nb.gCost;
	}
	else if(closedList.find(neighbor) != closedList.end()) 
	{
		SearchNode nb = closedList.find(neighbor)->second;
		h = max( nb.fCost - nb.gCost, hTop - edgeWeight);

		h_tmp = nb.fCost - nb.gCost;
	}
	else 
	{
		h = max( env->HCost(neighbor,goal), hTop - edgeWeight);

		h_tmp = env->HCost(neighbor,goal);
	}
}

void Prop::RelaxOpenNode(double f, double g, graphState neighbor, SearchNode &neighborNode, graphState topNodeID)
{

	if(f < neighborNode.fCost) 
	{
		neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
		openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
	}
	else //if(f > neighborNode.fCost)
	{
		neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
		openQueue.IncreaseKey(neighborNode);  // adjust its position in OPEN
	}
	//else
	//{// if f does not change, but since other info may be changed, we must still inform the openQueue !
	//	neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
	//	openQueue.Update(neighborNode);  
	//}
}

bool Prop::DoSingleStepA(std::vector<graphState> &thePath) {
// return false means the search is not finished, true otherwise

	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded++;
	SearchNode topNode = openQueue.Remove();
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if(verbose) {
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	for(unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;
		double h = env->HCost(neighbor,goal);
		double f = g + h;

		/* step (6), neither in OPEN nor CLOSED */
		if(!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end()) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if(verbose) {
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			SearchNode neighborNode;
			if(openQueue.IsIn(SearchNode(neighbor))) 
			{
				neighborNode = openQueue.find(SearchNode(neighbor));

				//if(neighborNode.gCost <= g)
				if(!fgreater(neighborNode.gCost,g))
					continue;
				
				if(verbose) {
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
				openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
			}
			else //if(closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;

				//if(neighborNode.gCost <= g)
				if(!fgreater(neighborNode.gCost,g))
					continue;

				if(verbose) {
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED

				NodesReopened++;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	neighbors.clear();

	return false;
}

bool Prop::DoSingleStepB(std::vector<graphState> &thePath) {
// return false means the search is not finished, true otherwise

	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded++;


	SearchNode topNode;
	if(fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if(verbose)
			printf("Expanding a node below F.\n");
	}
	else 
	{
		topNode = openQueue.Remove();
		
		if(fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if(verbose) {
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	for(unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;
		double h = env->HCost(neighbor,goal);
		double f = g + h;

		/* step (6), neither in OPEN nor CLOSED */
		if(!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end()) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if(verbose) {
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			SearchNode neighborNode;
			if(openQueue.IsIn(SearchNode(neighbor))) 
			{
				neighborNode = openQueue.find(SearchNode(neighbor));

				//if(neighborNode.gCost <= g)
				if(!fgreater(neighborNode.gCost,g))
					continue;
				
				if(verbose) {
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
				openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
			}
			else //if(closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;

				//if(neighborNode.gCost <= g)
				if(!fgreater(neighborNode.gCost,g))
					continue;

				if(verbose) {
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED

				NodesReopened++;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	neighbors.clear();

	return false;
}

/* Algorithm B' . step (3a) (3b) are inserted into algorithm B. step (7) is also changed, since we need to store the updated h */
bool Prop::DoSingleStepBP(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded++;

	// select the node to expand
	SearchNode topNode;
	if(fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if(verbose)
			printf("Expanding a node below F.\n");
	}
	else 
	{
		topNode = openQueue.Remove();
		
		if(fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if(verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )

	for(unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;
		
		ComputeNewHMero3a(h, h_tmp, neighbor, hTop, edgeWeight);

		if(verbose) 
		{
			if(fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if(!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() ) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if(verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			SearchNode neighborNode;
			if(openQueue.IsIn(SearchNode(neighbor))) 
			{
				neighborNode = openQueue.find(SearchNode(neighbor));

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						openQueue.IncreaseKey(neighborNode);
					continue;
				}
				
				if(verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else //if(closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						closedList[neighbor] = neighborNode;
					continue;
				}

				if(verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED
				NodesReopened++;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	/* step Mero (3b), update h of parent */
	if(fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + g
		closedList[topNodeID] = topNode;

		if(verbose) 
		{
			printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}

	neighbors.clear();

	return false;
}

bool Prop::DoSingleStepApprox(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded++;


	// select the node to expand
	SearchNode topNode;
	if(fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if(verbose)
			printf("Expanding a node below F.\n");
	}
	else 
	{
		topNode = openQueue.Remove();
		
		if(fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if(verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )

	for(unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;
		
		ComputeNewHMero3a(h, h_tmp, neighbor, hTop, edgeWeight);

		if(verbose) 
		{
			if(fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if(!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() ) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if(verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			SearchNode neighborNode;
			if(openQueue.IsIn(SearchNode(neighbor))) 
			{
				neighborNode = openQueue.find(SearchNode(neighbor));

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						openQueue.IncreaseKey(neighborNode);
					
					continue;
				}
				
				if(verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else //if(closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						closedList[neighbor] = neighborNode;

					continue;
				}

				if(verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				// careful! gCost will be altered in next operation!
				double oldG = neighborNode.gCost;

					// this operation is optional, as appears in the variant version of the alg
					neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
					closedList[neighbor] = neighborNode; // this line was MISSING !!!

				if(fgreater(oldG - g , delta)) // if we can reduce g, and not by a small amount, then don't ignore
				{
					closedList.erase(neighbor);  // delete from CLOSED
					NodesReopened++;

					openQueue.Add(neighborNode); // add to OPEN
				}

			}

		}
	}

	/* step Mero (3b), update h of parent */
	if(fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + g
		closedList[topNodeID] = topNode;

		if(verbose) 
		{
			printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}

	neighbors.clear();

	return false;
}

bool Prop::UpdateHOnly(SearchNode &neighborNode, double h)
{
	if(fgreater(h , neighborNode.fCost - neighborNode.gCost)) 
	{
		double f = h + neighborNode.gCost;
		neighborNode.fCost = f;
		return true;
	}
	return false;
}

bool Prop::DoSingleStepBFS(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded++;


	// select the node to expand
	SearchNode topNode;
	if(fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if(verbose)
			printf("Expanding a node below F.\n");
	}
	else 
	{
		topNode = openQueue.Remove();
		
		if(fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if(verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )

	for(unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;
		
		ComputeNewHMero3a(h, h_tmp, neighbor, hTop, edgeWeight);

		if(verbose) 
		{
			if(fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if(!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() ) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if(verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			SearchNode neighborNode;
			if(openQueue.IsIn(SearchNode(neighbor))) 
			{
				neighborNode = openQueue.find(SearchNode(neighbor));

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h)) {
						openQueue.IncreaseKey(neighborNode);
					}
					
					continue;
				}
				
				if(verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
				
			}
			else //if(closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						closedList[neighbor] = neighborNode;
					
					continue;
				}

				if(verbose) 
				{
					printf("Pushing node %ld to BFSQUEUE, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList[neighbor] = neighborNode;  // This was missing! which causes the BUG !!!

				if(verbose)
					printf("node %ld now has parent %ld\n",neighbor,neighborNode.prevNode); 

				bfsQueue.clear();
				bfsQueue.push_front(neighborNode);
				ClosedListRepair();
				//closedList.erase(neighbor);  // delete from CLOSED
				//openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	/* step Mero (3b), update h of parent */
	if(fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + g
		closedList[topNodeID] = topNode;

		if(verbose) 
		{
			printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}

	neighbors.clear();

	return false;
}


void Prop::ClosedListRepair() 
{
	std::vector<graphState> surrounding;

//fprintf(stderr,"*");

	// all nodes in bfsQueue are closed nodes
	while (! bfsQueue.empty()) 
	{

//fprintf(stderr,".");

		nodesExpanded++;

		SearchNode topNode = bfsQueue.front();
		bfsQueue.pop_front();
		graphState topNodeID = topNode.currNode;

		if(verbose) 
		{
			printf("BFS: Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
		}

		surrounding.clear();
		env->GetSuccessors(topNodeID, surrounding);

		double hTop = topNode.fCost - topNode.gCost;
		double minH2 = DBL_MAX;

		for(unsigned int x=0;x<surrounding.size();x++) 
		{
			nodesTouched++;

			graphState neighbor = surrounding[x];
			double edgeCost = env->GCost(topNodeID,neighbor);
			double gc = topNode.gCost + edgeCost;
			double h;
			double h_tmp; // not used

			/* step Mero (3a) */

			// to find the max h
			ComputeNewHMero3a(h, h_tmp, neighbor, hTop, edgeCost);

			if(verbose) 
			{
				if(fgreater(h,h_tmp))
					printf("BFS:Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
			}

			double f = gc + h;

			/* step Mero (3b) */
			minH2 = min(minH2, h + edgeCost);

			// to find the min g
			SearchNode neighborNode;
			if(openQueue.IsIn(SearchNode(neighbor)))
			{
				neighborNode = openQueue.find(SearchNode(neighbor));
				if(!fgreater(neighborNode.gCost , gc))
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h)) {
						openQueue.IncreaseKey(neighborNode);
					}
					
					continue;
				}

				if(verbose) 
				{
					printf("BFS:Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,gc,f-gc,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, gc, neighbor, neighborNode, topNodeID);

				if(verbose)
					printf("BFS:node %ld now has parent %ld\n",neighbor,neighborNode.prevNode); 
				
			}
			else // must be in closed
			{
				neighborNode = closedList.find(neighbor)->second;
				if(!fgreater(neighborNode.gCost , gc)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						closedList[neighbor] = neighborNode;
					
					continue;
				}

				if(verbose) 
				{
					printf("BFS:Pushing node %ld to BFSQUEUE, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,gc,f-gc,f, neighborNode.gCost);
				}

				neighborNode.copy(f,gc,neighbor,topNodeID);
				closedList[neighbor] = neighborNode; // copy current version back to closed list
				bfsQueue.push_back(neighborNode); // closed nodes will be pushed into queue

				if(verbose)
					printf("BFS:node %ld now has parent %ld\n",neighbor,neighborNode.prevNode); 
			}
		}

		/* step Mero (3b), update h of parent */
		if(fgreater(minH2 , hTop)) 
		{
			topNode.fCost = minH2 + topNode.gCost;  // f = h + g
			closedList[topNodeID] = topNode;

			if(verbose) 
			{
				printf("BFS:Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
			}
		}

		surrounding.clear();
	}

}

// get lowest g node from OPEN, with f < F
void Prop::GetLowestG(SearchNode &gNode)
{	
	gNode = openQueue.FindSpecialMin(F);

	double fCost = gNode.fCost;
	gNode.fCost = -1; // set it to min then extract from top

	openQueue.DecreaseKey(gNode);
	openQueue.Remove();

	gNode.fCost = fCost;
}

// used by Delay alg only
bool Prop::GetLowestG(TQueue &wList, SearchNode &gNode, double fBound, long TBound)
// fBound is to filter some nodes with f>=F, it could be set to DBL_MAX to allow all
{
	std::vector<SearchNode> cache;
	FCache.reset();
	// pull out everything from waitlist
	while(wList.size() > 0)
	{
		SearchNode tNode = wList.Remove();
		if(fless(tNode.fCost , fBound) && tNode.threshold <= TBound)
			FCache.Add(tNode);
		else
			cache.push_back(tNode);
	}

	// get the lowest g node
	bool answer = false;
	if(FCache.size() > 0)
	{
		gNode = FCache.Remove();
		answer = true;
	}

	// push everything back
	while(FCache.size()>0)
	{
		wList.Add(FCache.Remove());
	}
	while(cache.size()>0)
	{
		wList.Add(cache.back());
		cache.pop_back();
	}

	return answer;
}

// Nathan's alg
bool Prop::DoSingleStepDelay(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0 && WaitList.size()==0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		WaitList.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	//nodesExpanded++;

	// select the node to expand
	SearchNode topNode;
	// New. check waitlist first
	if((WaitList.size() > 0 && (WaitList.top().threshold <= nodesExpanded) || openQueue.size()==0)) 
	{
		//topNode = WaitList.Remove();
		if(openQueue.size()>0)
			GetLowestG(WaitList, topNode, DBL_MAX, nodesExpanded);
		else
			GetLowestG(WaitList, topNode, DBL_MAX, MAXINT);	

		//printf("[%d,%d,%d,%d]",openQueue.size(),closedList.size(),WaitList.size(),FCache.size());
		//fprintf(stderr,"."); 
		// infinitely reach here
	}
	else if(fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if(verbose)
			printf("Expanding a node below F.\n");

		//fprintf(stderr,".");
		// never reach here
	}
	else 
	{
		topNode = openQueue.Remove();
		
		if(fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}

		//fprintf(stderr,".");
		// finitely reach here
	}


	graphState topNodeID = topNode.currNode;
	//closedList[topNodeID] = topNode;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		SearchNode wNode;
		if(GetLowestG(WaitList,wNode,topNode.fCost,MAXINT))
		//if(fless(wfirst.fCost , topNode.fCost))
		{
		//fprintf(stderr,"we are here\n");

			F = topNode.fCost;  // set the F 
			// move goal to open;
			openQueue.Add(topNode);

			// select the lowest-g node with f<F
				//GetLowestG(WaitList,topNode,F); 
			topNode = wNode;

			topNodeID = topNode.currNode;
		}
		else
		{ // do the normal termination, since we found the goal
			closedList[topNodeID] = topNode;  // this is necessary for ExtractPathToStart()
			ExtractPathToStart(topNodeID, thePath);
			closedList.clear();
			openQueue.reset();
			FCache.reset();
			WaitList.reset();
			env = 0;
			nodesExpanded++;  // hack
			return true;
		}
	}


	// New. do the actual delay
	if(topNode.threshold > nodesExpanded)
	{
		//closedList.erase(topNodeID);
		WaitList.Add(topNode);

//fprintf(stderr,".");
		// infinitely reach here
		return false;
	}

	closedList[topNodeID] = topNode;  // put to CLOSED

	if(verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	nodesExpanded++; // newly moved here

	// New. book keeping
	topNode.lastExpanded = nodesExpanded;
	topNode.expansions++;
	topNode.threshold = topNode.lastExpanded + (long)pow(2.0,topNode.expansions);

	/* step (5), computing gi is delayed */
	neighbors.resize(0);
	env->GetSuccessors(topNodeID, neighbors);

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )

	for(unsigned int x = 0; x<neighbors.size(); x++) 
	{
		nodesTouched++;

		/* step (5) */
		graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;
		
		ComputeNewHMero3a(h, h_tmp, neighbor, hTop, edgeWeight);

		if(verbose) 
		{
			if(fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if(!openQueue.IsIn(SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() && !WaitList.IsIn(SearchNode(neighbor)) ) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if(verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			SearchNode neighborNode;
			if(openQueue.IsIn(SearchNode(neighbor))) 
			{
				neighborNode = openQueue.find(SearchNode(neighbor));

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						openQueue.IncreaseKey(neighborNode);
					
					continue;
				}
				
				if(verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else if(closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						closedList[neighbor] = neighborNode;
						
					continue;
				}

				if(verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED
				NodesReopened++;

				openQueue.Add(neighborNode); // add to OPEN
			}
			else { // must be in wait list !!
				neighborNode = WaitList.find(SearchNode(neighbor));

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						WaitList.IncreaseKey(neighborNode); // actually f is not the key in waitlist, but we can update it in this way
					
					continue;
				}
				
				if(verbose) 
				{
					printf("Adjusting node %ld in WaitList, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID); // parent may be changed
				WaitList.DecreaseKey(neighborNode); // the threshold does not change, but g decreases, and g is the 2nd key in waitlist
				//RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}

		}
	}

	/* step Mero (3b), update h of parent */
	if(fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + g
		closedList[topNodeID] = topNode;

		if(verbose) 
		{
			printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}

	neighbors.clear();

	return false;
}

void Prop::CleanUpOpen(double solCost) {
	while(openQueue.size()) 
	{
		SearchNode node = openQueue.Remove();
		if(fgreater(node.fCost,solCost))
			break;
		if(fless(node.gCost,solCost)) {
			closedList[node.currNode] = node; // put to closed
		}
	}
}

void Prop::Categorize(std::vector<graphState>& neighbors, std::vector<SearchNode>& closedNeighbors, std::vector<SearchNode>& openNeighbors, std::vector<graphState>& newNeighbors) {
	for(unsigned int x = 0; x<neighbors.size(); x++) 
	{
		graphState neighbor = neighbors[x];
		NodeLookupTable::iterator iter;

		if(openQueue.IsIn(neighbor)) {
			openNeighbors.push_back(openQueue.find(SearchNode(neighbor)));
		}
		else if((iter=closedList.find(neighbor)) != closedList.end()) {
			closedNeighbors.push_back(iter->second);
		}
		else {
			newNeighbors.push_back(neighbor);
		}
	}
}

void Prop::ReverseProp(SearchNode& topNode,std::vector<SearchNode>& closedNeighbors, std::vector<SearchNode>& openNeighbors) {
	graphState topNodeID = topNode.currNode;

	for(std::vector<SearchNode>::iterator iter = closedNeighbors.begin(); iter!=closedNeighbors.end(); iter++) 
	{
		graphState neighbor = iter->currNode;
		double newG = env->GCost(topNodeID,neighbor) + iter->gCost;
		if(fgreater(topNode.gCost,newG)) 
			topNode.copy(topNode.fCost - topNode.gCost + newG, newG, topNodeID, neighbor);
	}
	for(std::vector<SearchNode>::iterator iter = openNeighbors.begin(); iter!=openNeighbors.end(); iter++)
	{
		graphState neighbor = iter->currNode;
		double newG = env->GCost(topNodeID,neighbor) + iter->gCost;
		if(fgreater(topNode.gCost,newG))
			topNode.copy(topNode.fCost - topNode.gCost + newG, newG, topNodeID, neighbor);
	}
}

#define CLOSEDMODE 0
#define OPENMODE   1
#define NEWMODE    2
/* Algorithm Dual Propagation. Only applicable to undirected graphs! */
bool Prop::DoSingleStepDP(std::vector<graphState> &thePath) 
{
	/* step (2) */
	if (openQueue.size() == 0)
	{
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (3) */
	nodesExpanded++;

	// select the node to expand
	SearchNode topNode;
	if(fless(openQueue.top().fCost , F)) 
	{
		GetLowestG(topNode);
		if(verbose)
			printf("Expanding a node below F.\n");
	}
	else 
	{
		topNode = openQueue.Remove();
		
		if(fgreater(topNode.fCost,F)) 
		{
			F = topNode.fCost; // update F
			if (verbose) 
			{
				printf("F updated to %lf.\n",F);
			}
		}
	}

	neighbors.resize(0);
	env->GetSuccessors(topNode.currNode, neighbors);

	// categorize the neighbors
	std::vector<SearchNode> closedNeighbors;
	std::vector<SearchNode> openNeighbors;
	std::vector<graphState> newNeighbors;

	Categorize(neighbors,closedNeighbors,openNeighbors,newNeighbors);

	// reverseProp() here
	ReverseProp(topNode,closedNeighbors,openNeighbors);
	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;

	if(verbose) 
	{
		printf("Expanding node %ld , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}

	justExpanded = topNodeID;

	/* step (4) */
	if (env->GoalTest(topNodeID, goal))
	{
		//CleanUpOpen(topNode.gCost); // put nodes in open with f==F but g<g(goal) into closed, since they are likely part of the solution

		ExtractPathToStart(topNodeID, thePath);
		closedList.clear();
		openQueue.reset();
		FCache.reset();
		env = 0;
		return true;
	}

	/* step (5), computing gi is delayed */
	//neighbors.resize(0);
	//env->GetSuccessors(topNodeID, neighbors);

	double hTop = topNode.fCost - topNode.gCost;
	double minH2 = DBL_MAX; // min ( edgeWeight(i) + h(neighbor(i)) )

	while(true) 
	{
		SearchNode neighborNode;
		graphState neighbor;
		int mode;

		if(closedNeighbors.size()) 
		{
			neighborNode = closedNeighbors.back();
			neighbor = neighborNode.currNode;
			closedNeighbors.pop_back();
			mode = CLOSEDMODE;
		}
		else if(openNeighbors.size())
		{
			neighborNode = openNeighbors.back();
			neighbor = neighborNode.currNode;
			openNeighbors.pop_back();
			mode = OPENMODE;
		}
		else if(newNeighbors.size())
		{
			neighbor = newNeighbors.back();
			newNeighbors.pop_back();
			mode = NEWMODE;
		}
		else
			break;
	//}
	//for(unsigned int x = 0; x<neighbors.size(); x++) 
	//{
		nodesTouched++;

		/* step (5) */
		//graphState neighbor = neighbors[x];
		double edgeWeight = env->GCost(topNodeID,neighbor);
		double g = topNode.gCost + edgeWeight;

		/* step Mero (3a) */
		double h_tmp; // for printing reports only
		double h;
		
		ComputeNewHMero3a(h, h_tmp, neighbor, hTop, edgeWeight);

		if(verbose) 
		{
			if(fgreater(h,h_tmp))
				printf("Improving h of node %ld by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = g + h;

		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);

		/* step (6), neither in OPEN nor CLOSED */
		if(mode == NEWMODE) 
		{
			SearchNode n(f,g,neighbor,topNodeID);
			n.isGoal = (neighbor==goal);
			openQueue.Add(n);

			if(verbose) 
			{
				printf("Adding node %ld to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}

		/* step (7) */
		else 
		{
			//SearchNode neighborNode;
			if(mode == OPENMODE) 
			{
				//neighborNode = openQueue.find(SearchNode(neighbor));

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						openQueue.IncreaseKey(neighborNode);
					continue;
				}
				
				if(verbose) 
				{
					printf("Adjusting node %ld in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				RelaxOpenNode(f, g, neighbor, neighborNode, topNodeID);
			}
			else //if(closedList.find(neighbor) != closedList.end()) 
			{
				//neighborNode = closedList.find(neighbor)->second;

				//if(neighborNode.gCost <= g) {
				if(!fgreater(neighborNode.gCost,g)) 
				{
					// we may fail to update g, but still update h
					if(UpdateHOnly(neighborNode, h))
						closedList[neighbor] = neighborNode;
					continue;
				}

				if(verbose) 
				{
					printf("Moving node %ld from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}

				neighborNode.copy(f,g,neighbor,topNodeID);  // parent may be changed
				closedList.erase(neighbor);  // delete from CLOSED
				NodesReopened++;

				openQueue.Add(neighborNode); // add to OPEN
			}

		}
	}

	/* step Mero (3b), update h of parent */
	if(fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + g
		closedList[topNodeID] = topNode;

		if(verbose) 
		{
			printf("Improving h of node %ld by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}

	neighbors.clear();

	return false;
}

void Prop::ExtractPathToStart(graphState goalNode, std::vector<graphState> &thePath)
{
	SearchNode n;
	NodeLookupTable::iterator iter;

	if (closedList.find(goalNode) != closedList.end())
	{
		n = closedList[goalNode];
	}
	else n = openQueue.find(SearchNode(goalNode));

	solutionCost = 0;
	do {
		solutionCost += env->GCost(n.prevNode,n.currNode);

		if(verbose)
			printf("%ld<-%ld,",n.currNode,n.prevNode);

		thePath.push_back(n.currNode);
		//n = closedList[n.prevNode];
		iter = closedList.find(n.prevNode);
		if(iter != closedList.end())
			n = iter->second;
		else
			n = openQueue.find(SearchNode(n.prevNode));

	} while (n.currNode != n.prevNode);
	//thePath.push_back(n.currNode);
	pathSize = thePath.size();
}

void Prop::OpenGLDraw(int)
{
	OpenGLDraw();
}

void Prop::OpenGLDraw()
{
	// node to expand: blue
	// in open: green
	// in closed: red
	// in waitlist: yellow

	//float r,gcost,b;
	double x,y,z;
	SearchNode sn;
	graphState nodeID;
	SearchNode topn;
	char buf[100];

	// draw nodes
	node_iterator ni = grp->getNodeIter();
	for(node* n = grp->nodeIterNext(ni); n; n = grp->nodeIterNext(ni))
	{
		graphGenerator::GetLoc(n,x,y,z);

		nodeID = (graphState) n->GetNum();
		// draw sphere first

		// if it's just expanded
		NodeLookupTable::iterator hiter;
		if(nodeID == goal) 
		{
			glColor3f(1.0, 0.0, 1.0); // Magenta
			DrawSphere(x,y,z,0.025);
		}
		else if(nodeID == justExpanded)
		{
			sn = closedList.find(nodeID)->second;
			glColor3f(0,0,1);  // blue
			DrawSphere(x,y,z,0.025);

			memset(buf,0,100);
			sprintf(buf,"%d [%d,%d,%d]",n->GetNum(), (int)sn.gCost, (int)(sn.fCost - sn.gCost), (int)sn.fCost);
		}
		// if in closed
		else if ((hiter = closedList.find(nodeID)) != closedList.end())
		{
			sn = hiter->second;
			glColor3f(1,0,0);  // red
			DrawSphere(x,y,z,0.025);

			memset(buf,0,100);
			sprintf(buf,"%d [%d,%d,%d]",n->GetNum(), (int)sn.gCost, (int)(sn.fCost - sn.gCost), (int)sn.fCost);
		}
		// if in open
		else if(openQueue.IsIn(SearchNode(nodeID)))
		{
			sn = openQueue.find(SearchNode(nodeID));


			
				glColor3f(0,1,0);  // green
				DrawSphere(x,y,z,0.025);
			

			memset(buf,0,100);
			sprintf(buf,"%d [%ld,%ld,%ld]",n->GetNum(), (long)sn.gCost, (long)(sn.fCost - sn.gCost), (long)sn.fCost);
		}
		else if(WaitList.IsIn(SearchNode(nodeID)))
		{
			sn = WaitList.find(SearchNode(nodeID));

			glColor3f(1.0, 1.0, 0.0);  // yellow
			DrawSphere(x,y,z,0.025);

			memset(buf,0,100);
			sprintf(buf,"%d [%ld,%ld,%ld]",n->GetNum(), (long)sn.gCost, (long)(sn.fCost - sn.gCost), (long)sn.fCost);
		}
		// neither, ignore
		else 
		{
			continue;
			
			glColor3f(1,1,1); // white
			DrawSphere(x,y,z,0.025);

			memset(buf,0,100);
			sprintf(buf,"%d [?,%ld,?]",n->GetNum(), (long)env->HCost(nodeID,goal));
		}

		// draw the text info, in black
		//DrawText(x,y,z+0.05,0,0,0,buf);
	}

	// draw edges
//	edge_iterator ei = g->getEdgeIter();
//	for(edge* e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
//	{
//		DrawEdge(e->getFrom(), e->getTo(), e->GetWeight());
//	}
}

void Prop::DrawText(double x, double y, double z, float r, float gg, float b, char* str)
{
	//glPushMatrix();
	// rotate ?

	glPushMatrix();
	glColor3f(r,gg,b);
	glTranslatef(x,y,z);
	glScalef(1.0/(20*120.0), 1.0/(20*120.0), 1);
	glRotatef(180, 0.0, 0.0, 1.0);
	glRotatef(180, 0.0, 1.0, 0.0);
	
	int i=0;
	while(str[i]) 
	{
		glutStrokeCharacter(GLUT_STROKE_ROMAN,str[i]);
		i++;
	}
	glPopMatrix();
}

void Prop::DrawEdge(unsigned int from, unsigned int to, double weight)
{
	double x1,y1,z1;
	double x2,y2,z2;
	char buf[100] = {0};

	node* nfrom = grp->GetNode(from);
	node* nto = grp->GetNode(to);

	graphGenerator::GetLoc(nfrom,x1,y1,z1);
	graphGenerator::GetLoc(nto,x2,y2,z2);

	// draw line segment
	glBegin(GL_LINES);
	glColor3f(1,1,0); // yellow
	glVertex3f(x1,y1,z1);
	glVertex3f(x2,y2,z2);
	glEnd();

	// draw weight info
	sprintf(buf,"%ld",(long)weight);
	DrawText((x1+x2)/2, (y1+y2)/2, (z1+z2)/2 + 0.05, 1, 0, 0, buf); // in red
}

