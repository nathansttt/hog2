/*
 *  MeroB.cpp
 *  hog2
 *
 *  by Zhifu Zhang 
 *
 *  This code is not optimized for speed, but rather emphasizing on correctness.
 */

#include "MeroB.h"

bool verbose = true;

void MeroB::GetPath(GraphEnvironment *_env, graphState from, graphState to, std::vector<graphState> &thePath) 
{
	if (!InitializeSearch(_env,from,to,thePath))
		return;
	
	while(!DoSingleSearchStep(thePath)) 
		{}
	
	if (thePath.size() > 0)
		printf("\nNodes expanded=%d, Nodes touched=%d.\n",GetNodesExpanded(),GetNodesTouched());
}

bool MeroB::InitializeSearch(GraphEnvironment *_env, graphState from, graphState to, std::vector<graphState> &thePath) 
{
	env = _env;
	nodesTouched = nodesExpanded = 0;
	start = from;
	goal = to;
	
	closedList.clear();
	openQueue.reset();
	FCache.reset();
	
	if ((from == UINT32_MAX) || (to == UINT32_MAX) || (from == to))
	{
		thePath.resize(0);
		return false;
	}
	
	// step (1)
	MeroBUtil::SearchNode first(env->HCost(start, goal), 0, start, start);
	openQueue.Add(first);
	
	//if (verID == MB_B || verID == MB_BP)
	F = 0;
		
	return true;
}

bool MeroB::DoSingleSearchStep(std::vector<graphState> &thePath)

{
	if (verID == MB_BP) 
		return DoSingleStepBP(thePath);
	else if (verID == MB_B)
		return DoSingleStepB(thePath);
	else
		return DoSingleStepA(thePath);
}

/* The steps refer to the pseudo codes of the corresponding algorithms */

bool MeroB::DoSingleStepA(std::vector<graphState> &thePath)

{
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
	MeroBUtil::SearchNode topNode = openQueue.Remove();
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;
	
	if (verbose)
	{
		printf("Expanding node %d , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}
	
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
		if (!openQueue.IsIn(MeroBUtil::SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end())
		{
			MeroBUtil::SearchNode n(f,g,neighbor,topNodeID);
			openQueue.Add(n);
			
			if (verbose)
			{
				printf("Adding node %d to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}
		
		/* step (7) */
		else 
		{
			MeroBUtil::SearchNode neighborNode;
			if (openQueue.IsIn(MeroBUtil::SearchNode(neighbor)))
			{
				neighborNode = openQueue.find(MeroBUtil::SearchNode(neighbor));
				
				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g))
					continue;
				
				if (verbose)
				{
					printf("Adjusting node %d in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,g,neighbor,topNodeID); // parent is changed
				openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
			}
			else if (closedList.find(neighbor) != closedList.end())
			{
				neighborNode = closedList.find(neighbor)->second;
				
				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g))
					continue;
				
				if (verbose)
				{
					printf("Moving node %d from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,g,neighbor,topNodeID);  // parent is changed
				closedList.erase(neighbor);  // delete from CLOSED
				
				openQueue.Add(neighborNode); // add to OPEN
			}
		}
	}
	
	return false;
}

/* Algorithm B. step (3) is modified from algorithm A.*/
bool MeroB::DoSingleStepB(std::vector<graphState> &thePath)
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
	
	// put those with f < F into cache
	FCache.reset();
	while(openQueue.size() > 0)
		
	{
		MeroBUtil::SearchNode tmpNode = openQueue.top();
		if (fless(tmpNode.fCost, F))
		{
			FCache.Add(openQueue.Remove());
		}
		else
			break;
	}
	
	// select the node to expand
	MeroBUtil::SearchNode topNode;
	if (FCache.size() > 0) 
	{
		topNode = FCache.Remove();
	}
	else 
	{
		topNode = openQueue.Remove();
		F = topNode.fCost; // update F
		
		if (verbose)
		{
			printf("F updated to %lf.\n",F);
		}
	}
	
	// move remaining nodes from FCache back to openQueue
	while (FCache.size() > 0)
	{
		openQueue.Add(FCache.Remove());
	}
	
	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;
	
	if (verbose)
	{
		printf("Expanding node %d , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}
	
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
		if (!openQueue.IsIn(MeroBUtil::SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() )
			
		{
			MeroBUtil::SearchNode n(f,g,neighbor,topNodeID);
			openQueue.Add(n);
			
			if (verbose)
			{
				printf("Adding node %d to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}
		/* step (7) */
		else {
			MeroBUtil::SearchNode neighborNode;
			if (openQueue.IsIn(MeroBUtil::SearchNode(neighbor)))
			{
				neighborNode = openQueue.find(MeroBUtil::SearchNode(neighbor));
				
				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g))
					continue;
				
				if (verbose) 
				{
					printf("Adjusting node %d in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,g,neighbor,topNodeID); // parent is changed
				openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
			}
			else if (closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;
				
				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g))
					continue;
				
				if (verbose) 
				{
					printf("Moving node %d from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,g,neighbor,topNodeID);  // parent is changed
				closedList.erase(neighbor);  // delete from CLOSED
				
				openQueue.Add(neighborNode); // add to OPEN
			}
		}
	}
	return false;
}
	
/* Algorithm B' . step (3a) (3b) are inserted into algorithm B. step (7) is also changed, since we need to store the updated h */
bool MeroB::DoSingleStepBP(std::vector<graphState> &thePath) 
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
	
	// put those with f < F into cache
	FCache.reset();
	while(openQueue.size() > 0) 
	{
		MeroBUtil::SearchNode tmpNode = openQueue.top();
		if (fless(tmpNode.fCost , F)) 
		{
			FCache.Add(openQueue.Remove());
		}
		else
			break;
	}
	
	// select the node to expand
	MeroBUtil::SearchNode topNode;
	if (FCache.size() > 0) 
	{
		topNode = FCache.Remove();
	}
	else 
	{
		topNode = openQueue.Remove();
		F = topNode.fCost; // update F
		
		if (verbose) 
		{
			printf("F updated to %lf.\n",F);
		}
	}
	
	// move remaining nodes from FCache back to openQueue
	while(FCache.size() > 0) 
	{
		openQueue.Add(FCache.Remove());
	}
	
	graphState topNodeID = topNode.currNode;
	closedList[topNodeID] = topNode;
	
	if (verbose) 
	{
		printf("Expanding node %d , g=%lf, h=%lf, f=%lf.\n",topNodeID,topNode.gCost,topNode.fCost-topNode.gCost,topNode.fCost);
	}
	
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
		// this is necessary because the modified h is stored in the node, not the environment
		if (openQueue.IsIn(MeroBUtil::SearchNode(neighbor))) { 
			MeroBUtil::SearchNode nb = openQueue.find(MeroBUtil::SearchNode(neighbor));
			h = max( nb.fCost - nb.gCost, hTop - edgeWeight);
			
			h_tmp = nb.fCost - nb.gCost;
		}
		else if (closedList.find(neighbor) != closedList.end()) 
		{
			MeroBUtil::SearchNode nb = closedList.find(neighbor)->second;
			h = max( nb.fCost - nb.gCost, hTop - edgeWeight);
			
			h_tmp = nb.fCost - nb.gCost;
		}
		else 
		{
			h = max( env->HCost(neighbor,goal), hTop - edgeWeight);
			
			h_tmp = env->HCost(neighbor,goal);
		}
		
		if (verbose) 
		{
			if (fgreater(h,h_tmp))
				printf("Improving h of node %d by Mero rule (a), %lf->%lf\n",neighbor,h_tmp,h);
		}
		
		double f = g + h;
		
		/* step Mero (3b) */
		minH2 = min(minH2, h + edgeWeight);
		
		/* step (6), neither in OPEN nor CLOSED */
		if (!openQueue.IsIn(MeroBUtil::SearchNode(neighbor)) && closedList.find(neighbor) == closedList.end() ) 
		{
			MeroBUtil::SearchNode n(f,g,neighbor,topNodeID);
			openQueue.Add(n);
			
			if (verbose) 
			{
				printf("Adding node %d to OPEN, g=%lf, h=%lf, f=%lf.\n",neighbor,g,f-g,f);
			}
		}
		/* step (7) */
		else 
		{
			MeroBUtil::SearchNode neighborNode;
			if (openQueue.IsIn(MeroBUtil::SearchNode(neighbor))) 
			{
				neighborNode = openQueue.find(MeroBUtil::SearchNode(neighbor));
				
				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g)) 
				{
					if (fgreater(h , neighborNode.fCost - neighborNode.gCost)) 
					{
						// we may fail to update g, but still update h
						f = h + neighborNode.gCost;
						neighborNode.fCost = f;
						openQueue.IncreaseKey(neighborNode);
					}
					continue;
				}
				
				if (verbose) 
				{
					printf("Adjusting node %d in OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}
				
				if (fless(f , neighborNode.fCost)) 
				{
					neighborNode.copy(f,g,neighbor,topNodeID); // parent is changed
					openQueue.DecreaseKey(neighborNode);  // adjust its position in OPEN
				}
				else 
				{
					neighborNode.copy(f,g,neighbor,topNodeID); // parent is changed
					openQueue.IncreaseKey(neighborNode);  // adjust its position in OPEN
				}
			}
			else if (closedList.find(neighbor) != closedList.end()) 
			{
				neighborNode = closedList.find(neighbor)->second;
				
				//if (neighborNode.gCost <= g)
				if (!fgreater(neighborNode.gCost,g)) 
				{
					if (fgreater(h , neighborNode.fCost - neighborNode.gCost)) 
					{
						// we may fail to update g, but still update h
						f = h + neighborNode.gCost;
						neighborNode.fCost = f;
						closedList[neighbor] = neighborNode;
					}
					continue;
				}
				
				if (verbose) 
				{
					printf("Moving node %d from CLOSED to OPEN, g=%lf, h=%lf, f=%lf; g_old=%lf.\n",neighbor,g,f-g,f, neighborNode.gCost);
				}
				
				neighborNode.copy(f,g,neighbor,topNodeID);  // parent is changed
				closedList.erase(neighbor);  // delete from CLOSED
				
				openQueue.Add(neighborNode); // add to OPEN
			}
		}
	}
	/* step Mero (3b), update h of parent */
	if (fgreater(minH2 , hTop)) 
	{
		topNode.fCost = minH2 + topNode.gCost;  // f = h + g
		closedList[topNodeID] = topNode;
		
		if (verbose) 
		{
			printf("Improving h of node %d by Mero rule (b), %lf->%lf\n",topNodeID,hTop,minH2);
		}
	}
	
	return false;
}
		
void MeroB::ExtractPathToStart(graphState goalNode, std::vector<graphState> &thePath)
{
	MeroBUtil::SearchNode n;
	if (closedList.find(goalNode) != closedList.end())
	{
		n = closedList[goalNode];
	}
	else {
		n = openQueue.find(MeroBUtil::SearchNode(goalNode));
	}
	
	do {
		thePath.push_back(n.currNode);
		n = closedList[n.prevNode];
	} while (n.currNode != n.prevNode);
	//thePath.push_back(n.currNode);
}
