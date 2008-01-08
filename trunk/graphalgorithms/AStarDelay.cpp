/*
 *  AStarDelay.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 9/27/07.
 *  Copyright 2007 __MyCompanyName__. All rights reserved.
 *
 */

#include <sys/time.h>
#include <math.h>
#include "AStarDelay.h"

using namespace AStarDelayUtil;
using namespace GraphSearchConstants;

const static bool verbose = false;//true;

void AStarDelay::GetPath(GraphEnvironment *_env, Graph* _g, graphState from, graphState to, std::vector<graphState> &thePath) 
{
	if (!InitializeSearch(_env,_g,from,to,thePath))
		return;
	
	struct timeval t0,t1;

    gettimeofday(&t0,0);

	while(!DoSingleSearchStep(thePath)) 
	{}

	gettimeofday(&t1,0);

	double usedtime = t1.tv_sec-t0.tv_sec + (t1.tv_usec-t0.tv_usec)/1000000.0;
	
	//if(thePath.size() > 0)
		printf("\nNodes expanded=%ld, Nodes touched=%ld, Reopenings=%ld.\n",GetNodesExpanded(),GetNodesTouched(),GetNodesReopened());

	printf("Algorithm AStarDelay, time used=%lf sec, N/sec=%lf, solution cost=%lf, solution edges=%d.\n",usedtime, GetNodesExpanded()/usedtime,solutionCost,pathSize);
}

bool AStarDelay::InitializeSearch(GraphEnvironment *_env, Graph* _g, graphState from, graphState to, std::vector<graphState> &thePath) 
{
	env = _env;
	g = _g;
	nodesTouched = nodesExpanded = nodesReopened = 0;
	start = from;
	goal = to;
	
	closedList.clear();
	openQueue.reset();
	delayQueue.reset();
//	fQueue.reset();

	thePath.clear();

	//strcpy(algname,"AStarDelay");
		
	if ((from == UINT32_MAX) || (to == UINT32_MAX) || (from == to))
	{
		thePath.resize(0);
		return false;
	}
	
	// step (1)
	SearchNode first(env->HCost(start, goal), 0, start, start);
	openQueue.Add(first);
	
	F = 0;
		
	return true;
}

bool AStarDelay::DoSingleSearchStep(std::vector<graphState> &thePath)
{
	static int reopenCount = 0;

	SearchNode topNode;
	graphState temp;
	bool found = false;
	
//	printf("Reopen count: %d. Unique nodes: %ld. Log2 = %f\n",
//		   reopenCount, nodesExpanded-nodesReopened, log2(nodesExpanded-nodesReopened));
//	printf("Delay queue: %d Open queue: %d goal? %s\n",
//		   delayQueue.size(), openQueue.size(),
//		   (env->GoalTest(temp = openQueue.top().currNode, goal))?"yes":"no");
	// if we are about to open the goal off open, but the delay queue has
	// lower costs, go to the delay queue first, no matter if we're allowed to
	// or not
	if ((delayQueue.size() > 0) && (openQueue.size() > 0) &&
			(env->GoalTest(temp = openQueue.top().currNode, goal)) &&
			(fless(delayQueue.top().gCost, openQueue.top().fCost)))
	{
		do { // throw out nodes with higher cost than goal
			topNode = delayQueue.Remove();
		} while (fgreater(topNode.fCost, openQueue.top().fCost));
		//reopenCount = 0;
		nodesReopened++;
		found = true;
	}
	else if ((reopenCount < log2(nodesExpanded-nodesReopened)) && (delayQueue.size() > 0) && (openQueue.size() > 0))
	{
		if (fless(delayQueue.top().gCost, openQueue.top().fCost))
		{
			topNode = delayQueue.Remove();
			reopenCount++;
			nodesReopened++;
		}
		else {
			topNode = openQueue.Remove();
			reopenCount = 0;
		}
		found = true;
	}
	else if ((reopenCount < log2(nodesExpanded-nodesReopened)) && (delayQueue.size() > 0))
	{
		nodesReopened++;
		topNode = delayQueue.Remove();
		reopenCount++;
		found = true;
	}
//	else if (fQueue.size() > 0)
//	{
//		topNode = fQueue.Remove();
//		canReopen = true;
//		found = true;
//	}
	else if ((openQueue.size() > 0))
	{
		F = topNode.fCost;
		topNode = openQueue.Remove();
		reopenCount = 0;
		found = true;
	}

	if (found)
	{
		return DoSingleStep(topNode, thePath);
	}
	else {
		thePath.resize(0); // no path found!
		closedList.clear();
		openQueue.reset();
		env = 0;
		return true;
	}
	return false;
}

/* The steps refer to the pseudo codes of the corresponding algorithms */

bool AStarDelay::DoSingleStep(SearchNode &topNode,
															std::vector<graphState> &thePath)
{
	nodesExpanded++;
	
	if (verbose)
	{
		printf("Expanding node %ld , gcost=%lf, h=%lf, f=%lf.\n",
					 topNode.currNode, topNode.gCost, topNode.fCost-topNode.gCost, topNode.fCost);
	}
	
	// and if there are no lower f-costs on the other open lists...
	// otherwise we need to delay this node
	if (env->GoalTest(topNode.currNode, goal))
	{
		//printf("Found path to goal!\n");
		closedList[topNode.currNode] = topNode; // this is necessary for ExtractPathToStart()
		ExtractPathToStart(topNode.currNode, thePath);
		return true;
	}
	
	// step (5), computing gi is delayed
	neighbors.resize(0);
	env->GetSuccessors(topNode.currNode, neighbors);

	if(bpmxLevel > 0)
		ReversePropX1(topNode);
	
	double minCost = DBL_MAX;
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		nodesTouched++;
		
		double cost = HandleNeighbor(neighbors[x], topNode);
		if (fless(cost, minCost))
			minCost = cost;
	}

	// path max rule (i or ii?)  // this is Mero rule (b), min rule -- allen
	if (fless(topNode.fCost-topNode.gCost, minCost))
		topNode.fCost = topNode.gCost+minCost;
	closedList[topNode.currNode] = topNode;

	if(fifo.size() > 0) {
		Broadcast(1,fifo.size()); // (level, levelcount)
	}

	return false;
}

// for BPMX
void AStarDelay::ReversePropX1(SearchNode& topNode) 
{

	double maxh = topNode.fCost - topNode.gCost;
	for(unsigned int x=0;x<neighbors.size();x++) 
	{
		graphState neighbor = neighbors[x];
		SearchNode neighborNode = openQueue.find(SearchNode(neighbor));
		if(neighborNode.currNode == neighbor) {
			maxh = max(maxh, (neighborNode.fCost - neighborNode.gCost) - env->GCost(topNode.currNode,neighbor));
		}
		else {
			neighborNode = delayQueue.find(SearchNode(neighbor));
			if(neighborNode.currNode == neighbor) {
				maxh = max(maxh, (neighborNode.fCost - neighborNode.gCost) - env->GCost(topNode.currNode, neighbor));
			}
			else {
				NodeLookupTable::iterator iter = closedList.find(neighbor);
				if(iter != closedList.end()) {
					neighborNode = iter->second;
					maxh = max(maxh, (neighborNode.fCost - neighborNode.gCost) - env->GCost(topNode.currNode,neighbor));

					if(bpmxLevel > 1) {
						fifo.push_back(neighbor);
					}
				}
				else {
					maxh = max(maxh, env->HCost(neighbor,goal) - env->GCost(topNode.currNode,neighbor));
				}
			}
		}
	}

	topNode.fCost = maxh + topNode.gCost;
}

// multi level BPMX in *closed* list
// a recursive function
void AStarDelay::Broadcast(int level, int levelcount)
{ // we will only enque the newly updated (neighbor) nodes; or neighbor nodes being able to update others
	NodeLookupTable::iterator iter;

	for(int i=0;i<levelcount;i++) {
		graphState front = fifo.front();
		fifo.pop_front();

		iter = closedList.find(front);
		if(iter == closedList.end())
			continue;

		SearchNode frontNode = iter->second;
		double frontH = frontNode.fCost - frontNode.gCost;

		myneighbors.clear();
		env->GetSuccessors(front, myneighbors);

		// backward pass
		for(int x=0;x<myneighbors.size();x++) 
		{
			graphState neighbor = myneighbors[x];
			 iter = closedList.find(neighbor);
			if(iter != closedList.end()) {
				double edgeWeight = env->GCost(front,neighbor);
				SearchNode neighborNode = iter->second;

				double neighborH = neighborNode.fCost - neighborNode.gCost;
				
				if(fgreater(neighborH - edgeWeight, frontH)) {
					frontH = neighborH - edgeWeight;
					frontNode.fCost = frontNode.gCost + frontH;
					fifo.push_back(neighbor);
				}
			}
		}
		// store frontNode
		closedList[front].fCost = frontNode.fCost;

		// forward pass
		for(int x=0;x<myneighbors.size();x++) 
		{
			graphState neighbor = myneighbors[x];
			NodeLookupTable::iterator iter = closedList.find(neighbor);
			if(iter != closedList.end()) {
				double edgeWeight = env->GCost(front,neighbor);
				SearchNode neighborNode = iter->second;

				double neighborH = neighborNode.fCost - neighborNode.gCost;
				
				if(fgreater(frontH - edgeWeight, neighborH)) {
					closedList[neighbor].fCost = neighborNode.gCost + frontH - edgeWeight;  // store neighborNode
					fifo.push_back(neighbor);
				}
			}
		}
	}

	level++;
	if(level < bpmxLevel)
		Broadcast(level, fifo.size());
}

double AStarDelay::HandleNeighbor(graphState neighbor, SearchNode &topNode)
{
	
	if (openQueue.IsIn(SearchNode(neighbor))) // in OPEN
	{
		return UpdateOpenNode(neighbor, topNode);
	} 
	else if (closedList.find(neighbor) != closedList.end()) // in CLOSED
	{
		return UpdateClosedNode(neighbor, topNode);
	} 
	else if (delayQueue.IsIn(SearchNode(neighbor))) // in DELAY
	{
		return UpdateDelayedNode(neighbor, topNode);
	} 
//	else if (fQueue.IsIn(SearchNode(neighbor))) // in low g-cost!
//	{
//		return UpdateLowGNode(neighbor, topNode);
//	}
	else { // not opened yet
		return AddNewNode(neighbor, topNode);
	}
	return 0;
}

// return edge cost + h cost
double AStarDelay::AddNewNode(graphState neighbor, SearchNode &topNode)
{
	graphState topNodeID = topNode.currNode;
	double edgeCost = env->GCost(topNodeID,neighbor);
	double gcost = topNode.gCost + edgeCost;
	double h = env->HCost(neighbor,goal);
	double fcost = gcost + h;
	
	SearchNode n(fcost, gcost, neighbor, topNodeID);
//	if (fless(fcost, F))
//	{
//		fQueue.Add(n); // nodes with cost < F
//	}
//	else {
		openQueue.Add(n);
//	}
	return edgeCost+n.fCost-n.gCost;
}

// return edge cost + h cost
double AStarDelay::UpdateOpenNode(graphState neighbor, SearchNode &topNode)
{
	// lookup node
	SearchNode n;
	n = openQueue.find(SearchNode(neighbor));
	double edgeCost = env->GCost(topNode.currNode, neighbor);

	if (fless(topNode.gCost+edgeCost, n.gCost))
	{
		n.fCost -= n.gCost;
		n.gCost = topNode.gCost+edgeCost;
		n.fCost += n.gCost;
		n.prevNode = topNode.currNode;
		openQueue.DecreaseKey(n);
	}
	
	// return value for pathmax
	return edgeCost+n.fCost-n.gCost;
}

// return edge cost + h cost
double AStarDelay::UpdateClosedNode(graphState neighbor, SearchNode &topNode)
{
	// lookup node
	SearchNode n = closedList[neighbor];
	double edgeCost = env->GCost(topNode.currNode, neighbor);
	// check if we should update cost
	if (fless(topNode.gCost+edgeCost, n.gCost))
	{
		double hCost = n.fCost - n.gCost;
		n.gCost = topNode.gCost+edgeCost;

		// do pathmax here -- update child-h to parent-h - edge cost
		hCost = max(topNode.fCost-topNode.gCost-edgeCost, hCost);

		n.fCost = n.gCost + hCost;
		n.prevNode = topNode.currNode;

		// put into delay list if we can open it
		closedList.erase(neighbor);  // delete from CLOSED
		delayQueue.Add(n); // add to delay list
	}
	else if (fgreater(topNode.fCost-topNode.gCost-edgeCost, n.fCost-n.gCost)) // pathmax
	{
		n.fCost = topNode.fCost-topNode.gCost-edgeCost;
		n.fCost += n.gCost;
		closedList[neighbor] = n;
	}
	// return value for pathmax
	return edgeCost+n.fCost-n.gCost;
}

// return edge cost + h cost
double AStarDelay::UpdateDelayedNode(graphState neighbor, SearchNode &topNode)
{
	// lookup node
	SearchNode n;
	n = delayQueue.find(SearchNode(neighbor));
	double edgeCost = env->GCost(topNode.currNode, neighbor);

	if (fless(topNode.gCost+edgeCost, n.gCost))
	{
		n.fCost -= n.gCost;
		n.gCost = topNode.gCost+edgeCost;
		n.fCost += n.gCost;
		n.prevNode = topNode.currNode;
		delayQueue.DecreaseKey(n);
	}
	
	// return value for pathmax
	return edgeCost+n.fCost-n.gCost;
}


		
void AStarDelay::ExtractPathToStart(graphState goalNode, std::vector<graphState> &thePath)
{
	SearchNode n;
	if (closedList.find(goalNode) != closedList.end())
	{
		n = closedList[goalNode];
	}
	else {
		n = openQueue.find(SearchNode(goalNode));
	}
	
	solutionCost = 0;
	do {
		solutionCost += env->GCost(n.prevNode,n.currNode);

		thePath.push_back(n.currNode);
		n = closedList[n.prevNode];
	} while (n.currNode != n.prevNode);
	//thePath.push_back(n.currNode);
	pathSize = thePath.size();
}


void AStarDelay::OpenGLDraw(int)
{
	OpenGLDraw();
}

void AStarDelay::OpenGLDraw()
{
	//float r,gcost,b;
	double x,y,z;
	SearchNode sn;
	graphState nodeID;
	SearchNode topn;
	char buf[100];

	// draw nodes
	node_iterator ni = g->getNodeIter();
	for(node* n = g->nodeIterNext(ni); n; n = g->nodeIterNext(ni))
	{
		x = n->GetLabelF(kXCoordinate);
		y = n->GetLabelF(kYCoordinate);
		z = n->GetLabelF(kZCoordinate);

		nodeID = (graphState) n->GetNum();

		// if in closed
		NodeLookupTable::iterator hiter;
		if ((hiter = closedList.find(nodeID)) != closedList.end())
		{
			sn = hiter->second;

			memset(buf,0,100);
			sprintf(buf,"C %1.0f=%1.0f+%1.0f", sn.fCost, sn.gCost, (sn.fCost - sn.gCost));
			DrawText(x,y,z+0.05,0,0,0,buf);

			glColor3f(1,0,0);  // red
		}
		// if in open
		else if (openQueue.IsIn(SearchNode(nodeID)))
		{
			sn = openQueue.find(SearchNode(nodeID));
			memset(buf,0,100);
			sprintf(buf,"O %1.0f=%1.0f+%1.0f", sn.fCost, sn.gCost, (sn.fCost - sn.gCost));
			DrawText(x,y,z+0.05,0,0,0,buf);
			
			glColor3f(0,0,1);  // blue
		}
//		else if (fQueue.IsIn(SearchNode(nodeID)))
//		{
//			sn = fQueue.find(SearchNode(nodeID));
//			memset(buf,0,100);
//			sprintf(buf,"L %1.0f=%1.0f+%1.0f", sn.fCost, sn.gCost, (sn.fCost - sn.gCost));
//			DrawText(x,y,z+0.05,0,0,0,buf);
//
//			glColor3f(1,1,0);  // yellow
//		}
		else if (delayQueue.IsIn(SearchNode(nodeID)))
		{
			sn = delayQueue.find(SearchNode(nodeID));
			memset(buf,0,100);
			sprintf(buf,"D %1.0f=%1.0f+%1.0f", sn.fCost, sn.gCost, (sn.fCost - sn.gCost));
			DrawText(x,y,z+0.05,0,0,0,buf);

			glColor3f(1,0,1); // purple
		}
		// neither in open nor closed, white
		else 
		{
			glColor3f(1,1,1);  // white
		}
		DrawSphere(x,y,z,0.025);

		// draw the text info, in black
		//DrawText(x,y,z+0.05,0,0,0,buf);
	}

// draw edges
	edge_iterator ei = g->getEdgeIter();
	for(edge* e = g->edgeIterNext(ei); e; e = g->edgeIterNext(ei))
	{
		DrawEdge(e->getFrom(), e->getTo(), e->GetWeight());
	}
}

void AStarDelay::DrawText(double x, double y, double z, float r, float gg, float b, char* str)
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

void AStarDelay::DrawEdge(unsigned int from, unsigned int to, double weight)
{
	double x1,y1,z1;
	double x2,y2,z2;
	char buf[100] = {0};

	node* nfrom = g->GetNode(from);
	node* nto = g->GetNode(to);

	x1 = nfrom->GetLabelF(kXCoordinate);
	y1 = nfrom->GetLabelF(kYCoordinate);
	z1 = nfrom->GetLabelF(kZCoordinate);
	x2 = nto->GetLabelF(kXCoordinate);
	y2 = nto->GetLabelF(kYCoordinate);
	z2 = nto->GetLabelF(kZCoordinate);

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

