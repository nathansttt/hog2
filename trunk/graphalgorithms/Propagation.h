/*
 *  Propagation.h
 *  hog2
 *
 *  by Zhifu Zhang 
 *
 *  This code emphasizes on correctness.
 */

#ifndef PROPAGATION_H
#define PROPAGATION_H

#include <math.h>
#include <cstdlib>
#include "GraphAlgorithm.h"
#include "SearchEnvironment.h"
#include "GraphEnvironment.h"
#include <deque>
#include <ext/hash_map>
#include "FPUtil.h"
#include "OpenClosedList.h"

#ifndef UINT32_MAX
#define UINT32_MAX        4294967295U
#endif

#define PROP_A        0
#define PROP_B        1
#define PROP_BP       2
#define PROP_APPROX   3
#define PROP_BFS      4
#define PROP_DELAY    5

namespace PropUtil
{
	class SearchNode {
	public:
		/* the no parameter constructor will be called by openclosedlist, and should construct invalid objects */
		//SearchNode()
		//:fCost(0),gCost(0),currNode((graphState)-1),prevNode((graphState)-1),lastExpanded(0),expansions(0),threshold(1) {}

		SearchNode(double _fCost=0, double _gCost=0, graphState curr=UINT32_MAX, graphState prev=UINT32_MAX)
		:fCost(_fCost), gCost(_gCost), currNode(curr), prevNode(prev),lastExpanded(0),expansions(0),threshold(0),isGoal(false) {}

		SearchNode(graphState curr)
		:fCost(0), gCost(0), currNode(curr), prevNode(curr),lastExpanded(0),expansions(0),threshold(0),isGoal(false) {}


		void copy(double f, double g, graphState curr, graphState prev) {
			fCost = f;
			gCost = g;
			currNode = curr;
			prevNode = prev;
		}

		double fCost;
		double gCost;
		graphState currNode;
		graphState prevNode;

		// for delay algorithm (A'?)
		long lastExpanded;
		long expansions;
		long threshold; // = lastExpanded + 2^expansions

		bool isGoal; // is it goal?
	};

	struct SearchNodeEqual {
		bool operator()(const SearchNode &i1, const SearchNode &i2)
		{ return (i1.currNode == i2.currNode); } 
	};

	// comparing f value
	struct SearchNodeCompare { // true means i2 is preferable over i1
		// in favor of goal when f ties
		// prefering larger g, i.e. smaller h is also in favor of goal nodes
		bool operator()(const SearchNode &i1, const SearchNode &i2)
		{
			if (fequal(i1.fCost, i2.fCost))
			{
				if(i2.isGoal) // always prefer a goal node in tie
					return true;
				if(i1.isGoal)
					return false;
				return (fless(i1.gCost, i2.gCost));
			}
			return (fgreater(i1.fCost, i2.fCost));
		} 
	};

	struct GGreater {
		bool operator()(const SearchNode &i1, const SearchNode &i2) {
			if(fequal(i1.gCost,i2.gCost)) {
				if(i2.isGoal) // always prefer a goal node in tie
					return true;
				if(i1.isGoal)
					return false;

				return fgreater(i1.fCost,i2.fCost);
			}

			return fgreater(i1.gCost,i2.gCost);
		}
	};

	struct TGreater {
		bool operator()(const SearchNode &i1, const SearchNode &i2) {
			if(i1.threshold == i2.threshold) // threshold is integer
				return fgreater(i1.gCost,i2.gCost);
			return i1.threshold > i2.threshold;
		}
	};

	struct SearchNodeHash {
		size_t operator()(const SearchNode &x) const
		{ return (size_t)(x.currNode); }
	};

	struct graphGenerator {
		static void SetLoc(node *n, double x, double y, double z)
		{
			n->SetLabelF(GraphSearchConstants::kXCoordinate, x);
			n->SetLabelF(GraphSearchConstants::kYCoordinate, y);
			n->SetLabelF(GraphSearchConstants::kZCoordinate, z);
//			std::cout << *n << std::endl;
//			printf("(%f, %f, %f)\n", x, y, z);
		}

		static void GetLoc(node *n, double &x, double &y, double &z)
		{
			x = n->GetLabelF(GraphSearchConstants::kXCoordinate);
			y = n->GetLabelF(GraphSearchConstants::kYCoordinate);
			z = n->GetLabelF(GraphSearchConstants::kZCoordinate);
		}

		static Graph* genFig1(unsigned int N)
	  {
			// h(0) = h(1) = 0; 
			// h(i) = 2^(i-1) + 2i - 3, for 1<i<=N
			// c(i,j) = 2^(i-2) + i - 2^(j-1) - j, for 1<=j<i<=N
			// c(1,0) = 2^(N-1) + N - 2
			assert(N >= 3);

			Graph *g = new Graph();

			// add nodes
			
			for(unsigned int nodeID = 0; nodeID <= N; nodeID++)
			{
				node *n = new node("");
				if (nodeID == 0 || nodeID == 1)
				{
					n->SetLabelF(GraphSearchConstants::kHCost, 0);
				}
				else {
					double h = pow(2,nodeID-1) + 2*nodeID - 3;
					n->SetLabelF(GraphSearchConstants::kHCost,h);
				}
				g->AddNode(n); // the real nodeID is assigned here

				if(nodeID == 0)
				{
					SetLoc(n,-1,-1,0);
				}
				else // nodes 1 - N will be drawn along 3/4 circle
				{
					double alpha = ((double)N - nodeID) * 1.5*PI /(N - 1.0);
					double beta = alpha - 0.5*PI;
					SetLoc(n,cos(beta),sin(beta),0);
				}
			}

			// add edges
			double c = pow(2,N-1) + N - 2;
			edge *e = new edge(1,0,c);
			g->AddEdge(e);
			for (unsigned int j = 1; j < N; j++)
			{
				for (unsigned int i = j+1; i <= N; i++)
				{
					c = pow(2,i-2) + i - pow(2,j-1) - j;
					e = new edge(i,j,c);
					g->AddEdge(e);
				}
			}

			return g;
		}

		static Graph* genFig2(unsigned int N)
	  {
			// h(0) = h(2N-1) = 0
			// h(i) = 2(N-1)^2 - N - i + 2, for i = 1,...,N-1
			// h(N+j) = 2(N-1)(N-2-j) + 1,  for j = 0,...,N-2
			// c(0,i) = 1,  for i = 1,...,N-1
			// c(i,N) = 2(N-1)(i-1) + 1,  for i = 1,...,N-1
			// c(i,i+1) = 2N - 2,  for i = N,...,2N-1 (should it be 2N-2 ?)
			assert(N >= 2);

			Graph* g = new Graph();

			// add nodes
			node* n = new node("");
			n->SetLabelF(GraphSearchConstants::kHCost,0);
			SetLoc(n, 0, -0.9, 0);
			g->AddNode(n);

			for (unsigned int i=1;i<=N-1;i++)
			{
				n = new node("");
				double h = 2*(N-1)*(N-1) - N - i + 2;
				n->SetLabelF(GraphSearchConstants::kHCost,h);
				g->AddNode(n);
				SetLoc(n, -1+(double)(i-1)*2.0/((double)N-2.0), 0.0, 0);
			}

			for (unsigned int j=0; j<=N-2; j++)
			{
				n = new node("");
				double h = 0; // 2*(N-1)*(N-2-j) + 1;
				n->SetLabelF(GraphSearchConstants::kHCost,h);
				g->AddNode(n);
				SetLoc(n, -(double)j/((double)N-1.0), 0.9+((j%2)?0.1:0.0), 0);
			}

			n = new node(""); // the last node (2N-1)
			n->SetLabelF(GraphSearchConstants::kHCost,0);
			SetLoc(n, -1, 1, 0);
			g->AddNode(n);

			// add edges

			// type 1
			for (unsigned int i=1;i<=N-1;i++)
			{
				edge* e = new edge(0,i,1);
				g->AddEdge(e);
			}

			// type 2
			for (unsigned int i=1; i<=N-1; i++)
			{
				double c = 2*(N-1)*(i-1) + 1;
				edge* e = new edge(i,N,c);
				g->AddEdge(e);
			}

			// type 3
			for (unsigned int i=N; i<=2*N-2; i++)
			{
				double c = 2*N - 2;
				edge* e = new edge(i,i+1,c);
				g->AddEdge(e);
			}

			return g;
		}
	};


	typedef OpenClosedList<PropUtil::SearchNode, PropUtil::SearchNodeHash,
		PropUtil::SearchNodeEqual, PropUtil::SearchNodeCompare> PQueue;

	typedef __gnu_cxx::hash_map<graphState, PropUtil::SearchNode > NodeLookupTable;

	typedef OpenClosedList<PropUtil::SearchNode, PropUtil::SearchNodeHash,
		PropUtil::SearchNodeEqual, PropUtil::GGreater> GQueue;

	typedef OpenClosedList<PropUtil::SearchNode, PropUtil::SearchNodeHash,
		PropUtil::SearchNodeEqual, PropUtil::TGreater> TQueue;
}


class Prop  : public GraphAlgorithm {
public:
	Prop() { verID = PROP_BP; delta=0;}
	Prop(unsigned int v) { verID = v; delta=0;}
	Prop(unsigned int v, double del) {verID=v; delta=del;}
	virtual ~Prop() {}
	void GetPath(GraphEnvironment *env, Graph *_g, graphState from, graphState to, std::vector<graphState> &thePath);
	
	long GetNodesExpanded() { return nodesExpanded; }
	long GetNodesTouched() { return nodesTouched; }
	long GetNodesReopened() {return reopenings;}

	bool InitializeSearch(GraphEnvironment *env, Graph *_g, graphState from, graphState to, std::vector<graphState> &thePath);
	bool DoSingleSearchStep(std::vector<graphState> &thePath);
	bool DoSingleStepA(std::vector<graphState> &thePath);
	bool DoSingleStepB(std::vector<graphState> &thePath);
	bool DoSingleStepBP(std::vector<graphState> &thePath);
	bool DoSingleStepApprox(std::vector<graphState> &thePath);
	bool DoSingleStepBFS(std::vector<graphState> &thePath);
	void ClosedListRepair();
	bool DoSingleStepDelay(std::vector<graphState> &thePath);
	void ExtractPathToStart(graphState goalNode, std::vector<graphState> &thePath);

	void GetLowestG(PropUtil::SearchNode &gNode);
	bool GetLowestG(PropUtil::TQueue &wList, PropUtil::SearchNode &gNode, double fBound, long TBound);
	bool UpdateHOnly(PropUtil::SearchNode &node, double h);
	void ComputeNewHMero3a(double &h, double &h_tmp, graphState neighbor, double hTop, double edgeWeight);
	void RelaxOpenNode(double f, double g, graphState neighbor, PropUtil::SearchNode &neighborNode, graphState topNodeID);

	void OpenGLDraw();
	void OpenGLDraw(int window);
	void DrawText(double x, double y, double z, float r, float g, float b, char* str);
	void DrawEdge(unsigned int from, unsigned int to, double weight);

	//int GetReopenings() {return reopenings;}
	double GetSolutionCost() {return solutionCost;}
	const char* GetName() {return algname;}
	int GetSolutionEdges() {return pathSize;}

	char algname[20];

private:
	unsigned int verID;
	double F;
	long nodesExpanded, nodesTouched;
	std::vector<graphState> neighbors;
	std::deque<PropUtil::SearchNode> bfsQueue;
	graphState goal, start;
	GraphEnvironment *env;
	PropUtil::PQueue openQueue;
	PropUtil::NodeLookupTable closedList; 
	PropUtil::GQueue FCache; // storing nodes with f < F, this is temporary cache. Mero proves that it is not used in B'

	PropUtil::TQueue WaitList; // for delay alg

	double delta;  // the threshold for approx

	double solutionCost;
	int pathSize;

	long reopenings;

	Graph *grp;  // for drawing only

	graphState justExpanded; // the node that is expanded in previous round, for drawing only
};




#endif
