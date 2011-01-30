/*
 *  BFS.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/15/10.
 *  Copyright 2010 University of Denver. All rights reserved.
 *
 */

#ifndef BFS_H
#define BFS_H

#include <iostream>
#include "SearchEnvironment.h"
#include <ext/hash_map>
#include "FPUtil.h"

template <class state, class action>
class BFS {
public:
	BFS() { }
	virtual ~BFS() {}
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
private:
	typedef __gnu_cxx::hash_map<uint64_t, bool, Hash64> BFSClosedList;
	
	uint64_t nodesExpanded, nodesTouched;
	
	std::deque<state> mOpen;
	std::deque<int> depth;
	BFSClosedList mClosed; // store parent id!
};	

template <class state, class action>
void BFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
								 state from, state to,
								 std::vector<state> &thePath)
{
	nodesExpanded = nodesTouched = 0;
	
	mOpen.clear();
	mClosed.clear();
	depth.clear();
	
	depth.push_back(0);
	mOpen.push_back(from);
	
	int currDepth = 0;
	unsigned long lastNodes = 0, lastIter = 0;
	while (mOpen.size() > 0)
	{
		assert(mOpen.size() == depth.size());
		state s = mOpen.front();
		mOpen.pop_front();
		if (depth.front() != currDepth)
		{
			printf("%ld tot %d inc %d b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
			lastIter = nodesExpanded-lastNodes;
			lastNodes = nodesExpanded;
		}			
		currDepth = depth.front();
		depth.pop_front();
		
		if (mClosed.find(env->GetStateHash(s)) != mClosed.end())
			continue;
		mClosed[env->GetStateHash(s)] = true;
		
		nodesExpanded++;
		env->GetSuccessors(s, thePath);
		for (unsigned int x = 0; x < thePath.size(); x++)
		{
			if (mClosed.find(env->GetStateHash(thePath[x])) == mClosed.end())
			{
				mOpen.push_back(thePath[x]);
				depth.push_back(currDepth+1);
			}
		}
	}
	printf("Final depth: %d, Nodes Expanded %lu, Exponential BF: %f\n", currDepth, nodesExpanded, pow(nodesExpanded, (double)1.0/currDepth));
}

template <class state, class action>
void BFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
								 state from, state to,
								 std::vector<action> &thePath)
{
	assert(!"not defined");
}


#endif
