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
	void DoBFS(SearchEnvironment<state, action> *env, state from);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<state> &thePath);
	
	uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
private:
	
	uint64_t nodesExpanded, nodesTouched;
	
};

// pure BFS, just marking which states have been visited
// no path is saved
template <class state, class action>
void BFS<state, action>::DoBFS(SearchEnvironment<state, action> *env, state from)
{
	typedef __gnu_cxx::hash_map<uint64_t, bool, Hash64> BFSClosedList;
	std::deque<state> mOpen;
	std::deque<int> depth;
	BFSClosedList mClosed; // store parent id!

	
	std::vector<state> successors;
	nodesExpanded = nodesTouched = 0;
	
	mOpen.clear();
	mClosed.clear();
	depth.clear();
	
	depth.push_back(0);
	mOpen.push_back(from);
	
	int currDepth = 0;
	uint64_t lastNodes = 0, lastIter = 0;
	while (mOpen.size() > 0)
	{
		assert(mOpen.size() == depth.size());
		state s = mOpen.front();
		mOpen.pop_front();
		if (depth.front() != currDepth)
		{
//			printf("%ld tot %d inc %lld b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
			lastIter = nodesExpanded-lastNodes;
			lastNodes = nodesExpanded;
		}
		currDepth = depth.front();
		depth.pop_front();
		
		if (mClosed.find(env->GetStateHash(s)) != mClosed.end())
		{
//			if (mOpen.size() == 0)
//			{
//				std::cout << "Final state:\n" << s << std::endl;
//			}
			continue;
		}
		mClosed[env->GetStateHash(s)] = true;
		
		nodesExpanded++;
		env->GetSuccessors(s, successors);
		for (unsigned int x = 0; x < successors.size(); x++)
		{
			if (mClosed.find(env->GetStateHash(successors[x])) == mClosed.end())
			{
				mOpen.push_back(successors[x]);
				depth.push_back(currDepth+1);
			}
		}
//		if (mOpen.size() == 0)
//		{
//			std::cout << "Final state:\n" << s << std::endl;
//		}
	}
//	printf("Final depth: %d, Nodes Expanded %lu, Exponential BF: %f\n", currDepth, nodesExpanded, pow(nodesExpanded, (double)1.0/currDepth));
}

// Richer BFS which saves information to allow the best path to be reconstructed.
template <class state, class action>
void BFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
								 state from, state to,
								 std::vector<state> &thePath)
{
	typedef __gnu_cxx::hash_map<uint64_t, uint64_t, Hash64> BFSClosedList;
	std::deque<state> mOpen;
	std::deque<int> depth;
	BFSClosedList mClosed; // store parent id!
	thePath.resize(0);
	bool goalFound = false;
	nodesExpanded = nodesTouched = 0;
	
	mOpen.clear();
	mClosed.clear();
	depth.clear();
	
	depth.push_back(0);
	mOpen.push_back(from);
	mClosed[env->GetStateHash(from)] = env->GetStateHash(from);
//	printf("Setting parent of %llu to be %llu\n", env->GetStateHash(from),
//		   env->GetStateHash(from));

	int currDepth = 0;
	uint64_t lastNodes = 0, lastIter = 0;
	state s;
	state goal;
	while (mOpen.size() > 0)
	{
		assert(mOpen.size() == depth.size());
		s = mOpen.front();
		mOpen.pop_front();
		if (depth.front() != currDepth)
		{
//			printf("%ld tot %d inc %lld b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
			lastIter = nodesExpanded-lastNodes;
			lastNodes = nodesExpanded;
		}			
		currDepth = depth.front();
		depth.pop_front();
		if (env->GoalTest(s, to))
		{
			goal = s;
			goalFound = true;
		}
		else { // don't expand goal nodes
			nodesExpanded++;
			env->GetSuccessors(s, thePath);
			for (unsigned int x = 0; x < thePath.size(); x++)
			{
				if (mClosed.find(env->GetStateHash(thePath[x])) == mClosed.end())
				{
					mOpen.push_back(thePath[x]);
					depth.push_back(currDepth+1);
					//				printf("Setting parent of %llu to be %llu\n", env->GetStateHash(thePath[x]),
					//					   env->GetStateHash(s));
					mClosed[env->GetStateHash(thePath[x])] = env->GetStateHash(s);
				}
			}
		}
	}
//	printf("%d tot %d inc %lld b %.2f\n", currDepth, nodesExpanded, nodesExpanded-lastNodes, (double)(nodesExpanded-lastNodes)/lastIter);
//	std::cout << "Final state:\n" << s << std::endl;

	thePath.resize(0);
	if (goalFound)
	{
		uint64_t parent, lastParent;
		s = goal;
		//	std::cout << s << std::endl;
		do {
			lastParent = env->GetStateHash(s);
			thePath.push_back(s);
			parent = mClosed[env->GetStateHash(s)];
			env->GetStateFromHash(parent, s);
			//		std::cout << s << std::endl;
		} while (parent != lastParent);
	}
//	printf("Final depth: %d, Nodes Expanded %llu, Exponential BF: %f\n", currDepth, nodesExpanded, pow(nodesExpanded, (double)1.0/currDepth));
}

//template <class state, class action>
//void BFS<state, action>::GetPath(SearchEnvironment<state, action> *env,
//								 state from, state to,
//								 std::vector<action> &thePath)
//{
//	assert(!"not defined");
//}


#endif
