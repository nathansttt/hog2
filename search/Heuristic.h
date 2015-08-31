//
//  Heuristic.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 8/19/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#ifndef hog2_glut_Heuristic_h
#define hog2_glut_Heuristic_h

enum HeuristicTreeNodeType {
	kMaxNode,
	kAddNode,
	kLeafNode
};

struct HeuristicTreeNode
{
	HeuristicTreeNodeType nodeType;
	unsigned int whichNode;
	unsigned int numChildren;
};

#warning TODO: Previously HCost was not const

template <class state>
class Heuristic {
public:
	virtual ~Heuristic() {}
	virtual double HCost(const state &a, const state &b) const;
	std::vector<HeuristicTreeNode> lookups;
	std::vector<Heuristic*> heuristics;
private:
	double HCost(const state &s1, const state &s2, int treeNode) const;
};

template <class state>
double Heuristic<state>::HCost(const state &s1, const state &s2) const
{
	return HCost(s1, s2, 0);
}

template <class state>
double Heuristic<state>::HCost(const state &s1, const state &s2, int treeNode) const
{
	double hval = 0;
	switch (lookups[treeNode].nodeType)
	{
		case kMaxNode:
		{
			for (int x = 0; x < lookups[treeNode].numChildren; x++)
			{
				hval = std::max(hval, HCost(s1, s2, lookups[treeNode].whichNode+x));
			}
		} break;
		case kAddNode:
		{
			for (int x = 0; x < lookups[treeNode].numChildren; x++)
			{
				hval += HCost(s1, s2, lookups[treeNode].whichNode+x);
			}
		} break;
		case kLeafNode:
		{
			hval = heuristics[lookups[treeNode].whichNode]->HCost(s1, s2);
		} break;
	}
	return hval;
}

#endif
