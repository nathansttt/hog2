/*
 *  SimpleHierarchicalAStar.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 10/9/07.
 *  Copyright 2007 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 */

#include "SimpleHierarchicalAStar.h"

SimpleHierarchicalAStar::SimpleHierarchicalAStar()
{
}

~SimpleHierarchicalAStar::SimpleHierarchicalAStar()
{
}

path *SimpleHierarchicalAStar::GetPath(GraphAbstraction *aMap, node *from, node *to, reservationProvider *rp = 0)
{
}

path *SimpleHierarchicalAStar::DoOneSearchStep()
{
}

bool SimpleHierarchicalAStar::InitializeSearch(GraphAbstraction *aMap, node *from, node *to)
{
}

void SimpleHierarchicalAStar::OpenGLDraw()
{
}



HierarchicalHeuristicEnvironment::~HierarchicalHeuristicEnvironment() {}
void HierarchicalHeuristicEnvironment::GetSuccessors(unsigned long &nodeID, std::vector<unsigned long> &neighbors)
{
}

void HierarchicalHeuristicEnvironment::GetActions(unsigned long &nodeID, std::vector<unsigned long> &actions)
{
}

unsigned long HierarchicalHeuristicEnvironment::GetAction(unsigned long &s1, unsigned long &s2)
{
}

void HierarchicalHeuristicEnvironment::ApplyAction(unsigned long &s, unsigned long a)
{
}


void HierarchicalHeuristicEnvironment::GetNextState(unsigned long &currents, unsigned long dir, unsigned long &news)
{
}

bool HierarchicalHeuristicEnvironment::InvertAction(unsigned long &a)
{
}


double HierarchicalHeuristicEnvironment::HCost(unsigned long &node1, unsigned long &node2)
{
}

double HierarchicalHeuristicEnvironment::GCost(unsigned long &node1, unsigned long &node2)
{
}

bool HierarchicalHeuristicEnvironment::GoalTest(unsigned long &node, unsigned long &goal)
{
}


uint64_t HierarchicalHeuristicEnvironment::GetStateHash(unsigned long &node)
{
}

uint64_t HierarchicalHeuristicEnvironment::GetActionHash(unsigned long act)
{
}


HierarchicalHeuristicEnvironment::OccupancyInterface<unsigned long,unsigned long> *GetOccupancyInfo()
{
}


void OpenGLDraw(int window)
{
}

void OpenGLDraw(int window, unsigned long&)
{
}

void OpenGLDraw(int window, unsigned long&, unsigned long&)
{
}

