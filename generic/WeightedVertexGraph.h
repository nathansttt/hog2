//
//  WeightedVertexGraph.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/1/17.
//  Copyright © 2017 University of Denver. All rights reserved.
//

#ifndef WeightedVertexGraph_h
#define WeightedVertexGraph_h

#include "TemplateAStar.h"
#include "SVGUtil.h"
#include <map>
#include <sstream>

template <class state, class action, class environment>
uint64_t GetWeightedVertexGraph(const state &start, const state &goal, environment *e, Heuristic<state> *f, Heuristic<state> *b, const char *filename = 0)
{
//	printf("Analyzing weighted vertext graph.\n");
	std::vector<state> path;
	TemplateAStar<state, action, environment> astarf;
	TemplateAStar<state, action, environment> astarb;
	astarf.SetHeuristic(f);
	astarf.GetPath(e, start, goal, path);
	astarb.SetHeuristic(b);
	astarb.GetPath(e, goal, start, path);
	double optCost = e->GetPathLength(path);
	double forwardOptG = -1;
	double backwardOptG = -1;
	int forwardSum = 0;
	int backwardSum = 0;
	std::string optCostStr = "Optimal cost: "+std::to_string(optCost);
//	std::cout << optCostStr << "\n";
	// Contains the count of how many states have each g-cost
	std::map<double, int> m_f, m_b;
	if (astarf.GetNecessaryExpansions() == 0 || astarb.GetNecessaryExpansions() == 0)
	{
		printf("No necessary expansions\n");
		return 0;
	}
	for (int x = 0; x < astarf.GetNumItems(); x++)
	{
		const auto &i = astarf.GetItem(x);
		if (i.where != kClosedList)
			continue;
		if (!fless(i.g+i.h, optCost))
			continue;
		auto i2 = m_f.find(i.g);
		if (i2 == m_f.end())
		{
			m_f.insert({i.g, 1});
		}
		else {
			i2->second++;
		}
	}
	if (m_f.size() == 0)
	{
		printf("No forward item\n");
		return 0;
	}
	for (int x = 0; x < astarb.GetNumItems(); x++)
	{
		const auto &i = astarb.GetItem(x);
		if (i.where != kClosedList)
			continue;
		if (!fless(i.g+i.h, optCost))
			continue;
		auto i2 = m_b.find(i.g);
		if (i2 == m_b.end())
		{
			m_b.insert({i.g, 1});
		}
		else {
			i2->second++;
		}
	}
	if (m_b.size() == 0)
	{
		printf("No backward item\n");
		return 0;
	}
//	std::cout << "Forward\n";
	for (auto i = m_f.begin(); i != m_f.end(); i++)
	{
//		std::cout << i->first << " : " << i->second << "\n";
		forwardSum += i->second;
	}
//	std::cout << "Backward\n";
	for (auto i = m_b.begin(); i != m_b.end(); i++)
	{
//		std::cout << i->first << " : " << i->second << "\n";
		backwardSum += i->second;
	}
	int height = 75*std::max(m_f.size(), m_b.size())+50;
	int width = height;
	std::string s;
	// 10% margin on all sides of image
	s = "<svg xmlns=\"http://www.w3.org/2000/svg\" version=\"1.1\" width = \""+std::to_string(width)+"\" height = \""+std::to_string(height)+"\" ";
	s += "preserveAspectRatio = \"none\" ";
	s += ">\n";

	int totalWork = backwardSum;
	const int forwardCopy = forwardSum;
	const int backwardCopy = backwardSum;
//	printf("--At f: %1f b:%1f work is %d\n", -1.0f, m_b.rbegin()->first, backwardSum);
	{
		// compute optimal partition
		auto fi = m_f.begin();
		auto bi = m_b.rbegin();
		backwardOptG = bi->first;
		forwardOptG = -1;
		forwardSum = 0;
		
		// fi points to the first unused entry
		// bi points to the last used entry
		while (true)
		{
			do {
				// Remove the next backwars entry
				backwardSum -= bi->second;
//				printf("Subtract %d from back\n", bi->second);
				bi++;
				if (bi == m_b.rend())
					break;
			} while (fi->first+bi->first > optCost);
			if (bi == m_b.rend())
				break;
			// Find all the necessary forward entries to maintain the vertex cover
			while (true)
			{
				forwardSum += fi->second;
//				printf("--Add %d to front\n", fi->second);
				fi++;
				// If we reach one end, we can remove everything in the other direction
				if (fi == m_f.end())
				{
					while (bi != m_b.rend())
					{
						backwardSum -= bi->second;
						bi++;
					}
					break;
				}
//				printf("%1.1f+%1.1f vs %1.1f\n", fi->first, bi->first, optCost);
				auto tmp = bi;
				tmp--;
				if (fi->first+(tmp)->first >= optCost)
					break;

			}
//			printf("Considering: %d + %d = %d\n", forwardSum, backwardSum, forwardSum+backwardSum);
			if (fi == m_f.end())
				break;
//			printf("--At f: %1f b:%1f work is %d\n", fi->first, (bi == m_b.rend())?-1:bi->first, forwardSum+backwardSum);
			
			if (forwardSum+backwardSum < totalWork)
			{
				totalWork = forwardSum+backwardSum;
				forwardOptG = fi->first;
				if (bi == m_b.rend())
					backwardOptG = -1;
				else
					backwardOptG = bi->first;
			}
		}
		if (fi != m_f.end())
		{
			forwardSum += fi->second;
			fi++;
		}
		fi--;
//		printf("--At f: %1f b:%1f work is %d\n", fi->first, -1.0f, forwardSum+backwardSum);
		if (forwardSum+backwardSum < totalWork)
		{
			totalWork = forwardSum+backwardSum;
			forwardOptG = fi->first;
			backwardOptG = -1;
		}
	}
	
	{
		printf("Forward/Backward/Min: %d %d %d %s\n", forwardCopy, backwardCopy, totalWork,
			   (std::min(forwardCopy, backwardCopy)!=totalWork)?"bidirectional wins!":"" );
	}
	if (filename == 0)
		return totalWork;
	
	//printf("Forward optimal dist: %1.1f; backwards optimal distances: %1.1f\n", forwardOptG, backwardOptG);
//	if (forwardOptG == -1 ||  backwardOptG == -1)
//	{
//		printf("Unidirectional solution\n");
//		return;
//	}
	
	// TODO: need to use cnt computed from below not i->first / j->first
	int fcnt = 0;
	for (auto i = m_f.begin(); i != m_f.end(); i++)
	{
		int bcnt = m_b.size()-1;
		for (auto j = m_b.rbegin(); j != m_b.rend(); j++)
		{
			auto tmp = i;
			tmp++;
//			if (fless(i->first + j->first, optCost))
			if (fless(i->first + j->first, optCost) && ((tmp) == m_f.end() || !fless((tmp)->first + j->first, optCost)))
			{
				s += SVGDrawLine(100, 75+75*fcnt, width-100, 75+75*bcnt, 1, Colors::black);
				break;
			}
			bcnt--;
		}
		fcnt++;
	}

	int cnt = 0;
	for (auto i = m_f.begin(); i != m_f.end(); i++)
	{
		std::string tmp = std::to_string((int)i->first);
		//std::cout << i->first << " : " << i->second << "\n";
		if (!fgreater(i->first, forwardOptG))
			s += SVGDrawCircle(100, 75+75*cnt, 25, Colors::lightblue);
		else
			s += SVGDrawCircle(100, 75+75*cnt, 25, Colors::lightgray);
		s += SVGFrameCircle(100, 75+75*cnt, 25, 1, Colors::black);
		s += SVGDrawText(100, 75+75*cnt, tmp.c_str(), Colors::black, 30);
		tmp = std::to_string((int)i->second);
		s += SVGDrawText(37.5, 75+75*cnt, tmp.c_str(), Colors::black, 20);
		cnt++;
	}
	cnt = 0;
	for (auto i = m_b.begin(); i != m_b.end(); i++)
	{
		std::string tmp = std::to_string((int)i->first);
		//std::cout << i->first << " : " << i->second << "\n";
		if (!fgreater(i->first, backwardOptG))
			s += SVGDrawCircle(width-100, 75+75*cnt, 25, Colors::lightblue);
		else
			s += SVGDrawCircle(width-100, 75+75*cnt, 25, Colors::lightgray);
		s += SVGFrameCircle(width-100, 75+75*cnt, 25, 1, Colors::black);
		s += SVGDrawText(width-100, 75+75*cnt, tmp.c_str(), Colors::black, 30);
		tmp = std::to_string((int)i->second);
		s += SVGDrawText(width-37.5, 75+75*cnt, tmp.c_str(), Colors::black, 20);
		cnt++;
	}
	
	s += SVGDrawText(width/2, height-100, optCostStr.c_str(), Colors::black, 30);
	optCostStr = "Forward A*: "+std::to_string(astarf.GetNecessaryExpansions());
	s += SVGDrawText(width/2, height-70, optCostStr.c_str(), Colors::black, 20);
	optCostStr = "Backward A*: "+std::to_string(astarb.GetNecessaryExpansions());
	s += SVGDrawText(width/2, height-45, optCostStr.c_str(), Colors::black, 20);
	optCostStr = "Optimal: "+std::to_string(totalWork);
	s += SVGDrawText(width/2, height-20, optCostStr.c_str(), Colors::black, 20);
	optCostStr = "Domain: "+e->GetName();
	s += SVGDrawText(width/2, 20, optCostStr.c_str(), Colors::black, 20);
	std::stringstream tmp;
	tmp << "Start: " << start;
	s += SVGDrawText(width/2, 45, tmp.str().c_str(), Colors::black, 20);
	tmp.str("");
	tmp << "Goal: " << goal;
	s += SVGDrawText(width/2, 70, tmp.str().c_str(), Colors::black, 20);

	if (filename != 0)
	{
		std::fstream svgFile;
		svgFile.open(filename, std::fstream::out | std::fstream::trunc);
		svgFile << s;
		svgFile << "</svg>";
		svgFile.close();
		printf("Generated SVG '%s'\n", filename);
	}
	return totalWork;
}

#endif /* WeightedVertexGraph_h */
