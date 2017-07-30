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
void GetWeightedVertexGraph(const state &start, const state &goal, environment *e, const char *filename)
{
	std::vector<state> path;
	TemplateAStar<state, action, environment> astarf;
	TemplateAStar<state, action, environment> astarb;
	astarf.GetPath(e, start, goal, path);
	astarb.GetPath(e, goal, start, path);
	double optCost = e->GetPathLength(path);
	double forwardOptG = -1;
	double backwardOptG = -1;
	int forwardSum = 0;
	int backwardSum = 0;
	std::string optCostStr = "Optimal cost: "+std::to_string((int)optCost);
	std::map<double, int> m_f, m_b;
	if (astarf.GetNecessaryExpansions() == 0 || astarb.GetNecessaryExpansions() == 0)
		return;
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
		return;
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
		return;
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
			// Remove the next backwars entry
			backwardSum -= bi->second;
			bi++;
			if (bi == m_b.rend())
				break;
			// Find all the necessary forward entries to maintain the vertex cover
			while (true)
			{
				forwardSum += fi->second;
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
				//printf("%1.1f+%1.1f vs %1.1f\n", fi->first, bi->first, optCost);
				if (fi->first+bi->first >= optCost)
					break;

			}
			//printf("Considering: %d + %d = %d\n", forwardSum, backwardSum, forwardSum+backwardSum);
			if (fi == m_f.end())
				break;
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
		if (forwardSum+backwardSum < totalWork)
		{
			totalWork = forwardSum+backwardSum;
			forwardOptG = (--fi)->first;
			backwardOptG = -1;
		}
	}
	//printf("Forward optimal dist: %1.1f; backwards optimal distances: %1.1f\n", forwardOptG, backwardOptG);
	if (forwardOptG == -1 ||  backwardOptG == -1)
	{
		return;
	}
	
	// TODO: need to use cnt computed from below not i->first / j->first
	int fcnt = 0;
	for (auto i = m_f.begin(); i != m_f.end(); i++)
	{
		int bcnt = m_b.size()-1;
		for (auto j = m_b.rbegin(); j != m_b.rend(); j++)
		{
			if (fless(i->first + j->first, optCost))
			{
				s += SVGDrawLine(100, 75+75*fcnt, width-100, 75+75*bcnt, 1, colors::black);
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
			s += SVGDrawCircle(100, 75+75*cnt, 25, colors::lightblue);
		else
			s += SVGDrawCircle(100, 75+75*cnt, 25, colors::lightgray);
		s += SVGFrameCircle(100, 75+75*cnt, 25, 1, colors::black);
		s += SVGDrawText(100, 75+75*cnt, tmp.c_str(), colors::black, 30);
		tmp = std::to_string((int)i->second);
		s += SVGDrawText(37.5, 75+75*cnt, tmp.c_str(), colors::black, 20);
		cnt++;
	}
	cnt = 0;
	for (auto i = m_b.begin(); i != m_b.end(); i++)
	{
		std::string tmp = std::to_string((int)i->first);
		//std::cout << i->first << " : " << i->second << "\n";
		if (!fgreater(i->first, backwardOptG))
			s += SVGDrawCircle(width-100, 75+75*cnt, 25, colors::lightblue);
		else
			s += SVGDrawCircle(width-100, 75+75*cnt, 25, colors::lightgray);
		s += SVGFrameCircle(width-100, 75+75*cnt, 25, 1, colors::black);
		s += SVGDrawText(width-100, 75+75*cnt, tmp.c_str(), colors::black, 30);
		tmp = std::to_string((int)i->second);
		s += SVGDrawText(width-37.5, 75+75*cnt, tmp.c_str(), colors::black, 20);
		cnt++;
	}
	
	s += SVGDrawText(width/2, height-100, optCostStr.c_str(), colors::black, 30);
	optCostStr = "Forward A*: "+std::to_string(astarf.GetNecessaryExpansions());
	s += SVGDrawText(width/2, height-70, optCostStr.c_str(), colors::black, 20);
	optCostStr = "Backward A*: "+std::to_string(astarb.GetNecessaryExpansions());
	s += SVGDrawText(width/2, height-45, optCostStr.c_str(), colors::black, 20);
	optCostStr = "Optimal: "+std::to_string(totalWork);
	s += SVGDrawText(width/2, height-20, optCostStr.c_str(), colors::black, 20);
	optCostStr = "Domain: "+e->GetName();
	s += SVGDrawText(width/2, 20, optCostStr.c_str(), colors::black, 20);
	std::stringstream tmp;
	tmp << "Start: " << start;
	s += SVGDrawText(width/2, 45, tmp.str().c_str(), colors::black, 20);
	tmp.str("");
	tmp << "Goal: " << goal;
	s += SVGDrawText(width/2, 70, tmp.str().c_str(), colors::black, 20);

	std::fstream svgFile;
	svgFile.open(filename, std::fstream::out | std::fstream::trunc);
	svgFile << s;
	svgFile << "</svg>";
	svgFile.close();
	printf("Generated SVG '%s'", filename);
}

#endif /* WeightedVertexGraph_h */
