/*
 * $Id: sample.cpp,v 1.23 2006/11/01 23:33:56 nathanst Exp $
 *
 *  sample.cpp
 *  hog
 *
 *  Created by Nathan Sturtevant on 5/31/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#include "Common.h"
#include "Driver.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include "OptimisticSearch.h"
#include "MapGenerators.h"
#include "AStarEpsilon.h"
#include "DynamicPotentialSearch.h"
#include "IOS.h"
#include <string>
#include "Plot2D.h"
#include "SVGUtil.h"

enum mode {
	kFindPathAStar = 0,
	kFindPathWAStar = 1,
	kFindPathOptimistic = 2,
	kFindPathAStar_e = 3,
	kFindPathDPS = 4,
	kFindPathXDP = 5,
	kFindPathXUP = 6,
	kFindPathIOS = 7,
	kFindPathIOSXDP = 8,
	kFindPathIOSXUP = 9,
};

double proveBound = 1.5, exploreBound = 3.0;

MapEnvironment *me = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
OptimisticSearch<xyLoc, tDirection, MapEnvironment> optimistic;
AStarEpsilon<xyLoc, tDirection, MapEnvironment> astar_e(1.5);
DynamicPotentialSearch<xyLoc, tDirection, MapEnvironment> dps;
ImprovedOptimisticSearch<xyLoc, tDirection, MapEnvironment> ios;

std::vector<xyLoc> path;
void GetMap(Map *m);

xyLoc start, goal;
double solutionCost = 0;

mode m = kFindPathAStar;

int stepsPerFrame = 1;
bool recording = false;
bool running = false;
bool mapChanged = true;

void StartSearch();
Map *ReduceMap(Map *inputMap);

Plotting::Plot2D plot;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 1200, 1200);
	return 0;
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Pause Simulation", "Pause simulation execution.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Run faster", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Run slower", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "A*", "A*", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "wA*", "wA*", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "wA*(XDP)", "wA*(XDP)", kAnyModifier, 'x');
	InstallKeyboardHandler(MyDisplayHandler, "wA*(XUP)", "wA*(XUP)", kAnyModifier, 'u');
	InstallKeyboardHandler(MyDisplayHandler, "Optimistic", "Optimistic", kAnyModifier, 't');
	InstallKeyboardHandler(MyDisplayHandler, "IOS", "Improved Optimistic", kAnyModifier, 'i');
	InstallKeyboardHandler(MyDisplayHandler, "IOS(XDP)", "Improved Optimistic", kAnyModifier, 'k');
	InstallKeyboardHandler(MyDisplayHandler, "IOS(XUP)", "Improved Optimistic", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "A*_e", "A*_e", kAnyModifier, 'e');
	InstallKeyboardHandler(MyDisplayHandler, "DPS", "DPS", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Weight", "Set weight", kAnyModifier, '0', '9');

	//InstallCommandLineHandler(MyCLHandler, "-map", "-map filename", "Selects the default map to be loaded.");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		//glClearColor(0.99, 0.99, 0.99, 1.0);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
//		SetNumPorts(windowID, 1);
		ReinitViewports(windowID, {-1.0f, -1.f, 0.f, 1.f}, kScaleToSquare);
		AddViewport(windowID, {0.f, -1.f, 1.f, 1.f}, kScaleToSquare); // kTextView

		Map *map = new Map(150, 150);
		GetMap(map);
//		//MakeRandomMap(map, 40);
//		//MakeMaze(map, 25);
//		BuildRandomRoomMap(map, 10, 60);
//
//		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
//		//Map *map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
//		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/lak303d.map");
//		//Map *map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
//		//Map *map = new Map("/Users/nathanst/hog2/maps/dao/den201d.map");
//
//		Map *m2 = ReduceMap(map);
//		m2->Save("/Users/nathanst/tmp.map");
//		delete map;
//		map = m2;

//		for (int x = 0; x < 300; x++)
//		for (int y = 0; y < 300; y++)
//		{
//			if (map->GetTerrainType(x, y) == kGround)
//				map->SetTerrainType(x, y, (random()%2)?kSwamp:kGround);
//		}
		
		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
		
		start.x = 0xFFFF;
		start.y = 0xFFFF;
	}
}

int frameCnt = 0;

void StepAlgorithms(int numSteps)
{
	if (running && (m == kFindPathOptimistic))
	{
		if (path.size() == 0)
		{
			for (int x = 0; x < numSteps && path.size() == 0; x++)
				optimistic.DoSingleSearchStep(path);
			if (path.size() != 0)
			{
				std::string s = ": "+std::to_string(optimistic.GetNodesExpanded())+" nodes expanded";
				appendTextToBuffer(s.c_str());
			}
		}
	}
	else if (running && ((m == kFindPathIOS || m == kFindPathIOSXDP || m == kFindPathIOSXUP)))
	{
		if (path.size() == 0)
		{
			for (int x = 0; x < numSteps && path.size() == 0; x++)
				ios.DoSingleSearchStep(path);
			if (path.size() != 0)
			{
				std::string s = ": "+std::to_string(ios.GetNodesExpanded())+" nodes expanded";
				appendTextToBuffer(s.c_str());
			}
		}
	}
	else if (running && m == kFindPathAStar_e)
	{
		if (path.size() == 0)
		{
			for (int x = 0; x < numSteps && path.size() == 0; x++)
				astar_e.DoSingleSearchStep(path);
			if (path.size() != 0)
			{
				std::string s = ": "+std::to_string(astar_e.GetNodesExpanded())+" nodes expanded";
				appendTextToBuffer(s.c_str());
			}
		}
	}
	else if (running && m == kFindPathDPS)
	{
		if (path.size() == 0)
		{
			for (int x = 0; x < numSteps && path.size() == 0; x++)
				dps.DoSingleSearchStep(path);
			if (path.size() != 0)
			{
				std::string s = ": "+std::to_string(dps.GetNodesExpanded())+" nodes expanded";
				appendTextToBuffer(s.c_str());
			}
		}
	}
	else if (running)
	{
		if (path.size() == 0)
		{
			for (int x = 0; x < numSteps && path.size() == 0; x++)
				astar.DoSingleSearchStep(path);
			if (path.size() != 0)
			{
				std::string s = ": "+std::to_string(astar.GetNodesExpanded())+" nodes expanded";
				appendTextToBuffer(s.c_str());

				for (int x = 0; x < astar.GetNumItems(); x++)
				{
					auto &item = astar.GetItem(x);
//					if (item.where == kClosedList)
//					{
//						printf("%f, %f\n", item.h/astar.GetWeight(), item.g/astar.GetWeight());
//					}
				}
			}
		}
	}
}

void GetPlotPoints()
{
	if (running && (m == kFindPathOptimistic ))
	{
		plot.Clear();
		plot.SetAxis(0, 0, solutionCost, solutionCost);
		for (int x = 0; x < optimistic.GetNumOpenItems(); x++)
		{
			const auto &item = optimistic.GetOpenItem(x);
			Plotting::Point p = {me->HCost(item.data, goal), item.g, 1.0/5.0, Colors::blue};
			plot.AddPoint(p);
		}
		for (int x = 0; x < optimistic.GetNumFocalItems(); x++)
		{
			const auto &item = optimistic.GetFocalItem(x);
			Plotting::Point p = {me->HCost(item.data, goal), item.g, 1.0/5.0, Colors::red};
			plot.AddPoint(p);
		}
		plot.NormalizeAxes();

		double target;
		//		astar.DoSingleSearchStep(path);
		{
			double w = optimistic.GetOptimalityBound();
			double g, h;
			const auto &s = optimistic.CheckNextOpenNode();
			optimistic.GetOpenListGCost(s, g);
			h = me->HCost(s, goal);
			target = w*(g+h);
			static Plotting::Line upperBound("");
			static Plotting::Line lowerBound("");
			static Plotting::Line projectedBound("");
			upperBound.Clear();
			upperBound.SetWidth(1);
			upperBound.SetColor(Colors::darkgreen);
			upperBound.AddPoint(0, target);
			upperBound.AddPoint(plot.GetMaxX(), target);
			
			lowerBound.Clear();
			lowerBound.SetWidth(1);
			lowerBound.SetColor(Colors::darkgreen);
			lowerBound.AddPoint(0, g+h);
			lowerBound.AddPoint(plot.GetMaxX(), g+h);

			projectedBound.Clear();
			projectedBound.SetWidth(1);
			projectedBound.SetColor(Colors::darkgreen);
			projectedBound.AddPoint(h, g);
			projectedBound.AddPoint(0, g+h);

			plot.AddLine(&lowerBound);
			plot.AddLine(&upperBound);
			plot.AddLine(&projectedBound);
		}
		
		if (0)
		{
			const auto &s = optimistic.CheckNextFocalNode();
			double g, h;
			optimistic.GetOpenListGCost(s, g);
			h = me->HCost(s, goal);
			
			static Plotting::Line line("");
			line.SetWidth(3.0f/5.0f);
			line.SetColor(Colors::blue);
			line.Clear();
			line.AddPoint(h, 0);
			line.AddPoint(h, target);
			// if g == 0 h = target/w
			plot.AddLine(&line);
		}

	}
	else if (running && ((m == kFindPathIOS || m == kFindPathIOSXDP || m == kFindPathIOSXUP)))
	{
		plot.Clear();
		plot.SetAxis(0, 0, solutionCost, solutionCost);
		for (int x = 0; x < ios.GetNumOpenItems(); x++)
		{
			const auto &item = ios.GetOpenItem(x);
			Plotting::Point p = {me->HCost(item.data, goal), item.g, 1.0/5.0, Colors::blue};
			plot.AddPoint(p);
		}
		plot.NormalizeAxes();
		
		double target;

		// TODO: Use the largest f-cost expanded for drawing bound lines
		{
			double w = optimistic.GetOptimalityBound();
			double g, h;
			const auto &s = ios.CheckNextNode();
			ios.GetOpenListGCost(s, g);
			h = me->HCost(s, goal);
			target = w*(g+h);
			static Plotting::Line upperBound("");
			static Plotting::Line lowerBound("");
			static Plotting::Line projectedBound("");
			upperBound.Clear();
			upperBound.SetWidth(1);
			upperBound.SetColor(Colors::darkgreen);
			upperBound.AddPoint(0, target);
			upperBound.AddPoint(plot.GetMaxX(), target);
			
			lowerBound.Clear();
			lowerBound.SetWidth(1);
			lowerBound.SetColor(Colors::darkgreen);
			lowerBound.AddPoint(0, g+h);
			lowerBound.AddPoint(plot.GetMaxX(), g+h);
			
			projectedBound.Clear();
			projectedBound.SetWidth(1);
			projectedBound.SetColor(Colors::darkgreen);
			projectedBound.AddPoint(h, g);
			projectedBound.AddPoint(0, g+h);
			
			plot.AddLine(&lowerBound);
			plot.AddLine(&upperBound);
			plot.AddLine(&projectedBound);
		}
		
		
	}	else if (running && m == kFindPathAStar_e)
	{
		plot.Clear();
		plot.SetAxis(0, 0, solutionCost, solutionCost);

		for (int x = 0; x < astar_e.GetNumOpenItems(); x++)
		{
			const auto &item = astar_e.GetOpenItem(x);
			Plotting::Point p = {me->HCost(item.data, goal), item.g, 1.0/5.0, Colors::blue};
			plot.AddPoint(p);
		}
		for (int x = 0; x < astar_e.GetNumFocalItems(); x++)
		{
			const auto &item = astar_e.GetFocalItem(x);
			Plotting::Point p = {me->HCost(item.data, goal), item.g, 1.0/5.0, Colors::red};
			plot.AddPoint(p);
		}
		plot.NormalizeAxes();

		double target;
		//		astar.DoSingleSearchStep(path);
		{
			double w = astar_e.GetOptimalityBound();
			double g, h;
			const auto &s = astar_e.CheckNextOpenNode();
			astar_e.GetOpenListGCost(s, g);
			h = me->HCost(s, goal);
			target = w*(g+h);

			plot.AddPoint({h, g, 3.0/5.0, Colors::purple});

			
			static Plotting::Line upperBound("");
			static Plotting::Line lowerBound("");
			static Plotting::Line projectedLowerBound("");
			upperBound.Clear();
			upperBound.SetWidth(1);
			upperBound.SetColor(Colors::darkgreen);
			upperBound.AddPoint(0, w*(g+h));
			upperBound.AddPoint(plot.GetMaxX(), w*(g+h));
//			printf("Min optimal f: %1.2f, upper bound %1.2f\n", g+h, w*(g+h));
			
			lowerBound.Clear();
			lowerBound.SetWidth(1);
			lowerBound.SetColor(Colors::darkgreen);
			lowerBound.AddPoint(0, g+h);
			lowerBound.AddPoint(plot.GetMaxX(), g+h);

			projectedLowerBound.Clear();
			projectedLowerBound.SetWidth(1);
			projectedLowerBound.SetColor(Colors::darkgreen);
			projectedLowerBound.AddPoint(h, g);
			projectedLowerBound.AddPoint(0, g+h);


			plot.AddLine(&lowerBound);
			plot.AddLine(&upperBound);
			plot.AddLine(&projectedLowerBound);
		}
		
		if (1)
		{
			const auto &s = astar_e.CheckNextFocalNode();
			double g, h;
			astar_e.GetFocalListGCost(s, g);
			h = me->HCost(s, goal);

			static Plotting::Line projectedUpperBound("");
			projectedUpperBound.Clear();
			projectedUpperBound.SetWidth(1);
			projectedUpperBound.SetColor(Colors::darkgreen);
			projectedUpperBound.AddPoint(h, g);
			projectedUpperBound.AddPoint(0, g+h);

			plot.AddLine(&projectedUpperBound);
		}
	}
	else if (running && m == kFindPathDPS)
	{
		static Plotting::Line l("");
		l.Clear();
		l.SetColor(Colors::darkgreen);
		l.SetWidth(1);
		plot.Clear();
		plot.SetAxis(0, 0, solutionCost, solutionCost);
		
		double w = dps.GetOptimalityBound();
		double fmin = dps.GetBestFMin();
		double g, h;
		double maxh = 0;
		double pr = 0;
		dps.ResetIterator();
		while (dps.GetNext(g, h))
		{
			if (h == 0)
				continue;
			pr = std::max(pr, (fmin*w-g)/h);
			maxh = std::max(h, maxh);
			Plotting::Point p = {h, g, 1.0/5.0, Colors::red};
			plot.AddPoint(p);
		}
//		std::cout << "--> Best priority: " << pr << "\n";
		// pr = (fmin*w-g)/h
		// h*pr = fmin*w-g
		// g = fmin*w + h*pr
		// h = (fmin*w-g)/pr
		l.AddPoint(0, fmin*w);
		l.AddPoint(fmin*w/pr, 0);
		plot.AddLine(&l);
		//printf("Bounds: x: [%1.2f, %1.2f] y:[%1.2f, %1.2f]\n", plot.GetMinX(), plot.GetMaxX(), plot.GetMinY(), plot.GetMaxY());
	}
	else if (running)
	{
		plot.Clear();
		plot.SetAxis(0, 0, solutionCost, solutionCost);
		//		astar.DoSingleSearchStep(path);
		double w = astar.GetWeight();
		double g, h;
		static Plotting::Line line("");
		line.SetWidth(1);
		line.Clear();
		const auto &next = astar.CheckNextNode();
		h = me->HCost(next, goal);
		astar.GetOpenListGCost(next, g);
		double target = astar.Phi(h, g);
		// draw priority function
		switch (m) {
			case kFindPathXDP:
				// XDP: (b−x)(bw−wx+x)/b
				for (double x = 0; x <= target; x+=0.5)
					line.AddPoint(x, (target-x)*(target*w-w*x+x)/target);
				break;
			case kFindPathXUP:
				// XUP: (b-x)(bw+wx-x)/b
				for (double x = 0; x <= target; x+=0.5)
					line.AddPoint(x, (target-x)*(target*w+w*x-x)/target);
				break;
			default:
			{
				// f(g, h) = g+w*h = target
				// if h == 0, g = target
				line.AddPoint(0, target);
				line.AddPoint(h, g);
				// if g == 0 h = target/w
				line.AddPoint(target/w, 0);
			}
				break;
		}
		line.SetColor(Colors::blue);
		plot.AddLine(&line);

		for (int x = 0; x < astar.GetNumOpenItems(); x++)
		{
			const auto &item = astar.GetOpenItem(x);
			Plotting::Point p = {me->HCost(item.data, goal), item.g, 1.0/5.0, Colors::red};
			plot.AddPoint(p);
		}
	}

	plot.NormalizeAxes();
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	
	if (viewport == 1)
	{
		GetPlotPoints();
		plot.Draw(display);
	}
	if (viewport == 0)
	{
		if (mapChanged == true)
		{
			display.StartBackground();
			me->Draw(display);
			display.EndBackground();
			mapChanged = false;
		}
		
		
		if (start.x != 0xFFFF && start.y != 0xFFFF && !running)
		{
			me->SetColor(Colors::green);
			me->Draw(display, start);
			me->SetColor(Colors::red);
			me->DrawLine(display, start, goal, 10);
		}
		
		StepAlgorithms(stepsPerFrame);
		
		if (running && (m == kFindPathOptimistic))
		{
			optimistic.Draw(display);
			
			if (path.size() != 0)
			{
				me->SetColor(Colors::green);
				for (int x = 1; x < path.size(); x++)
				{
					me->DrawLine(display, path[x-1], path[x], 5);
				}
			}
		}
		else if (running && ((m == kFindPathIOS || m == kFindPathIOSXDP || m == kFindPathIOSXUP)))
		{
			ios.Draw(display);
			
			if (path.size() != 0)
			{
				me->SetColor(Colors::green);
				for (int x = 1; x < path.size(); x++)
				{
					me->DrawLine(display, path[x-1], path[x], 5);
				}
			}
		}
		else if (running && (m == kFindPathAStar_e))
		{
			astar_e.Draw(display);
			
			if (path.size() != 0)
			{
				me->SetColor(Colors::green);
				for (int x = 1; x < path.size(); x++)
				{
					me->DrawLine(display, path[x-1], path[x], 5);
				}
			}
		}
		else if (running && (m == kFindPathDPS))
		{
			dps.Draw(display);
			
			if (path.size() != 0)
			{
				me->SetColor(Colors::green);
				for (int x = 1; x < path.size(); x++)
				{
					me->DrawLine(display, path[x-1], path[x], 5);
				}
			}
		}
		else if (running)
		{
			astar.Draw(display);
			
			if (path.size() != 0)
			{
				me->SetColor(Colors::green);
				for (int x = 1; x < path.size(); x++)
				{
					me->DrawLine(display, path[x-1], path[x], 5);
				}
			}
		}
	}
	
	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/sub-%d%d%d%d.svg",
				(frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		MakeSVG(GetContext(windowID)->display, fname, 1200, 1200, 0);
		printf("Saved %s\n", fname);
		frameCnt++;
	}
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	if (maxNumArgs <= 1)
		return 0;
	strncpy(gDefaultMap, argument[1], 1024);
	return 2;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '[':
		{
			stepsPerFrame /= 2;
			break;
		}
		case ']':
		{
			if (stepsPerFrame < 32768)
				stepsPerFrame *= 2;
			if (stepsPerFrame == 0)
				stepsPerFrame = 1;
			break;
		}
		case 'r':
			recording = !recording;
			break;
		case 'a': m = kFindPathAStar; StartSearch(); break;
		case 'w': m = kFindPathWAStar; StartSearch(); break;
		case 't': m = kFindPathOptimistic; StartSearch(); break;
		case 'e': m = kFindPathAStar_e; StartSearch(); break;
		case 'd': m = kFindPathDPS; StartSearch(); break;
		case 'x': m = kFindPathXDP; StartSearch(); break;
		case 'u': m = kFindPathXUP; StartSearch(); break;
		case 'i': m = kFindPathIOS; StartSearch(); break;
		case 'k': m = kFindPathIOSXDP; StartSearch(); break;
		case 'm': m = kFindPathIOSXUP; StartSearch(); break;
		case '1': proveBound = 1.5; StartSearch(); break;
		case '2': proveBound = 2; StartSearch(); break;
		case '3': proveBound = 3; StartSearch(); break;
		case '4': proveBound = 4; StartSearch(); break;
		case '5': proveBound = 5; StartSearch(); break;

		case '6': exploreBound = 2; StartSearch(); break;
		case '7': exploreBound = 4; StartSearch(); break;
		case '8': exploreBound = 8; StartSearch(); break;
		case '9': exploreBound = 16; StartSearch(); break;
		case '0': exploreBound = 32; StartSearch(); break;

			
		case 'p':
			stepsPerFrame = 0;
			break;
		case 'o':
		{
			StepAlgorithms(1);
		}
			break;
		case '?':
		{
		}
			break;
		case 's':
			break;
		case 'l':
			break;
		default:
			break;
	}
	
}

bool MyClickHandler(unsigned long , int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	if (button != kLeftButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround || me->GetMap()->GetTerrainType(x, y) == kSwamp)
			{
				start.x = x;
				start.y = y;
				goal = start;
				printf("Hit (%d, %d)\n", x, y);
				running = false;
			}
			return true;
		}
		case kMouseDrag:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround || me->GetMap()->GetTerrainType(x, y) == kSwamp)
			{
				goal.x = x;
				goal.y = y;
			}
			break;
		}
		case kMouseUp:
		{
			int x, y;
			me->GetMap()->GetPointFromCoordinate(loc, x, y);
			if (me->GetMap()->GetTerrainType(x, y) == kGround || me->GetMap()->GetTerrainType(x, y) == kSwamp)
			{
				goal.x = x;
				goal.y = y;
				printf("UnHit (%d, %d)\n", x, y);
			}

			StartSearch();
			return true;
		}
	}
	return false;
}

void StartSearch()
{
	if (start == goal)
		return;
	if (m == kFindPathOptimistic)
	{
		submitTextToBuffer("Searching with Optimistic");
		std::string tmp = "("+std::to_string(proveBound)+", "+std::to_string(exploreBound)+")";
		appendTextToBuffer(tmp.c_str());
		optimistic.SetWeight(exploreBound);
		optimistic.SetOptimalityBound(proveBound);
		optimistic.GetPath(me, start, goal, path);
		solutionCost = me->GetPathLength(path);
		optimistic.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathIOS || m == kFindPathIOSXDP || m == kFindPathIOSXUP)
	{
		submitTextToBuffer("Searching with Improved Optimistic Search ");
		std::string tmp = "";
		if (m == kFindPathIOSXDP)
		{
			tmp+= "XDP";
			ios.SetPhi([=](double x,double y){return (y+(2*exploreBound-1)*x+sqrt((y-x)*(y-x)+4*exploreBound*y*x))/(2*exploreBound);});
		}
		else if (m == kFindPathIOSXUP)
		{
			tmp+= "XUP";
			ios.SetPhi([=](double x,double y){return (y+x+sqrt((y+x)*(y+x)+4*exploreBound*(exploreBound-1)*x*x))/(2*exploreBound);});
		}
		else {
			ios.SetPhi(([=](double x, double y){return y/(exploreBound)+x;}));
		}
		tmp += "("+std::to_string(proveBound)+", "+std::to_string(exploreBound)+")";
		appendTextToBuffer(tmp.c_str());
		ios.SetWeight(exploreBound);
		ios.SetOptimalityBound(proveBound);
		ios.GetPath(me, start, goal, path);
		solutionCost = me->GetPathLength(path);
		ios.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathAStar)
	{
		submitTextToBuffer("Searching with A*");
		astar.SetWeight(1.0);
		astar.GetPath(me, start, goal, path);
		solutionCost = me->GetPathLength(path);
		astar.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathWAStar)
	{
		submitTextToBuffer("Searching with wA*");
		std::string tmp = "("+std::to_string(proveBound)+")";
		appendTextToBuffer(tmp.c_str());
		astar.SetWeight(proveBound);
		astar.GetPath(me, start, goal, path);
		for (int x = 0; x < astar.GetNumOpenItems(); x++)
		{
			double g = astar.GetOpenItem(x).g;
			double h = astar.GetOpenItem(x).h;
			solutionCost = std::max(solutionCost, g+h);
		}
//		solutionCost = me->GetPathLength(path);
		astar.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathXDP)
	{
		submitTextToBuffer("Searching with wA*(XDP)");
		std::string tmp = "("+std::to_string(proveBound)+")";
		appendTextToBuffer(tmp.c_str());
		astar.SetWeight(proveBound);
		astar.SetPhi([=](double x,double y){return (y+(2*proveBound-1)*x+sqrt((y-x)*(y-x)+4*proveBound*y*x))/(2*proveBound);});
		astar.GetPath(me, start, goal, path);
		for (int x = 0; x < astar.GetNumOpenItems(); x++)
		{
			solutionCost = std::max(solutionCost, astar.GetOpenItem(x).f);
		}
		//		solutionCost = me->GetPathLength(path);
		astar.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathXUP)
	{
		submitTextToBuffer("Searching with wA*(XUP)");
		std::string tmp = "("+std::to_string(proveBound)+")";
		appendTextToBuffer(tmp.c_str());
		astar.SetWeight(proveBound);
		astar.SetPhi([=](double x,double y){return (y+x+sqrt((y+x)*(y+x)+4*proveBound*(proveBound-1)*x*x))/(2*proveBound);});
		astar.GetPath(me, start, goal, path);
		for (int x = 0; x < astar.GetNumOpenItems(); x++)
		{
			solutionCost = std::max(solutionCost, astar.GetOpenItem(x).f);
		}
		//		solutionCost = me->GetPathLength(path);
		astar.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathAStar_e)
	{
		submitTextToBuffer("Searching with A*_e");
		std::string tmp = "("+std::to_string(proveBound)+")";
		astar_e.SetOptimalityBound(proveBound);
		astar_e.GetPath(me, start, goal, path);
		solutionCost = me->GetPathLength(path);
		astar_e.InitializeSearch(me, start, goal, path);
		running = true;
	}
	else if (m == kFindPathDPS)
	{
		submitTextToBuffer("Searching with DPS");
		std::string tmp = "("+std::to_string(proveBound)+")";
		dps.SetOptimalityBound(proveBound);
		dps.GetPath(me, start, goal, path);
		solutionCost = me->GetPathLength(path);
		dps.InitializeSearch(me, start, goal, path);
		running = true;
	}
}


#include "GraphEnvironment.h"

int LabelConnectedComponents(Graph *g);

Map *ReduceMap(Map *inputMap)
{
	Graph *g = GraphSearchConstants::GetGraph(inputMap);
	
	int biggest = LabelConnectedComponents(g);
	
	Map *m = new Map(inputMap->GetMapWidth(), inputMap->GetMapHeight());
	for (int x = 0; x < inputMap->GetMapWidth(); x++)
	{
		for (int y = 0; y < inputMap->GetMapHeight(); y++)
		{
			if (inputMap->GetTerrainType(x, y) == kTrees)
				m->SetTerrainType(x, y, kTrees);
			else if (inputMap->GetTerrainType(x, y) == kWater)
				m->SetTerrainType(x, y, kWater);
			else m->SetTerrainType(x, y, kOutOfBounds);
		}
	}
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		if (g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest)
		{
			int theX, theY;
			theX = g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX);
			theY = g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY);
			if (g->GetNode(inputMap->GetNodeNum(theX+1, theY)) &&
				(g->GetNode(inputMap->GetNodeNum(theX+1, theY))->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest) &&
				(!g->FindEdge(x, inputMap->GetNodeNum(theX+1, theY))))
			{
				m->SetTerrainType(theX, theY, kOutOfBounds);
			}
			else if (g->GetNode(inputMap->GetNodeNum(theX, theY+1)) &&
					 (g->GetNode(inputMap->GetNodeNum(theX, theY+1))->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest) &&
					 (!g->FindEdge(x, inputMap->GetNodeNum(theX, theY+1))))
			{
				m->SetTerrainType(theX, theY, kOutOfBounds);
			}
			//			else if (g->GetNode(inputMap->GetNodeNum(theX+1, theY+1)) &&
			//					 (g->GetNode(inputMap->GetNodeNum(theX+1, theY+1))->GetLabelL(GraphSearchConstants::kTemporaryLabel) == biggest) &&
			//					 (!g->FindEdge(x, inputMap->GetNodeNum(theX+1, theY+1))))
			//			{
			//				m->SetTerrainType(theX, theY, kOutOfBounds);
			//			}
			else {
				if (inputMap->GetTerrainType(theX, theY) == kSwamp)
					m->SetTerrainType(theX, theY, kSwamp);
				else
					m->SetTerrainType(theX, theY, kGround);
			}
		}
		else if (inputMap->GetTerrainType(g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX),
										  g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY)) == kGround)
			m->SetTerrainType(g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapX),
							  g->GetNode(x)->GetLabelL(GraphSearchConstants::kMapY), kTrees);
	}
	return m;
}

int LabelConnectedComponents(Graph *g)
{
	for (int x = 0; x < g->GetNumNodes(); x++)
		g->GetNode(x)->SetLabelL(GraphSearchConstants::kTemporaryLabel, 0);
	int group = 0;
	std::vector<int> groupSizes;
	for (int x = 0; x < g->GetNumNodes(); x++)
	{
		if (g->GetNode(x)->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
		{
			group++;
			groupSizes.resize(group+1);
			
			std::vector<unsigned int> ids;
			ids.push_back(x);
			while (ids.size() > 0)
			{
				unsigned int next = ids.back();
				ids.pop_back();
				if (g->GetNode(next)->GetLabelL(GraphSearchConstants::kTemporaryLabel) != 0)
					continue;
				groupSizes[group]++;
				g->GetNode(next)->SetLabelL(GraphSearchConstants::kTemporaryLabel, group);
				for (int y = 0; y < g->GetNode(next)->GetNumEdges(); y++)
				{
					edge *e = g->GetNode(next)->getEdge(y);
					if (g->GetNode(e->getFrom())->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
						ids.push_back(e->getFrom());
					if (g->GetNode(e->getTo())->GetLabelL(GraphSearchConstants::kTemporaryLabel) == 0)
						ids.push_back(e->getTo());
				}
			}
		}
	}
	int best = 0;
	for (unsigned int x = 1; x < groupSizes.size(); x++)
	{
		printf("%d states in group %d\n", groupSizes[x], x);
		if (groupSizes[x] > groupSizes[best])
			best = x;
	}
	printf("Keeping group %d\n", best);
	return best;
	//	kMapX;
}


void GetMap2(Map *m)
{
	m->Scale(150, 150);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@.@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@.@@@@@@@@@@.@@@@@@@@@@.@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT@.........@.........@.........@.........@...................@.........@...................@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.............................@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@...................@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@...................@.........@.........@.........@.........@...................@...................@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@@@@@@@@@T@@@@@@@@@.@@.@@@@@@@@@@@@@.@@@@.@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@.@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@.@@@@@@.@@@@@@@TTTTTTTTT@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@...................@.........@.........@.........TTTTTTTTTT@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@...................@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@TTTTTTTTT@...................@.........@.........@.........@.........@...................@TTTTTTTTT@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@...................@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@.@@@@@@.@@@@@.@@@@@@@@@.........@.........@.........@.........@.........@.........@TTTTTTTTTTTTTTTTTTT@...................@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.............................@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@...................@.........@.........@.........@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@.@@@@@@@@@@@@@@.@@@@@@@@@@@@@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@...................@.........@.........@.........@.........@TTTTTTTTT@.........@...................@.........@.........@.........@TTTTTTTTTTTTTTTTTTT@.........@.........@...................@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@...................@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@...................@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@...................@.........@.........@TTTTTTTTT@@@@@@@@@@@@.@@@@@@@@.@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@.@@@@@@@@@@@@@@@.@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@T@@@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@...................@.........@TTTTTTTTT@.........@.........@.............................@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT..........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@...................@TTTTTTTTT@.........@...................@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@@@@@@@@@@@@@@@@.@@@@@@@@@@.@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@.@@@@@@@@@@@@@@@T@@@@@@@@@T@@@@@@@.........@.........@.........@TTTTTTTTTTTTTTTTTTT@.........@.........@.........@...................@.........@.........@.........@TTTTTTTTT@TTTTTTTTT..........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@...................@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@...................@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@...................@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@...................@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@@@@.@@@@@@@@@@@@@@.@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@.@@@@@@.@@@@@@@@@@@.@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@...................@TTTTTTTTT..........@...................@.........@.........@.........@...................@.........@.........@...................@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@...................@.........@.........@.........@...................@.........@.........@.........@.........@TTTTTTTTT@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@@.@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@.@@@@.@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@.@@@@@@@@@@@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@...................@.........@.........@...................@...................@.........@.........@.........@.........@...................@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@@.@@@@@@@@@@@@@@@.@@@@.@@@@@@@@@@@@@.@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@.@@@@@@@@@@@@@@.@@@@@@.@@@@@.@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.......................................@.........@.........@.........@.........@...................@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@.@@@@@@@@@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@...................@.........@.........@.........@.......................................@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@...................@@@.@@@@@@@@@@@@@@@.@@@@@@.@@@@@@@@.@@@@@@@@@@@@@@@@@@@.@@@@@@.@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@.@@@@@.@@@@@@@@@@@@@.@@@@@@@@.@@@@@@@@@@@@@@.@@@@@@@@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@...................@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@@@@@@@@@@@@@@@@.@@@@@@@@@@.@@@.@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@TTTTTTTTT@.........@.........@.........@.........@.........@.........@...................@.........@...................@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@...................@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@...................@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@...................@.........@...................@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@...................@.........@...................@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@T@@@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT..........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@...................@.........@.........@.........@...................@TTTTTTTTT@TTTTTTTTT@TTTTTTTTTTTTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@...................@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@...................@.........@.........@...................@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTTTTTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@...................@...................@.........@.........@...................@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@@@@@.@@@@@@@@@@.@@@@@@@@.@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@.@@@@@@.@@@@@@@@@@.@@@@@@.@@@@@@@@@@@@@.@@@@@@.@@@@@@@T@@@@@@@@@@@@@@T@@@@@@@@@@T@@@@@@@T@@@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@...................@.........@.........@.........@.........@.........@.........@TTTTTTTTTTTTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@...................@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@...................@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@...................@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@@@@@@@@@.@@@@@@@@@@@.@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@T@@@@@@@@@@@@@@@@@@@@@@T@@@@T@@@@@@@.........@.........@.........@.........@.........@...................@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@...................@.........@.........@.........@TTTTTTTTT@TTTTTTTTTTTTTTTTTTT@TTTTTTTTT@TTTTTTTTTTTTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTTTTTTTTTTTT@TTTTTTTTT@TTTTTTTTT@...................@.........@...................@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@.........@.........@.........@.........@.........@.........@.........@.........@.........@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT..........@.........@.........@.........@.........@.........@.........@...................@TTTTTTTTT@TTTTTTTTT@TTTTTTTTT@TTTTTTTTTTTTTTTTTTT@TTTTTTTTT";
	int which = 0;
	for (int y = 0; y < m->GetMapHeight(); y++)
	{
		for (int x = 0; x < m->GetMapWidth(); x++)
		{
			if (map[which] == '.')
				m->SetTerrainType(x, y, kGround);
			else if (map[which] == 'S')
				m->SetTerrainType(x, y, kSwamp);
			else
				m->SetTerrainType(x, y, kOutOfBounds);
			which++;
		}
	}
	mapChanged = true;
}

void GetMap1(Map *theMap)
{
	theMap->Scale(148, 139);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@....@@@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@@....@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@@@@.@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@.@@@@..@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@...@@@..@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@....@..@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@.....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@..@.......................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@...........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@........@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....................@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...................@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.................@@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@.@@@@@@@...............@@@@@@@@..........@@@@@..@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@.......@@@@@.............@@@@@@@@..........@@@@@...@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@.........@@@@@...........@@@@@@@@........@@@@@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@..........@@@@@@@@.........@@@@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@..........@@@@@@@.........@@@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@.........@@@@@@@.........@@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@..........@@@@@@.........@@@@@@.....@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@.............@..@.........@@@@@@.....@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@..........................@@@@@@.....@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@.........................@@@@@@......@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@..........................@@@@@@.....@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@..........................@@@@@@.....@@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@..........................@@@@@@.....@@@@...................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........................@@.................@@@@@@.....@@@@...@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@.........@@@@...............@@@@@@@....@@@@...@@@@@......@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@.......@@@@@@.............@@@@@......@@@@...@@@@@@.@...@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@.......@@@@@@@............@@@@@......@@@@...@@@@@@@@@..@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@.......@@@@@@@@............@@@@......@@@@....@@@@@@@@..@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@.......@@@@@@@@@.............@@@.......@@.....@@@@@@@..@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@.......@@@@@@@@................@@................@@.@..@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@.......@@@@@@@@..........@.............................@@@@..................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@........@@@@@@@..........@@@...........................@@@@..@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@@@..@@@@.........@@@@@@..........@@@@@.........................@@@@..@@@@......@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@@@@..........@@@@@..........@@@@@@........................@@@@..@@@@@@....@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@@@............@@@..........@@@@@@@.......................@@@@..@@@@@@@...@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@.........................@@@@@@........................@@@@..@@@@@@@...@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@@......................@@@@@@....................@@...@@@....@@@@@...@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@@.....................@@@@@@....................@@@@...@......@@@...@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@.....@.....@@@@@@...................@@@@@@....................@@@@@@..............@@@@..................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@.....@@@.....@@@@@..................@@@@@@@........@.@@.......@@@..@@@............@@@@..@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@.....@@@@.................@@@@@..........@@@@@@.....@@@....@@@..........@@@@@@@@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@.....@..................@@@@@...........@@@@@@....@@@......@@@........@@@@@@@@@@@@.......@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@........@@@...........@@@@@...........@@@@@@....@@@........@@@......@@@@.@@@@@@@@@.....@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@........@@@@.........@@@@@@............@@@@....@@@..........@@@....@@@@..@.@@@@@@@....@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@..@......@@@@@.......@@@@@@@.............@@....@@@............@@@...@@@.....@@@@@@....@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@..@@@.......@@@@.....@@@@@@@@@.................@@@..............@@....@.......@@@@....@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@.......@@@@...@@@@@@@@...................@@...............@@@............@@....@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@.......@@@@.@@@@@@@@...................@@.........@@......@@@................@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@.......@@@@@@@@@@@@...................@@@.........@@@.....@@@...............@@@@.............@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@.......@@@@@@@@@@@@......@@...........@@@@..........@@......@...............@@@@.............@@@@@@....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@................@@@@@@@@@@@@......@@@@.........@@@@@@.........@@.....................@@@@.............@@@@@@@...@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@@@@@@@......@@@@@@........@@@@@@@........@@@.....@@............@@@@.............@@@@@@@@...@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@@@@@.....@@@@@@@@@......@@@@@@@@@@....@@@@.....@@...........@@@@.............@@@@@@@@@....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@@@@@@@....@@@@@@@@@@@......@@@@@@@@@@@@@@..@@.................@@@.............@@@@@@@@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@@@@@@@....@@@@@@@@@@@@@@@S.....@@@@@@@@@@.....@@..................@.............@@@@@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@@@@@@@.....@@@@@@@@@@@@@@@SS.....@@@@@@@@.......@...............................@@@@@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@@@@@......@@@@@@@@@@@@@@SSS.......@@@@@.......................................@@@@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@@.@@........@@@@@@@@@@@@SSSS........@@@.......................................@@@@@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@@SSSSS@@.......@.......................................@@@@@@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@SSSSS@@@@@............................................@@@@@@@@...................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@SSSSSS@@@@@@@..........................................@@@@@@@@.....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@SSSSS@@@@@@@@@........................................@@@@@@@@..........@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@@SSSS@@@@@@@@@@@......................................@@@@@@@@..........@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@@@SSSS@@@@@@@@@@@@S...................................@@@@@@@@..........@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@..SSS@@@@@@@@@@@@@@@SS................................@@@@@@@@............@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@...SS@@@@@@@@@@@@@@@SSS...............................@@@@@@@@..............@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@@@@.....S@@@@@@@@@@@@SSSS................................@@@@@@@...........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@...........@@@@@@@@@@SSSSS@@...............................@@.@@@.....@....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@...........@@@@@@@@@SSSSS@@@@...............................@........@@@..................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@.............@@@@@@@SSSSS@@@@@@......................................@@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@SSSSS@@@@@@@@......................................@@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@SSSS@@@@@@@@@@......................................@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@@SSS@@@@@@@@@@@@......................................@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@SSSS@@@@@@@@@@@@@@......................................@@@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@......@@@@@@@@@@@@@@................................@......@@@@@......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@.....@@@@@@@@@@@@@@.............@...................@@......@@@@@....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@.@@@......@@@@@@@@@@@@.............@@@...................@@......@@@@@..@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@@@...........@@@@@@@@@@.............@@@@@...................@@......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@@@@...........@@@@@@@@@.............@@@@@@...........@@.......@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@.............@@@@@@@@.............@@@@@@@..........@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@@.............@@@@@@............@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@@@............@@@@@@..............@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@@@@@...........@@@@@@@............................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@@.....@.......@@@@@@@@...........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@......@@.....@@@@@@@@...........@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@@@@.......@@@...@@@@@@@@.............@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@...........@@@@@@@@@@@@...............@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@...........@@@@@@@@@@@.......@@........@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@............@@@@@@@@@@.......@@@@........@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@@@........@@@@...................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@............@@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@..............@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@................@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@...................@@@@......@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@.....................@@@@.....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@@@.......................@@@@...@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@@@.........................@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@@..........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@.@@...........@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@..............@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.................@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@................................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
	int which = 0;
	for (int y = 0; y < theMap->GetMapHeight(); y++)
	{
		for (int x = 0; x < theMap->GetMapWidth(); x++)
		{
			if (map[which] == '.')
				theMap->SetTerrainType(x, y, kGround);
			else if (map[which] == 'S')
				theMap->SetTerrainType(x, y, kSwamp);
			else
				theMap->SetTerrainType(x, y, kOutOfBounds);
			which++;
		}
	}
	mapChanged = true;
}

void GetMap(Map *map)
{
//	map->Load("/Users/nathanst/hog2/maps/dao/orz900d.map");
//	GetMap1(map);
	GetMap2(map);
}
