//
//  GUICode.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/29/15.
//  Copyright (c) 2015 University of Denver. All rights reserved.
//

#include <vector>
#include "GUICode.h"
#include "Map2DEnvironment.h"
#include "MapOverlay.h"
#include "TemplateAStar.h"

Map *map = 0;
MapEnvironment *me = 0;
MapOverlay *mo;
xyLoc start, goal;

std::vector<int> counts;

bool mouseTracking = false;
void SetupMapOverlay();

TemplateAStar<xyLoc, tDirection, MapEnvironment> forward;
TemplateAStar<xyLoc, tDirection, MapEnvironment> backward;

enum bibfs {
	NN = 1,
	NF = 2,
	NR = 3,
	FN = 4,
	FF = 5,
	FR = 6,
	RN = 7,
	RF = 8,
	RR = 9
};

const char *bibfs_desc[10] = {
	"", "NN", "NF", "NR", "FN", "FF", "FR", "RN", "RF", "RR"
};

void InstallHandlers()
{
	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler);
}

void MyWindowHandler(unsigned long windowID, tWindowEventType eType)
{
	if (eType == kWindowDestroyed)
	{
		printf("Window %ld destroyed\n", windowID);
		RemoveFrameHandler(MyFrameHandler, windowID, 0);
		mouseTracking = false;
		delete map;
		delete me;
		map = 0;
		me = 0;
	}
	else if (eType == kWindowCreated)
	{
		printf("Window %ld created\n", windowID);
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
		
		delete map;
		delete me;
		//map = new Map("/Users/nathanst/hog2/maps/dao/lak308d.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/ht_chantry.map");
		//map = new Map("/Users/nathanst/hog2/maps/random/random512-35-6.map");
		//map = new Map("/Users/nathanst/hog2/maps/da2/lt_backalley_g.map");
		//map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0011SR.map");
		map = new Map("/Users/nathanst/hog2/maps/bgmaps/AR0012SR.map");
		//map = new Map("/Users/nathanst/hog2/maps/rooms/8room_000.map");
		//map = new Map("/Users/nathanst/hog2/maps/mazes/maze512-16-0.map");
		//map = new Map("/Users/nathanst/hog2/maps/dao/orz107d.map");
		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
	}
	
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	map->OpenGLDraw();
	if (mo)
	{
		mo->OpenGLDraw();
		for (int x = 1; x < 10; x++)
		{
			recColor r = mo->GetValueColor(x);
			glColor3f(r.r, r.g, r.b);
			DrawBox(-1+0.2*x-1.0/40.0, -1-1.0/40.0, 0, 1.0/40.0);
			glColor3f(1.0, 1.0, 1.0);
			DrawText(-1+0.2*x+1.0/40.0, -1-1.0/40.0, -0.01, 1.0/10.0, bibfs_desc[x]);
			char num[16];
			sprintf(num, "%d", counts[x]);
			DrawText(-1+0.2*x+1.0/40.0, -1+1.0/40.0, -0.01, 1.0/10.0, num);
		}
	}
	if (mouseTracking)
	{
		me->SetColor(1.0, 0, 0);
		glLineWidth(3.0);
		me->GLDrawLine(start, goal);
		glLineWidth(1.0);
		me->SetColor(1.0, 1.0, 1.0);
		me->GLDrawLine(start, goal);
	}
	me->SetColor(1.0, 1.0, 1.0);
	glLineWidth(1.0);
	me->GLLabelState(start, "start", map->GetMapHeight()/8.0);
	me->GLLabelState(goal, "goal", map->GetMapHeight()/8.0);
	glLineWidth(1.0);

}

bool MyClickHandler(unsigned long windowID, int, int, point3d loc, tButtonType button, tMouseEventType mType)
{
	//	return false;
	static point3d startLoc;
	if (mType == kMouseDown)
	{
		switch (button)
		{
			case kRightButton: printf("Right button\n"); break;
			case kLeftButton: printf("Left button\n"); break;
			case kMiddleButton: printf("Middle button\n"); break;
		}
	}
	if (button != kRightButton)
		return false;
	switch (mType)
	{
		case kMouseDown:
		{
			delete mo;
			mo = 0;
			
			int x, y;
			map->GetPointFromCoordinate(loc, x, y);
			start.x = x; start.y = y;
			goal = start;
			mouseTracking = true;
			return true;
		}
		case kMouseDrag:
		{
			int x, y;
			map->GetPointFromCoordinate(loc, x, y);
			goal.x = x; goal.y = y;
			mouseTracking = true;
			return true;
		}
		case kMouseUp:
			if (mouseTracking)
			{
				int x, y;
				map->GetPointFromCoordinate(loc, x, y);
				goal.x = x; goal.y = y;
				mouseTracking = false;
				SetupMapOverlay();
				return true;
			}
	}
	return false;
}

void SetupMapOverlay()
{
	std::cout << "Doing map overlay from " << start << " to " << goal << "\n";
	counts.resize(0);
	counts.resize(10);
	delete mo;
	mo = new MapOverlay(map);
	me->SetDiagonalCost(1.5);
	forward.SetStopAfterGoal(false);
	backward.SetStopAfterGoal(false);
	std::vector<xyLoc> path;
	forward.GetPath(me, start, goal, path);
	backward.GetPath(me, goal, start, path);
	
	double optimal;
	forward.GetClosedListGCost(goal, optimal);
	for (int x = 0; x < map->GetMapWidth(); x++)
	{
		for (int y = 0; y < map->GetMapHeight(); y++)
		{
			if (map->GetTerrainType(x, y) == kGround)
			{
				xyLoc l(x, y);
				double startDist, goalDist;
				forward.GetClosedListGCost(l, startDist);
				backward.GetClosedListGCost(l, goalDist);
				if (startDist <= optimal/2 && goalDist <= optimal/2)
				{
					counts[NN]++;
					mo->SetOverlayValue(x, y, NN);
				}
				else if (startDist <= optimal/2 && goalDist <= optimal)
				{
					counts[NF]++;
					mo->SetOverlayValue(x, y, NF);
				}
				else if (startDist <= optimal/2)
				{
					counts[NR]++;
					mo->SetOverlayValue(x, y, NR);
				}
				else if (startDist <= optimal && goalDist <= optimal/2)
				{
					counts[FN]++;
					mo->SetOverlayValue(x, y, FN);
				}
				else if (startDist <= optimal && goalDist <= optimal)
				{
					counts[FF]++;
					mo->SetOverlayValue(x, y, FF);
				}
				else if (startDist <= optimal)
				{
					counts[FR]++;
					mo->SetOverlayValue(x, y, FR);
				}
				else if (goalDist <= optimal/2)
				{
					counts[RN]++;
					mo->SetOverlayValue(x, y, RN);
				}
				else if (goalDist <= optimal)
				{
					counts[RF]++;
					mo->SetOverlayValue(x, y, RF);
				}
				else {
					counts[RR]++;
					mo->SetOverlayValue(x, y, RR);
				}
			}
		}
	}
	mo->SetOverlayValue(start.x, start.y, 0);
	mo->SetOverlayValue(start.x, start.y, 10);
	mo->SetOverlayValue(goal.x, goal.y, 10);
	for (int x = 0; x < counts.size(); x++)
	{
		switch (x)
		{
			case 1: printf("NN: %d\n", counts[x]); break;
			case 2: printf("NF: %d\n", counts[x]); break;
			case 3: printf("NR: %d\n", counts[x]); break;
			case 4: printf("FN: %d\n", counts[x]); break;
			case 5: printf("FF: %d\n", counts[x]); break;
			case 6: printf("FR: %d\n", counts[x]); break;
			case 7: printf("RN: %d\n", counts[x]); break;
			case 8: printf("RF: %d\n", counts[x]); break;
			case 9: printf("RR: %d\n", counts[x]); break;
			default: break;
		}
	}
}

