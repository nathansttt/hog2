#include "Common.h"
#include "Driver.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
#include "NBS.h"
#include "SVGUtil.h"
#include "Plot2D.h"
#include "MapGenerators.h"

enum mode {
	kFindPath = 1,
	kDrawObstacles = 2
};

enum drawing {
	kDrawWall,
	kDrawGround,
	kDrawSwamp
};

int whichAlgorithm = 0;
int numAlgorithms = 3;
bool mapChanged = true;

MapEnvironment *me = 0;
TemplateAStar<xyLoc, tDirection, MapEnvironment> AStarForward, AStarBackward;
//fMM<xyLoc, tDirection, MapEnvironment> fmm;
NBS<xyLoc, tDirection, MapEnvironment> nbs;
std::vector<xyLoc> path;

xyLoc start, goal;

mode m = kFindPath;
drawing d = kDrawGround;

Plotting::Plot2D plot;
Plotting::Line forwardLine("forward");
Plotting::Line backwardLine("backward");
Plotting::Line totalLine("total");
Plotting::Line nbsline("NBS");
Plotting::Line selection("select");

bool recording = false;
bool running = false;
int stepsPerFrame = 1;
bool showPerc = false;
float frac = 0;
bool showNBS = false;

void LoadMap(Map *m, int which);

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
	InstallKeyboardHandler(MyDisplayHandler, "Draw", "Draw Obstacles", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Path", "Find Path", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Draw Walls", "Draw Wall Obstacles", kAnyModifier, '1');
	InstallKeyboardHandler(MyDisplayHandler, "Draw Ground", "Draw Open Ground", kAnyModifier, '2');
	InstallKeyboardHandler(MyDisplayHandler, "Draw Swamp", "Draw Slow Ground", kAnyModifier, '3');
	InstallKeyboardHandler(MyDisplayHandler, "Draw forward", "Draw only forward line", kAnyModifier, 'f');
	InstallKeyboardHandler(MyDisplayHandler, "Draw backward", "Draw only backward line", kAnyModifier, 'b');
	InstallKeyboardHandler(MyDisplayHandler, "Draw all", "Draw everything but NBS", kAnyModifier, 't');
	
	InstallKeyboardHandler(MyDisplayHandler, "Toggle NBS", "Toggle drawing NBS", kAnyModifier, 'n');

	InstallKeyboardHandler(MyDisplayHandler, "Minima", "Load map with local minima", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Asymmetry", "Load asymmetric map", kAnyModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "Deafult", "Load default map", kAnyModifier, 'l');

	InstallKeyboardHandler(MyDisplayHandler, "Faster", "Double steps per frame", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Slower", "Half steps per frame", kAnyModifier, '[');
	InstallKeyboardHandler(MyDisplayHandler, "Rotate", "Show different algorithm", kAnyModifier, '=');


	InstallKeyboardHandler(MyDisplayHandler, "Record", "Toggle SVG recording", kAnyModifier, 'r');

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
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		
		Map *map = new Map(148,139);
		LoadMap(map, 0);

		submitTextToBuffer("Drag to set start/goal");
		map->SetTileSet(kWinter);
		me = new MapEnvironment(map);
		me->SetDiagonalCost(1.5);
		start.x = start.y = 0xFFFF;
		printf("Done creating window\n");
		plot.AddLine(&forwardLine);
		plot.AddLine(&backwardLine);
		plot.AddLine(&totalLine);
		plot.AddLine(&nbsline);
		plot.AddLine(&selection);
		
		forwardLine.SetWidth(10.0);
		backwardLine.SetWidth(10.0);
		totalLine.SetWidth(10.0);
		nbsline.SetWidth(10.0);
		selection.SetWidth(10.0);
		
//		selection.SetHidden(true);
		nbsline.SetHidden(true);
		//SetNumPorts(windowID, 2);
		ReinitViewports(windowID, Graphics::rect{-1.f, -1.f, 1.f, 0.5f}, kScaleToSquare);
		AddViewport(windowID, Graphics::rect{-1.f, 0.5f, 1.f, 1.f}, kScaleToFill);
	}
}

int frameCnt = 0;

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	Graphics::Display &display = getCurrentContext()->display;
	
	if (viewport == 1)
	{
		plot.Draw(display);
		return;
	}

	if (mapChanged)
	{
		display.StartBackground();
		me->Draw(display);
		display.EndBackground();
		mapChanged = false;
	}
		
	if (!(start.x == 0xFFFF || running || showPerc))
	{
		//printf("Drawing start and goal\n");
		//me->Draw(display, start);
		me->SetColor(Colors::cb1);
		me->Draw(display, start);
		
		me->SetColor(Colors::cb2);
		me->Draw(display, goal);
		
		me->SetColor(Colors::cb5);
		me->DrawArrow(display, start, goal, 2.0);
	}
	
	if (showPerc)
	{
		float C = me->GetPathLength(path);
		if (!forwardLine.IsHidden())
		{
			for (int x = 0; x < AStarForward.GetNumItems(); x++)
			{
				auto i = AStarForward.GetItem(x);
				if (i.where != kClosedList)
					continue;
				if (i.g < C*frac)
				{
				}
				else {
					me->SetColor(Colors::lightgray);
					me->Draw(display, i.data);
				}
			}
		}
		if (!backwardLine.IsHidden())
		{
			for (int x = 0; x < AStarBackward.GetNumItems(); x++)
			{
				auto i = AStarBackward.GetItem(x);
				if (i.where != kClosedList)
					continue;
				if (i.g < C*(1-frac))
				{
				}
				else {
					me->SetColor(Colors::lightgray);
					me->Draw(display, i.data);
				}
			}
		}

		
		if (!forwardLine.IsHidden())
		{
			for (int x = 0; x < AStarForward.GetNumItems(); x++)
			{
				auto i = AStarForward.GetItem(x);
				if (i.where != kClosedList)
					continue;
				if (i.g < C*frac)
				{
					me->SetColor(Colors::red);
					me->Draw(display, i.data);
				}
				else {
//					me->SetColor(Colors::lightgray);
//					me->Draw(display, i.data);
				}
			}
		}
		if (!backwardLine.IsHidden())
		{
			for (int x = 0; x < AStarBackward.GetNumItems(); x++)
			{
				auto i = AStarBackward.GetItem(x);
				if (i.where != kClosedList)
					continue;
				if (i.g < C*(1-frac))
				{
					me->SetColor(Colors::lightblue);
					me->Draw(display, i.data);
				}
				else {
//					me->SetColor(Colors::lightgray);
//					me->Draw(display, i.data);
				}
			}
		}
		
	}
	else if (running)
	{
		if (path.size() == 0)
		{
			nbs.Draw(display);
		}
		else if (whichAlgorithm == 0)
		{
			nbs.Draw(display);
		}
		else if (whichAlgorithm == 1)
		{
			AStarForward.Draw(display);
		}
		else if (whichAlgorithm == 2)
		{
			AStarBackward.Draw(display);
		}

		for (int x = 0; x < stepsPerFrame; x++)
			if (path.size() == 0)
			{
				nbs.DoSingleSearchStep(path);
				if (path.size() != 0)
				{
					char tmp[255];
					sprintf(tmp, "A*f %llu, A*b %llu, A*avg %llu, NBS %llu\n", AStarForward.GetNodesExpanded(), AStarBackward.GetNodesExpanded(),
							(AStarForward.GetNodesExpanded()+AStarBackward.GetNodesExpanded())/2, nbs.GetNodesExpanded());
					submitTextToBuffer(tmp);
					nbsline.Clear();
					nbsline.SetColor(Colors::green);
					nbsline.SetHidden(false);
					nbsline.AddPoint(0, nbs.GetNodesExpanded());
					nbsline.AddPoint(plot.GetMaxX(), nbs.GetNodesExpanded());
				}
			}

		if (path.size() != 0) {
			me->SetColor(0, 0, 1);
			for (int x = 1; x < path.size(); x++)
			{
				me->DrawLine(display, path[x-1], path[x], 3);
			}
		}
	}
	if (recording )//&& viewport == GetNumPorts(windowID)-1)
	{
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/NBS-%d%d%d%d.svg",
				(frameCnt/1000)%10, (frameCnt/100)%10, (frameCnt/10)%10, frameCnt%10);
		
		MakeSVG(display, fname, 800, 1200);
		
		printf("Saved %s\n", fname);
		frameCnt++;
//		if (path.size() == 0)
//		{
//			MyDisplayHandler(windowID, kNoModifier, 'o');
//		}
//		else {
//			recording = false;
//		}
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
		case 'n':
			showNBS = !showNBS;
			showPerc = false;
			if (showNBS)
			{
				forwardLine.SetHidden(false);
				backwardLine.SetHidden(false);
				totalLine.SetHidden(false);
				nbsline.SetHidden(false);
			}
			else {
				nbsline.SetHidden(true);
			}
			break;
		case 't':
			forwardLine.SetHidden(false);
			backwardLine.SetHidden(false);
			totalLine.SetHidden(false);
			nbsline.SetHidden(true);
			break;
		case 'f':
			forwardLine.SetHidden(false);
			backwardLine.SetHidden(true);
			totalLine.SetHidden(true);
			nbsline.SetHidden(true);
			break;
		case 'b':
			forwardLine.SetHidden(true);
			backwardLine.SetHidden(false);
			totalLine.SetHidden(true);
			nbsline.SetHidden(true);
			break;
		case 'm':
			LoadMap(me->GetMap(), 1);
			//			if (me->GetMap()->GetMapWidth() == 100)
//			{
//				me->GetMap()->Load("/Users/nathanst/hog2/maps/local/bidir4.map");
//			}
//			else {
//				me->GetMap()->Scale(100, 100);
//				MakeMaze(me->GetMap(), 2);
//			}
			break;
		case 'o':
			LoadMap(me->GetMap(), 2);
			break;
		case 'l':
			LoadMap(me->GetMap(), 0);
//			me->GetMap()->Load("/Users/nathanst/hog2/maps/local/bidir5.map");

			//			nbs.DoSingleSearchStep(path);
			break;
		case 'r':
			recording = !recording;
			break;
		case 'd':
			start = goal = xyLoc();
			m = kDrawObstacles;
			running = false;
			showPerc = false;
			forwardLine.Clear();
			backwardLine.Clear();
			totalLine.Clear();
			nbsline.Clear();
			submitTextToBuffer("Mode: Draw Obstacles");
			break;
		case 'p':
			start = goal = xyLoc();
			m = kFindPath;
			running = false;
			submitTextToBuffer("Click and drag to set path");
			break;
		case '1': if (m != kDrawObstacles) break; d = kDrawWall; submitTextToBuffer("Drawing walls"); break;
		case '2': if (m != kDrawObstacles) break;d = kDrawGround; submitTextToBuffer("Drawing ground"); break;
		case '3': if (m != kDrawObstacles) break;d = kDrawSwamp; submitTextToBuffer("Drawing swamp"); break;
		case '=': whichAlgorithm = (whichAlgorithm+1)%numAlgorithms; break;
		case '[':
			stepsPerFrame /= 2;
			break;
		case ']':
			stepsPerFrame *= 2;
			if (stepsPerFrame == 0)
				stepsPerFrame = 1;
			break;
			
	}
	
}

void GetLine(Plotting::Line *l, TemplateAStar<xyLoc, tDirection, MapEnvironment> &astar, float C, std::vector<std::pair<int, int>> &items, bool reverse = false)
{
	l->Clear();
//	printf("===BEGIN===\n");

	// get all g-costs
	std::unordered_map<int, int> distribution;
	for (int x = 0; x < astar.GetNumItems(); x++)
	{
		auto i = astar.GetItem(x);
		if (i.where != kClosedList)
			continue;
//		printf("%1.1f ", i.g);
		int f = (int)((i.g+i.h)*10.0);
		if (f <= (int)(C*10))
		{
//			printf("Adding %f+%f=%f < %f\n", i.g, i.h, i.g+i.h, C);
			distribution[(int)(i.g*10.0)]++;
		}
	}
//	printf("\n===END===\n");
	if (distribution.empty())
	{
		printf("No necessary states\n");
		return;
	}

	// extract g-cost/count pairs
	for (auto i = distribution.begin(); i != distribution.end(); i++)
		items.push_back({i->first, i->second});
	// sort by g-cost
	auto func = [](const std::pair<int, int> &x,
				   const std::pair<int, int> &y)
	{
		return x.first < y.first;
	};
	std::sort(items.begin(), items.end(), func);
	// turn into cumulative distribution
	int sum = 0;
	for (int x = 0; x < items.size(); x++)
	{
		sum += items[x].second;
		items[x].second = sum;
		if (reverse)
			l->AddPoint(10*C-items[x].first, sum);
		else
			l->AddPoint(items[x].first, sum);
//		printf("(%d, %d) ", items[x].first, sum);
	}
	items.push_back({C*10, sum});
	if (reverse)
		l->AddPoint(0, sum);
	else
		l->AddPoint(C*10, sum);
//	printf("\n\n");
}

void DoLines()
{
	std::vector<std::pair<int, int>> itemsForward;
	std::vector<std::pair<int, int>> itemsBackward;
	
	float C = (float)(me->GetPathLength(path));
	GetLine(&forwardLine, AStarForward, C, itemsForward);
	GetLine(&backwardLine, AStarBackward, C, itemsBackward, true);
	totalLine.Clear();
	for (int x = 0; x < itemsForward.size(); x++)
	{
		int forwardG = itemsForward[x].first;
		int backWork = 0;
		for (int y = 0; y < itemsBackward.size(); y++)
		{
			if (itemsBackward[y].first + forwardG + 10 <= C*10) // add epsilon
			{
				backWork = itemsBackward[y].second;
			}
			else
				break;
		}
		totalLine.AddPoint(forwardG, backWork+itemsForward[x].second);
	}
	forwardLine.SetColor(Colors::red);
	backwardLine.SetColor(Colors::lightblue);
	totalLine.SetColor(Colors::purple);
	plot.ResetAxis();

}

void SetStartGoalHandler(uint16_t x, uint16_t y, tMouseEventType mType)
{
	static bool currentlyDrawing = false;
	assert(x != 0xFFFF);
	assert(y != 0xFFFF);
	switch (mType)
	{
		case kMouseDown:
		{
			currentlyDrawing = true;
			running = false;
			showPerc = false;
			nbsline.SetHidden(true);
			nbsline.Clear();
			if ((me->GetMap()->GetTerrainType(x, y)&kGround) == kGround)
			{
				start.x = x;
				start.y = y;
				goal = start;
				//printf("Hit (%d, %d)\n", x, y);
			}
		}
		case kMouseDrag:
		{
			if ((me->GetMap()->GetTerrainType(x, y)&kGround) == kGround && currentlyDrawing)
			{
				//printf("drag (%d, %d)\n", x, y);
				goal.x = x;
				goal.y = y;
			}
			break;
		}
		case kMouseUp:
		{
			if (!currentlyDrawing)
				return;
			currentlyDrawing = false;
			if ((me->GetMap()->GetTerrainType(x, y)&kGround) == kGround)
			{
				goal.x = x;
				goal.y = y;
				AStarForward.GetPath(me, start, goal, path);
				AStarBackward.GetPath(me, goal, start, path);
				DoLines();
				if (showNBS)
				{
					nbs.InitializeSearch(me, start, goal, me, me, path);
					running = true;
					whichAlgorithm = 0;
				}
				else {
					showPerc = true;
					frac = 1.0;
				}
			}
		}
	}
}


void DrawHandler(uint16_t x, uint16_t y, tMouseEventType mType)
{
	static bool currentlyDrawing = false;
	static int lastx = 0, lasty = 0;
	if (mType == kMouseUp)
	{
		currentlyDrawing = false;
		return;
	}
//	if (me->GetMap()->GetTerrainType(x, y) == kOutOfBounds)
//		return;
	
	mapChanged = true;
	if (mType == kMouseDown)
	{
		currentlyDrawing = true;
		switch (d)
		{
			case kDrawGround: me->GetMap()->SetTerrainType(x, y, kGround); break;
			case kDrawWall: me->GetMap()->SetTerrainType(x, y, kTrees); break;
			case kDrawSwamp: me->GetMap()->SetTerrainType(x, y, kSwamp); break;
		}
	}
	else if (mType == kMouseDrag && currentlyDrawing)
	{
		switch (d)
		{
			case kDrawGround:
				for (int xdelta = -2; xdelta <= 2; xdelta++)
					for (int ydelta = -2; ydelta <= 2; ydelta++)
						me->GetMap()->SetTerrainType(x+xdelta, y+ydelta, kGround);
				break;
			case kDrawWall:
				for (int xdelta = -2; xdelta <= 2; xdelta++)
					for (int ydelta = -2; ydelta <= 2; ydelta++)
						me->GetMap()->SetTerrainType(x+xdelta, y+ydelta, kOutOfBounds);
				break;
			case kDrawSwamp:
				for (int xdelta = -2; xdelta <= 2; xdelta++)
					for (int ydelta = -2; ydelta <= 2; ydelta++)
						if (me->GetMap()->GetTerrainType(x+xdelta, y+ydelta) == kGround)
							me->GetMap()->SetTerrainType(x+xdelta, y+ydelta, kSwamp);
				break;
		}
	}
	lastx = x; lasty = y;
}

void HandleGraphMouse(point3d loc, tMouseEventType mType)
{
	loc.x *= 1.11111111; // 5% margins on each side
	if (loc.x > 1) loc.x = 1;
	if (loc.x < -1) loc.x = -1;
	switch (mType)
	{
		case kMouseDown:
		{
			showPerc = true;
			frac = (loc.x+1.0)/2.0;
//			printf("Showing %f%% (%f)\n", frac, loc.x);
			float C = me->GetPathLength(path);
			selection.SetColor(Colors::gray);
			selection.AddPoint(frac*C*10.0, plot.GetMaxY());
			selection.AddPoint(frac*C*10.0, 0);
		}
			break;
		case kMouseDrag:
		{
			if (showPerc)
			{
				frac = (loc.x+1.0)/2.0;
	//			printf("Showing %f%% (%f)\n", frac, loc.x);
				float C = me->GetPathLength(path);
				selection.Clear();
				selection.SetColor(Colors::gray);
				selection.AddPoint(frac*C*10.0, plot.GetMaxY());
				selection.AddPoint(frac*C*10.0, 0);
			}
		}
			break;
		case kMouseUp:
		{
			selection.Clear();
			if (running)
				showPerc = false;
		}
			break;
	}
}

bool MyClickHandler(unsigned long , int viewport, int windowX, int windowY, point3d loc, tButtonType button, tMouseEventType mType)
{
	if (button != kLeftButton)
		return false;
	if (viewport == 1)
	{
		HandleGraphMouse(loc, mType);
		return true;
	}
	int x, y;
	me->GetMap()->GetPointFromCoordinate(loc, x, y);

	if (x == -1 || y == -1)
		return false;
	switch (m)
	{
		case kFindPath: SetStartGoalHandler(x, y, mType); break;
		case kDrawObstacles: DrawHandler(x, y, mType); break;
	}
	return true;
}
	
	
void BGMap2(Map *m)
{
	m->Scale(148, 139);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@....@@@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@@....@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@@@@.@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@.@@@@..@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@...@@@..@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@....@..@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@.....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@..@.......................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@...........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@........@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....................@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...................@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.................@@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@.@@@@@@@...............@@@@@@@@..........@@@@@..@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@.......@@@@@.............@@@@@@@@..........@@@@@...@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@.........@@@@@...........@@@@@@@@........@@@@@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@..........@@@@@@@@.........@@@@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@..........@@@@@@@.........@@@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@.........@@@@@@@.........@@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@..........@@@@@@.........@@@@@@.....@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@.............@..@.........@@@@@@.....@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@..........................@@@@@@.....@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@.........................@@@@@@......@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@..........................@@@@@@.....@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@..........................@@@@@@.....@@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@..........................@@@@@@.....@@@@...................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........................@@.................@@@@@@.....@@@@...@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@.........@@@@...............@@@@@@@....@@@@...@@@@@......@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@.......@@@@@@.............@@@@@......@@@@...@@@@@@.@...@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@.......@@@@@@@............@@@@@......@@@@...@@@@@@@@@..@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@.......@@@@@@@@............@@@@......@@@@....@@@@@@@@..@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@.......@@@@@@@@@.............@@@.......@@.....@@@@@@@..@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@.......@@@@@@@@................@@................@@.@..@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@.......@@@@@@@@..........@.............................@@@@..................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@........@@@@@@@..........@@@...........................@@@@..@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@@@..@@@@.........@@@@@@..........@@@@@.........................@@@@..@@@@......@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@@@@..........@@@@@..........@@@@@@........................@@@@..@@@@@@....@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@@@............@@@..........@@@@@@@.......................@@@@..@@@@@@@...@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@.........................@@@@@@........................@@@@..@@@@@@@...@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@@......................@@@@@@....................@@...@@@....@@@@@...@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@@.....................@@@@@@....................@@@@...@......@@@...@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@.....@.....@@@@@@...................@@@@@@....................@@@@@@..............@@@@..................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@.....@@@.....@@@@@..................@@@@@@@........@.@@.......@@@..@@@............@@@@..@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@.....@@@@.................@@@@@..........@@@@@@.....@@@....@@@..........@@@@@@@@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@.....@..................@@@@@...........@@@@@@....@@@......@@@........@@@@@@@@@@@@.......@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@........@@@...........@@@@@...........@@@@@@....@@@........@@@......@@@@.@@@@@@@@@.....@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@........@@@@.........@@@@@@............@@@@....@@@..........@@@....@@@@..@.@@@@@@@....@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@..@......@@@@@.......@@@@@@@.............@@....@@@............@@@...@@@.....@@@@@@....@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@..@@@.......@@@@.....@@@@@@@@@.................@@@..............@@....@.......@@@@....@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@.......@@@@...@@@@@@@@...................@@...............@@@............@@....@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@.......@@@@.@@@@@@@@...................@@.........@@......@@@................@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@.......@@@@@@@@@@@@...................@@@.........@@@.....@@@...............@@@@.............@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@.......@@@@@@@@@@@@......@@...........@@@@..........@@......@...............@@@@.............@@@@@@....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@................@@@@@@@@@@@@......@@@@.........@@@@@@.........@@.....................@@@@.............@@@@@@@...@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@@@@@@@......@@@@@@........@@@@@@@........@@@.....@@............@@@@.............@@@@@@@@...@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@@@@@.....@@@@@@@@@......@@@@@@@@@@....@@@@.....@@...........@@@@.............@@@@@@@@@....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@@@@@@@....@@@@@@@@@@@......@@@@@@@@@@@@@@..@@.................@@@.............@@@@@@@@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@@@@@@@....@@@@@@@@@@@@@@@S.....@@@@@@@@@@.....@@..................@.............@@@@@@@@.........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@@@@@@@.....@@@@@@@@@@@@@@@SS.....@@@@@@@@.......@...............................@@@@@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@@@@@......@@@@@@@@@@@@@@SSS.......@@@@@.......................................@@@@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@@.@@........@@@@@@@@@@@@SSSS........@@@.......................................@@@@@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@@SSSSS@@.......@.......................................@@@@@@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@@SSSSS@@@@@............................................@@@@@@@@...................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@SSSSSS@@@@@@@..........................................@@@@@@@@.....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@SSSSS@@@@@@@@@........................................@@@@@@@@..........@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@@SSSS@@@@@@@@@@@......................................@@@@@@@@..........@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@@@SSSS@@@@@@@@@@@@S...................................@@@@@@@@..........@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@..SSS@@@@@@@@@@@@@@@SS................................@@@@@@@@............@@@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@...SS@@@@@@@@@@@@@@@SSS...............................@@@@@@@@..............@@...........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@@@@.....S@@@@@@@@@@@@SSSS................................@@@@@@@...........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@...........@@@@@@@@@@SSSSS@@...............................@@.@@@.....@....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@...........@@@@@@@@@SSSSS@@@@...............................@........@@@..................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@.............@@@@@@@SSSSS@@@@@@......................................@@@@@................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@SSSSS@@@@@@@@......................................@@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@SSSS@@@@@@@@@@......................................@@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@@SSS@@@@@@@@@@@@......................................@@@@@..........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@@@@SSSS@@@@@@@@@@@@@@......................................@@@@@........@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@......@@@@@@@@@@@@@@................................@......@@@@@......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@.....@@@@@@@@@@@@@@.............@...................@@......@@@@@....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@.@@@......@@@@@@@@@@@@.............@@@...................@@......@@@@@..@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.....@@@@@@@...........@@@@@@@@@@.............@@@@@...................@@......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@@@@...........@@@@@@@@@.............@@@@@@...........@@.......@.......@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@.............@@@@@@@@.............@@@@@@@..........@@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@@.............@@@@@@............@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@@@............@@@@@@..............@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............@@@@@@@@@@...........@@@@@@@............................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.............@@@@@@@@.....@.......@@@@@@@@...........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@......@@.....@@@@@@@@...........@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.........@@@@@@@@.......@@@...@@@@@@@@.............@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.......@@@@@@...........@@@@@@@@@@@@...............@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@...........@@@@@@@@@@@.......@@........@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@............@@@@@@@@@@.......@@@@........@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@@@........@@@@...................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@............@@@@.................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@..............@@@@...............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...........@@@@@@@@@................@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............@@@@@@@@...................@@@@......@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........@@@@@@@@.....................@@@@.....@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@@@.......................@@@@...@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......@@@@@@@@.........................@@@@.@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....@@@@@@@@@..........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...@@@@@.@@...........@@@..............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.@@@@@..............@@@@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@...............@@@@............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@.................@@.............@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@................................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..............................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@............................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@..........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@......................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
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

void MapAsymmetry(Map *m)
{
	m->Scale(25, 25);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@@@@@.......................@@..@@@@@@@@@@@@@@@@@@@..@@@@@@@@@@@@@@@@@@@@@@@.@@@@@@@@@@............@@.@@@@@@@@@@............@@.@@@@@@@@@@............@@.@@@@@@@@@@............@@.@@@@@@@@@@............@@.@@@@@@@@@@.........@..@@.@@@@@@@@@@.........@.@@..@@@@@@@@@@.........@.@..@@@@@@@@@@@.........@...@@@@@@@@@@@@..........@@@@@@@@@@@@@@@..........@@@@@@@@@@@@@@@..........@@@@@@@@@@@@@@@..........@@@@@@@@@@@@@@@@@@.@@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@@@@@@@@@@@...@..@@@@@@@@@@@@@@@@@@..@...@@@@@@@@@@@@@@@@@@..@@@@@@@@@@@@@@@@@@@@@...@@@@@@@@@@@@@@@@@@@@@@..@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
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

void LocalMinima(Map *m)
{
	m->Scale(25, 25);
	const char map[] = "@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@........@@@@@@@@@@@@@@@@@.@@@@@@S@@@@@@@@@@@@@@@@@.@@@@@@S@@@@@@@@@@@@@@@@@.@@@@@@S@@@@@@@@@@@@@@@@@.@@@@@@S@@@@@@@@@@@@@@@@@@@@@@@@S@@@@.....................@@@@.....................@@@@.....................@@@@.....................@@@@.....................@@@@.....................@@@@.....................@@@@.....................@@@@S@@@@@@@@@@@@@@@@@@@S@@@@.....................@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
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


void LoadMap(Map *m, int which)
{
	recording = false;
	running = false;
	showPerc = false;
	frac = 0;
	showNBS = false;
	mapChanged = true;

	if (which == 1)
	{
		MapAsymmetry(m);
		return;
	}
	if (which == 2)
	{
		LocalMinima(m);
		return;
	}
	BGMap2(m);
}

