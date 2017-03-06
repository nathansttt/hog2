#include "Common.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "TOH.h"
#include "IDAStar.h"
#include "Timer.h"
#include "PermutationPDB.h"
#include "TemplateAStar.h"

const int numDisks = 16;

TOH<numDisks> toh;
TOHState<numDisks> s, g;

IDAStar<TOHState<numDisks>, TOHMove> ida;
std::vector<TOHMove> solution;

TemplateAStar<TOHState<numDisks>, TOHMove, TOH<numDisks> >astar;


// PDB info
TOHState<numDisks> goal;
TOH<numDisks-2> absToh1;
TOH<numDisks-3> absToh2;
TOH<numDisks-4> absToh3;
TOHState<numDisks-2> absTohState1;
TOHState<numDisks-3> absTohState2;
TOHState<numDisks-4> absTohState3;
TOHPDB<numDisks-2, numDisks> pdb1(&absToh1, goal);
TOHPDB<numDisks-2, numDisks> pdb1a(&absToh1, goal);
TOHPDB<numDisks-2, numDisks> pdb1b(&absToh1, goal);
TOHPDB<numDisks-3, numDisks> pdb2(&absToh2, goal);
TOHPDB<numDisks-4, numDisks> pdb3(&absToh3, goal);

Heuristic<TOHState<numDisks>> h;

bool recording = false;

int main(int argc, char* argv[])
{
	InstallHandlers();
	RunHOGGUI(argc, argv, 640, 640);
}

/**
 * Allows you to install any keyboard handlers needed for program interaction.
 */
void InstallHandlers()
{
	InstallKeyboardHandler(MyDisplayHandler, "PDB", "Build a PDB", kAnyModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "PDB", "Value-Range Compress PDB", kAnyModifier, 'v');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Reset View", "Reset camera to initial view", kAnyModifier, '|');
	InstallKeyboardHandler(MyDisplayHandler, "Reset state", "Choose a new random initial state", kAnyModifier, 's');

	InstallCommandLineHandler(MyCLHandler, "-run", "-run", "Runs pre-set experiments.");
	
	InstallWindowHandler(MyWindowHandler);

	InstallMouseClickHandler(MyClickHandler);
}

void SolveProblem(bool reset = true)
{
	Timer t;
	if (reset)
	{
		s.Reset();
		srandom(1234);
		for (int x = 0; x < 200; x++)
		{
			toh.GetActions(s, solution);
			toh.ApplyAction(s, solution[random()%solution.size()]);
		}
	}
	std::cout << s << "\n";
	//toh.GetStateFromHash(random()%toh.GetNumStates(s), s);
	//toh.GetStateFromHash(0, s);
	ida.SetHeuristic(&h);
	t.StartTimer();
	//ida.GetPath(&toh, s, g, solution);
	t.EndTimer();
	for (auto &m : solution)
	{
		std::cout << m << " ";
	}
	std::cout << "\n";
	printf("%1.2fs elapsed; %llu expanded; %llu generated [%lu]\n", t.GetElapsedTime(), ida.GetNodesExpanded(), ida.GetNodesTouched(), solution.size());

	astar.SetHeuristic(&h);
	t.StartTimer();
	astar.GetPath(&toh, s, g, solution);
	t.EndTimer();
	printf("%1.2fs elapsed; %llu expanded; %llu generated [%lu]\n", t.GetElapsedTime(), astar.GetNodesExpanded(), astar.GetNodesTouched(), solution.size());
}

void SpeedTest()
{
	int nodeLimit = 10000000;
	std::vector<TOHMove> moves;
	Timer t;
	s.Reset();
	srandom(1234);
	t.StartTimer();
	for (int x = 0; x < nodeLimit; x++)
	{
		toh.GetActions(s, moves);
		toh.ApplyAction(s, moves[random()%moves.size()]);
	}
	t.EndTimer();
	printf("%1.2fs elapsed; %1.2f apply action per second\n", t.GetElapsedTime(), nodeLimit/t.GetElapsedTime());

	std::vector<TOHState<numDisks>> succ;
	s.Reset();
	t.StartTimer();
	for (int x = 0; x < nodeLimit; x++)
	{
		toh.GetSuccessors(s, succ);
		s = succ[random()%succ.size()];
	}
	t.EndTimer();
	printf("%1.2fs elapsed; %1.2f get succ per second\n", t.GetElapsedTime(), nodeLimit/t.GetElapsedTime());
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
		InstallFrameHandler(MyFrameHandler, windowID, 0);
		SetNumPorts(windowID, 1);
		glClearColor(0.9, 0.9, 0.9, 1.0);

		h.heuristics.push_back(&toh);
		h.lookups.push_back({kLeafNode, 0, 0});
//		s.StandardStart();
//		SolveProblem(false);
		SpeedTest();
	}
}

double v = 1;
uint64_t counter = 0;
const int animationFrames = 30;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	cameraMoveTo(0, -1, -12.5, 0.05);
	cameraLookAt(0, 0, 0, 0.2);
	
	glEnable(GL_LIGHTING);
	toh.OpenGLDraw();
	if (solution.size() != 0)
	{
		toh.GetNextState(s, solution[0], g);
		toh.OpenGLDraw(s, g, float(counter)/animationFrames);
		counter = (counter+1)%animationFrames;
		if (counter == 0)
		{
			toh.ApplyAction(s, solution[0]);
			solution.erase(solution.begin());
			if (solution.size() == 0)
				recording = false;
		}
	}
	else {
		toh.OpenGLDraw(s);
	}
	
	if (recording && viewport == GetNumPorts(windowID)-1)
	{
		static int cnt = 0;
		char fname[255];
		sprintf(fname, "/Users/nathanst/Movies/tmp/TOH-%d%d%d%d", (cnt/1000)%10, (cnt/100)%10, (cnt/10)%10, cnt%10);
		SaveScreenshot(windowID, fname);
		printf("Saved %s\n", fname);
		cnt++;
	}
	return;
	
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	exit(0);
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case '|': resetCamera(); break;
		case 'r': recording = !recording; break;
		case 's':
		{
			toh.GetStateFromHash(random()%toh.GetNumStates(s), s);
			g.Reset();
			SolveProblem(false);
//			ida.GetPath(&toh, s, g, solution);
//			for (auto &m : solution)
//			{
//				std::cout << m << " ";
//			}
//			std::cout << "\n";
		}
			break;
		case 'v': // value range compression
		{
			goal.Reset();
			// 268435456 entries (256 MB)
			printf("Dist: Full PDB\n");
			pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
			pdb1.ValueRangeCompress(4, true);
			break;
			// 67108864 entries (64 MB) (pdb2) [72.171778]
			// 16777216 entries (16 MB) (pdb3)
			//pdb3.BuildPDB(goal, std::thread::hardware_concurrency());
	
			pdb1a = pdb1;
			pdb1b = pdb1;
			//pdb1a.DivCompress(16, true); // 16 MB
			printf("Dist: Div factor\n");
			//pdb1a.DivCompress(32, true); // 8 MB
			pdb1a.DivCompress(64, true); // 4 MB
			printf("Dist: Delta (over original)\n");
			pdb1.DeltaCompress(&pdb1a, goal, true);
			
			// delta pdb2 = 14.867143 / after VRC = 14.864208
			// delta pdb3 = 28.029138 / after VRC = 27.922551
//			pdb1.DeltaCompress(&pdb3, goal, true);
			printf("Dist: VRC\n");
			pdb1.ValueRangeCompress(2, true);

			printf("Dist: Straight 4x compression (≈2 bits)\n");
			pdb1b.DivCompress(4, true);
			printf("Dist: Straight 8x compression (≈1 bit\n");
			pdb1b.DivCompress(2, true);
			//pdb.DivCompress(4, true);
			//pdb.ModCompress(pdb.GetPDBSize()/4, true);
			h.lookups.resize(0);
			h.lookups.push_back({kMaxNode, 1, 2});
			h.lookups.push_back({kLeafNode, 0, 0});
			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.resize(0);
			h.heuristics.push_back(&toh);
			h.heuristics.push_back(&pdb1);
			SolveProblem();
			break;
		}
		case 'p': // div compression
		{
			goal.Reset();
			// 268435456 entries (256 MB)
			// Average: 87.038921 -> 86.762030 -> 86.483987
			pdb1.BuildPDB(goal, std::thread::hardware_concurrency());

//			pdb1.DivCompress(2, true); // 128 MB
//			pdb1.DivCompress(2, true); // 64 MB

			h.lookups.resize(0);
//			h.lookups.push_back({kMaxNode, 1, 2});
			h.lookups.push_back({kLeafNode, 0, 0});
//			h.lookups.push_back({kLeafNode, 1, 1});
			h.heuristics.resize(0);
//			h.heuristics.push_back(&toh);
			h.heuristics.push_back(&pdb1);

			s.StandardStart();
			SolveProblem(false);
			//recording = true;
			break;
		}
		default:
			break;
	}
}


bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

