/**
 * Delta: Driver.cpp
 *
 * This code performs experiments for a yet-to-be submitted paper. More details
 * will be provided here when the paper is accepted/published
 */

#include <cstring>
#include "Common.h"
#include "PermutationPDB.h"
#include "MR1PermutationPDB.h"
#include "Driver.h"
#include "UnitSimulation.h"
#include "EpisodicSimulation.h"
#include "Plot2D.h"
#include "RandomUnit.h"
#include "MNPuzzle.h"
#include "IDAStar.h"
#include "ParallelIDAStar.h"
#include "Timer.h"
#include "TopSpin.h"
#include "TOH.h"

void MDDeltaPlusMinTest();
void MinDeltaPlusMinTest();
void MinDeltaPlusMinTopSpinTest();
void MinDeltaPlusMinTOHTest();
void TSVRC();

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
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'w');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'a');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 's');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'd');
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'v');

	
	InstallKeyboardHandler(MyDisplayHandler, "Record", "Record a movie", kAnyModifier, 'r');
	InstallKeyboardHandler(MyDisplayHandler, "Toggle Abstraction", "Toggle display of the ith level of the abstraction", kAnyModifier, '0', '9');
	InstallKeyboardHandler(MyDisplayHandler, "Cycle Abs. Display", "Cycle which group abstraction is drawn", kAnyModifier, '\t');
	InstallKeyboardHandler(MyDisplayHandler, "Delta + MD", "Test what happens when we add delta over MD with min compression.", kNoModifier, 'm');
	InstallKeyboardHandler(MyDisplayHandler, "Delta + Min", "Test what happens when we add delta over min with min compression.", kNoModifier, 'p');
	InstallKeyboardHandler(MyDisplayHandler, "Delta + Min", "Test what happens when we add delta over min with min compression on Top Spin", kAnyModifier, 't');
	//InstallKeyboardHandler(MyDisplayHandler, "Delta + Min", "Test what happens when we add delta over min with min compression on TOH", kAnyModifier, 'd');

	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallCommandLineHandler(MyCLHandler, "-run", "-run", "Runs pre-set experiments.");
	InstallCommandLineHandler(MyCLHandler, "-test", "-test", "Basic test with MD heuristic");
	
	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler);
}

MNPuzzleState s(4,4), t(4, 4);
std::vector<slideDir> moves;
double v = 1;

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
		SetNumPorts(windowID, 1);
	}
}

#include "MapGenerators.h"
#include "Map2DEnvironment.h"
#include "TemplateAStar.h"
Map *m = 0;
xyLoc start = {0, 0}, goal = {0, 0};
tDirection nextMove = kStay;
double timer = 0;
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *)
{
	if (m == 0)
	{
		m = new Map(10, 10);
		MakeMaze(m);
	}
	MapEnvironment e(m);
	if (start == goal)
	{
		MakeMaze(m);
		start = {1, 1};
		goal = {9, 9};
		double cost = 0;
		double best;
		std::vector<xyLoc> path;
		TemplateAStar<xyLoc, tDirection, MapEnvironment> astar;
		astar.SetStopAfterGoal(false);
		astar.GetPath(&e, start, start, path);
		best = astar.GetClosedListGCost(start, best);
		for (uint16_t x = 0; x < 10; x++)
			for (uint16_t y = 0; y < 10; y++)
			{
				if (m->GetTerrainType(x, y) == kGround)
				{
					astar.GetClosedListGCost({x, y}, cost);
					if (cost > best)
					{
						goal = {x, y};
						best = cost;
					}
				}
			}
		std::cout << start << goal << "\n";
		
	}
	m->OpenGLDraw();
	e.SetColor(1.0, 0.0, 1.0);
	if (nextMove == kStay)
	{
		e.OpenGLDraw(start);
	}
	else {
		xyLoc next;
		e.GetNextState(start, nextMove, next);
		e.OpenGLDraw(start, next, timer);
		timer += 0.2;
		if (timer >= 1)
		{
			timer = 0;
			nextMove = kStay;
			start = next;
		}
	}
	//m->GetOpenGLCoord(start.x, start.y, x, y, z, r);
	//DrawSphere(x, y, z, r);
	e.SetColor(0.0, 1.0, 0.0);
	e.OpenGLDraw(goal);
	//	glColor3f(0.0, 1.0, 0.0);
//	DrawText(0, 0, 0, 1.0, "No visualization");
}

int MyCLHandler(char *argument[], int maxNumArgs)
{
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
		case 'r':
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			break;
		case '\t':
			if (mod != kShiftDown)
				SetActivePort(windowID, (GetActivePort(windowID)+1)%GetNumPorts(windowID));
			else
			{
				SetNumPorts(windowID, 1+(GetNumPorts(windowID)%MAXPORTS));
			}
			break;
		case 'm':
		{
			MDDeltaPlusMinTest();
			break;
		}
		case 'p':
		{
			MinDeltaPlusMinTest();
			break;
		}
			break;
		case 't':
		{
			MinDeltaPlusMinTopSpinTest();
			break;
		}
		case 'w': if (nextMove == kStay && m->CanStep(start.x, start.y, start.x, start.y-1)) nextMove = kN; break; //start.y--; break;
		case 's': if (nextMove == kStay && m->CanStep(start.x, start.y, start.x, start.y+1)) nextMove = kS; break; //start.y++; break;
		case 'a': if (nextMove == kStay && m->CanStep(start.x, start.y, start.x-1, start.y)) nextMove = kW; break; //start.x--; break;
		case 'd': if (nextMove == kStay && m->CanStep(start.x, start.y, start.x+1, start.y)) nextMove = kE; break; //start.x++; break;
			//case 'd':
		{
			MinDeltaPlusMinTOHTest();
			break;
		}
		case 'v':
		{
			TSVRC();
		}
		default:
			break;
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	return false;
}

void MDDeltaPlusMinTest()
{
	// In each domain
	std::vector<int> pattern = {0, 1, 2, 3, 4, 5, 6};

	MNPuzzle mnp(4, 4);
	MNPuzzleState t(4, 4);
	mnp.StoreGoal(t);
	
	// DIV compress + MR Ranking
	{
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb2(&mnp, t, pattern);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&mnp, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
	}

	// MOD compress + MR Ranking
	{
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb2(&mnp, t, pattern);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][none][mod] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&mnp, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][Delta][mod] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
	}

	// DIV compress + LEX Ranking
	{
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb(&mnp, t, pattern);
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb2(&mnp, t, pattern);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&mnp, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
	}

	// MOD compression + LEX
	{
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb(&mnp, t, pattern);
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb2(&mnp, t, pattern);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&mnp, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
	}

	// (0a) for DIV, MOD compression
	// (0b) for MR & LEX rankings
	
	// (1) Build a reasonably sized PDB
	
	// (2) min compress 2-10x
	
	// (3) Re-load pdb
	
	// (4) delta compress
	
	// (5) min compress 2-10x
	
}

void MinDeltaPlusMinTest()
{
	// naive ordering
	//	std::vector<int> pattern = {0, 1, 2, 3, 4, 5, 6};
	//	std::vector<int> pattern2 = {0, 1, 2, 3, 4, 5};

	// re-ordered naive ordering
	std::vector<int> pattern = {0, 4, 2, 3, 6, 5, 1};
	std::vector<int> pattern2 = {5, 0, 4, 3, 1, 2};

	
	// putting the blank last effectively removes it during min compression
	//std::vector<int> pattern = {1, 2, 3, 4, 5, 6, 0};
	//std::vector<int> pattern2 = {1, 2, 3, 4, 5, 0};
	
	MNPuzzle mnp(4, 4);
	MNPuzzleState t(4, 4);
	mnp.StoreGoal(t);
	
	// DIV compress + MR Ranking
	{
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb2(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> smallpdb(&mnp, t, pattern2);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		smallpdb.BuildPDB(t, std::thread::hardware_concurrency());
		smallpdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&smallpdb, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
	}
	
	// MOD compress + MR Ranking
	{
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb2(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> smallpdb(&mnp, t, pattern2);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		smallpdb.BuildPDB(t, std::thread::hardware_concurrency());
		smallpdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][none][mod] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&smallpdb, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][Delta][mod] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
	}
	
	// DIV compress + LEX Ranking
	{
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb(&mnp, t, pattern);
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb2(&mnp, t, pattern);
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> smallpdb(&mnp, t, pattern2);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		smallpdb.BuildPDB(t, std::thread::hardware_concurrency());
		smallpdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&smallpdb, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
	}
	
	// MOD compression + LEX
	{
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb(&mnp, t, pattern);
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> pdb2(&mnp, t, pattern);
		PermutationPDB<MNPuzzleState, slideDir, MNPuzzle> smallpdb(&mnp, t, pattern2);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		smallpdb.BuildPDB(t, std::thread::hardware_concurrency());
		smallpdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&smallpdb, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
	}
	
	// (0a) for DIV, MOD compression
	// (0b) for MR & LEX rankings
	
	// (1) Build a reasonably sized PDB
	
	// (2) min compress 2-10x
	
	// (3) Re-load pdb
	
	// (4) delta compress
	
	// (5) min compress 2-10x
	
}


void MinDeltaPlusMinTopSpinTest()
{
	// In each domain
	std::vector<int> pattern = {0, 1, 2, 3, 4, 5, 6};
	std::vector<int> pattern2 = {0, 1, 2, 3, 4, 5};
	
	TopSpin ts(16, 4);
	TopSpinState t(16, 4);
	ts.StoreGoal(t);
//	MNPuzzle mnp(4, 4);
//	MNPuzzleState t(4, 4);
//	mnp.StoreGoal(t);
	
	// DIV compress + MR Ranking
	{
		MR1PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb2(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState, TopSpinAction, TopSpin> smallpdb(&ts, t, pattern2);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		smallpdb.BuildPDB(t, std::thread::hardware_concurrency());
		smallpdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&smallpdb, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
	}
	
	// MOD compress + MR Ranking
	{
		MR1PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb2(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState, TopSpinAction, TopSpin> smallpdb(&ts, t, pattern2);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		smallpdb.BuildPDB(t, std::thread::hardware_concurrency());
		smallpdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][none][mod] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&smallpdb, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[mr1][Delta][mod] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
	}
	
	// DIV compress + LEX Ranking
	{
		PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb(&ts, t, pattern);
		PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb2(&ts, t, pattern);
		PermutationPDB<TopSpinState, TopSpinAction, TopSpin> smallpdb(&ts, t, pattern2);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		smallpdb.BuildPDB(t, std::thread::hardware_concurrency());
		smallpdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&smallpdb, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.DivCompress(compression, true);
		}
	}
	
	// MOD compression + LEX
	{
		PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb(&ts, t, pattern);
		PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb2(&ts, t, pattern);
		PermutationPDB<TopSpinState, TopSpinAction, TopSpin> smallpdb(&ts, t, pattern2);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		smallpdb.BuildPDB(t, std::thread::hardware_concurrency());
		smallpdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&smallpdb, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
		}
	}
}

void TSVRC()
{
	std::vector<int> pattern = {0, 1, 2, 3, 4, 5, 6};
	std::vector<int> pattern2 = {0, 1, 2, 3, 4, 5};
	
	TopSpin ts(16, 4);
	TopSpinState t(16, 4);
	ts.StoreGoal(t);

	// MOD compression + LEX
	{
		PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb(&ts, t, pattern);
		PermutationPDB<TopSpinState, TopSpinAction, TopSpin> pdb2(&ts, t, pattern);
		PermutationPDB<TopSpinState, TopSpinAction, TopSpin> smallpdb(&ts, t, pattern2);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		smallpdb.BuildPDB(t, std::thread::hardware_concurrency());
		smallpdb.PrintHistogram();
		
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][none][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			//pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
			pdb2.DivCompress(compression, true);
		}
		for (int compression = 1; compression <= 4; compression*=2)
		{
			printf("[lex][Delta][min] VRC compression to %d bits\n", compression);
			pdb2 = pdb;
			pdb2.ValueRangeCompress(compression, true);
		}

		printf("Performing delta compression. New distribution:\n");
		t.Reset();
		pdb.DeltaCompress(&smallpdb, t, true);
		for (int compression = 2; compression <= 10; compression++)
		{
			printf("[lex][Delta][min] Compressing by a factor of %d\n", compression);
			pdb2 = pdb;
			//pdb2.ModCompress(pdb.GetPDBSize()/compression, true);
			pdb2.DivCompress(compression, true);
		}
		for (int compression = 2; compression <= 10; compression++)
		{
			for (int bits = 1; bits <= 2; bits*=2)
			{
				printf("[lex][Delta][min] Compressing by a factor of %d + VRC to %d bits\n", compression, bits);
				pdb2 = pdb;
				//pdb2.ModCompress(pdb.GetPDBSize()/compression, false);
				pdb2.DivCompress(compression, true);
				pdb2.ValueRangeCompress(bits, true);
			}
		}

	}
}

void MinDeltaPlusMinTOHTest()
{
	const int numDisks = 15; // disks - 2 (4^13 - 67 million)
	TOH<numDisks> toh;
	TOHState<numDisks> s, g;

	TOHState<numDisks> goal;
	TOH<numDisks-2> absToh1;
	TOH<numDisks-4> absToh4;
	TOHState<numDisks-2> absTohState1;
	TOHPDB<numDisks-2, numDisks> pdb1(&absToh1);
	TOHPDB<numDisks-2, numDisks> pdb1a(&absToh1);
	TOHPDB<numDisks-2, numDisks> pdb1b(&absToh1);
	TOHPDB<numDisks-2, numDisks> pdbSmaller(&absToh1);
	TOHPDB<numDisks-4, numDisks> pdbSmallest(&absToh4);

	goal.Reset();
	pdbSmallest.BuildPDB(goal, std::thread::hardware_concurrency());
	
	goal.Reset();
	pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
	pdb1b = pdb1;
	//	goal.Reset();
//	pdbSmaller.BuildPDB(goal, std::thread::hardware_concurrency());
	
	printf("Starting TOH DIV\n");
	{
		pdbSmaller = pdb1;
		pdbSmaller.DivCompress(16, true);

		pdb1.PrintHistogram();
		for (int compression = 2; compression <= 16; compression++)
		{
			printf("[toh][DIV] Compressing by a factor of %d\n", compression);
			pdb1a = pdb1;
			pdb1a.DivCompress(compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		pdb1.DeltaCompress(&pdbSmaller, goal, true);
		for (int compression = 2; compression <= 16; compression++)
		{
			printf("[toh][DIV] Compressing by a factor of %d\n", compression);
			pdb1a = pdb1;
			pdb1a.DivCompress(compression, true);
		}
	}

	
	printf("Starting TOH MOD\n");
	{
		pdb1 = pdb1b;
		pdbSmaller = pdb1;
		pdbSmaller.ModCompress(pdb1.GetPDBSize()/16, true);

		pdb1.PrintHistogram();
		for (int compression = 2; compression <= 16; compression++)
		{
			printf("[toh][MOD] Compressing by a factor of %d\n", compression);
			pdb1a = pdb1;
			pdb1a.ModCompress(pdb1.GetPDBSize()/compression, true);
		}
		printf("Performing delta compression. New distribution:\n");
		pdb1.DeltaCompress(&pdbSmaller, goal, true);
		for (int compression = 2; compression <= 16; compression++)
		{
			printf("[toh][MOD] Compressing by a factor of %d\n", compression);
			pdb1a = pdb1;
			pdb1a.ModCompress(pdb1.GetPDBSize()/compression, true);
		}
	}
}

/* Code for VRC on TOH */
//case 'v': // value range compression
//{
//	goal.Reset();
//	// 268435456 entries (256 MB)
//	printf("Dist: Full PDB\n");
//	pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
//	pdb1.ValueRangeCompress(4, true);
//	break;
//	// 67108864 entries (64 MB) (pdb2) [72.171778]
//	// 16777216 entries (16 MB) (pdb3)
//	//pdb3.BuildPDB(goal, std::thread::hardware_concurrency());
//	
//	pdb1a = pdb1;
//	pdb1b = pdb1;
//	//pdb1a.DivCompress(16, true); // 16 MB
//	printf("Dist: Div factor\n");
//	//pdb1a.DivCompress(32, true); // 8 MB
//	pdb1a.DivCompress(64, true); // 4 MB
//	printf("Dist: Delta (over original)\n");
//	pdb1.DeltaCompress(&pdb1a, goal, true);
//	
//	// delta pdb2 = 14.867143 / after VRC = 14.864208
//	// delta pdb3 = 28.029138 / after VRC = 27.922551
//	//			pdb1.DeltaCompress(&pdb3, goal, true);
//	printf("Dist: VRC\n");
//	pdb1.ValueRangeCompress(2, true);
//	
//	printf("Dist: Straight 4x compression (≈2 bits)\n");
//	pdb1b.DivCompress(4, true);
//	printf("Dist: Straight 8x compression (≈1 bit\n");
//	pdb1b.DivCompress(2, true);
//	//pdb.DivCompress(4, true);
//	//pdb.ModCompress(pdb.GetPDBSize()/4, true);
//	h.lookups.resize(0);
//	h.lookups.push_back({kMaxNode, 1, 2});
//	h.lookups.push_back({kLeafNode, 0, 0});
//	h.lookups.push_back({kLeafNode, 1, 1});
//	h.heuristics.resize(0);
//	h.heuristics.push_back(&toh);
//	h.heuristics.push_back(&pdb1);
//	SolveProblem();
//	break;
//}
