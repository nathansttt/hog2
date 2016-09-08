/**
 * Delta: Driver.cpp
 *
 * This code performs experiments for a yet-to-be submitted paper. More details
 * will be provided here when the paper is accepted/published
 */

#include <cstring>
#include "Common.h"
#include "PermutationPDB.h"
#include "LexPermutationPDB.h"
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
#include "ParallelIDAStar.h"

void MDDeltaPlusMinTest();
void MinDeltaPlusMinTest();
void MinDeltaPlusMinTopSpinTest();
void MinDeltaPlusMinTOHTest();
void TSVRC();
void TSIDAVRC(int bits, int compressionFactor, int problem);
void TSBiVRC(int bits, int compressionFactor, int problem = -1);
template <int N, int k>
float GetAveragePDBValue(int elts, int bits, int factor, bool div, bool mr);
template <int N, int k>
void PrintHeuristicTables();
template <int N, int k>
float PrebuildHeuristic(int elts, bool mr);

char prefix[1024] = "";

const int N = 18;
const int k = 4;
//const int K = 10; // cutoff 7
//const int K = 4; // cutoff 8
const int K = 4; // cutoff ?
const int cutoff = 25;

void TestTSBiVRC(Heuristic<TopSpinState<N>> *f, Heuristic<TopSpinState<N>> *b, int problem = -1);

int main(int argc, char* argv[])
{
	// Turn off text buffering
	setvbuf(stdout, NULL, _IONBF, 0);
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
	InstallKeyboardHandler(MyDisplayHandler, "TS VRC", "Top Spin VRC Experiments", kAnyModifier, 'v');
	InstallKeyboardHandler(MyDisplayHandler, "TS Bi VRC", "Top Spin Bidirectional VRC Experiments", kAnyModifier, 'b');

	
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

	InstallCommandLineHandler(MyCLHandler, "-pdb", "-pdb <dir>", "Set the directory to use for PDBs");
	InstallCommandLineHandler(MyCLHandler, "-test", "-test <bits> <factor>", "Basic test comparing A*, IDA*, MM");
	
	InstallWindowHandler(MyWindowHandler);
	InstallMouseClickHandler(MyClickHandler);
}

MNPuzzleState<4, 4> s, t;
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
	if ((strcmp(argument[0], "-pdb") == 0) && maxNumArgs > 1)
	{
		strncpy(prefix, argument[1], 1024);
		return 2;
	}
	else if (strcmp(argument[0], "-test") == 0 && maxNumArgs >= 3)
	{
		TSBiVRC(atoi(argument[1]), atoi(argument[2]));
		exit(0);
	}
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
//			MinDeltaPlusMinTOHTest();
			break;
		}
		case 'v':
		{
//			printf("-=-=-=8 bits=-=-=-\n");
			TSBiVRC(8, 4);
//			TSIDAVRC(8, 1, 0);
//			printf("-=-=-=4 bits=-=-=-\n");
//			TSBiVRC(4, 1);
//			GetAveragePDBValue<18, 4>(8, 3, 1, true, false);
//			TSVRC();
			break;
		}
			case 'b':
		{
//			TSBiVRC(4, 1);
//			ZeroHeuristic<TopSpinState<N>> z;
//			TestTSBiVRC(&z, &z);
//			template <int N, int k>
//			float GetAveragePDBValue(int elts, int bits, int factor, bool div, bool mr)

			PrintHeuristicTables<18, 2>();
			PrintHeuristicTables<18, 4>();
			PrintHeuristicTables<18, 6>();
			PrintHeuristicTables<18, 8>();
			PrintHeuristicTables<18, 10>();
			break;
		}
		default:
			break;
	}
}

bool MyClickHandler(unsigned long , int, int, point3d , tButtonType , tMouseEventType )
{
	// In each domain
	std::vector<int> p1 = {0,1,4,5};
	std::vector<int> p2 = {0,2,3,6,7};
	std::vector<int> p3 = {0,8,9,12,13};
	std::vector<int> p4 = {0,10,11,14,15};
	std::vector<int> p5 = {0,1,2,3,4,5,6,7};
	
	
	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> t;
	mnp.StoreGoal(t);

	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb1(&mnp, t, p1);
	pdb1.BuildPDB(t, std::thread::hardware_concurrency());
	pdb1.PrintHistogram();

	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, t, p2);
	pdb2.BuildPDB(t, std::thread::hardware_concurrency());
	pdb2.PrintHistogram();

	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb3(&mnp, t, p3);
	pdb3.BuildPDB(t, std::thread::hardware_concurrency());
	pdb3.PrintHistogram();

	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb4(&mnp, t, p4);
	pdb4.BuildPDB(t, std::thread::hardware_concurrency());
	pdb4.PrintHistogram();

	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb5(&mnp, t, p5);
	pdb5.BuildPDB(t, std::thread::hardware_concurrency());
	pdb5.PrintHistogram();

	
	return false;
}

void MDDeltaPlusMinTest()
{
	// In each domain
	std::vector<int> pattern = {0, 1, 2, 3, 4, 5, 6};

	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> t;
	mnp.StoreGoal(t);
	
	// DIV compress + MR Ranking
	{
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, t, pattern);
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
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, t, pattern);
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
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(&mnp, t, pattern);
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, t, pattern);
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
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(&mnp, t, pattern);
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, t, pattern);
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
	
	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> t;
	mnp.StoreGoal(t);
	
	// DIV compress + MR Ranking
	{
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> smallpdb(&mnp, t, pattern2);
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
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, t, pattern);
		MR1PermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> smallpdb(&mnp, t, pattern2);
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
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(&mnp, t, pattern);
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, t, pattern);
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> smallpdb(&mnp, t, pattern2);
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
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb(&mnp, t, pattern);
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, t, pattern);
		LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> smallpdb(&mnp, t, pattern2);
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
	
	TopSpin<N, k> ts;
	TopSpinState<N> t;
	ts.StoreGoal(t);
	
	// DIV compress + MR Ranking
	{
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb2(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> smallpdb(&ts, t, pattern2);
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
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb2(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> smallpdb(&ts, t, pattern2);
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
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb2(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> smallpdb(&ts, t, pattern2);
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
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb2(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> smallpdb(&ts, t, pattern2);
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
#include "MM.h"

void TestTSBiVRC(Heuristic<TopSpinState<N>> *f, Heuristic<TopSpinState<N>> *b, int problem)
{
	TemplateAStar<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> astar;
	IDAStar<TopSpinState<N>, TopSpinAction> ida;
	ParallelIDAStar<TopSpin<N, K>, TopSpinState<N>, TopSpinAction> pida;
	MM<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> mm;
	TopSpin<N, K> ts;
	TopSpinState<N> s;
	TopSpinState<N> g;
	std::vector<TopSpinState<N>> thePath;
	std::vector<TopSpinAction> actionPath;
	ts.StoreGoal(g);
	ZeroHeuristic<TopSpinState<N>> z;
	
	int table[] = {52058078,116173544,208694125,131936966,141559500,133800745,194246206,50028346,167007978,207116816,163867037,119897198,201847476,210859515,117688410,121633885};
	int table2[] = {145008714,165971878,154717942,218927374,182772845,5808407,19155194,137438954,13143598,124513215,132635260,39667704,2462244,41006424,214146208,54305743};
	int first = 0, last = 50;
	if (problem != -1)
	{
		first = problem;
		last = problem+1;
	}
	for (int count = first; count < last; count++)
	{
		printf("Seed: %d\n", table[count&0xF]^table2[(count>>4)&0xF]);
		srandom(table[count&0xF]^table2[(count>>4)&0xF]);
		for (int x = 0; x < 200; x++)
			ts.ApplyAction(s, random()%N);
		Timer timer;
		
		if (0)
		{
			printf("-=-=-MM0-=-=-\n");
			timer.StartTimer();
			mm.GetPath(&ts, s, g, &z, &z, thePath);
			timer.EndTimer();
			printf("%llu nodes expanded\n", mm.GetNodesExpanded());
			printf("Solution path length %1.0f\n", ts.GetPathLength(thePath));
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
		}

		
		if (0)
		{
			printf("-=-=-MM-=-=-\n");
			timer.StartTimer();
			mm.GetPath(&ts, s, g, f, b, thePath);
			timer.EndTimer();
			printf("%llu nodes expanded; %llu re-expanded\n", mm.GetNodesExpanded(), mm.GetUniqueNodesExpanded());
			printf("Solution path length %1.0f\n", ts.GetPathLength(thePath));
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
		}
		
		if (0)
		{
			printf("-=-=-A*-=-=-\n");
			astar.SetHeuristic(f);
			timer.StartTimer();
			astar.GetPath(&ts, s, g, thePath);
			timer.EndTimer();
			printf("%llu nodes expanded\n", astar.GetNodesExpanded());
			printf("Solution path length %1.0f\n", ts.GetPathLength(thePath));
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
		}
		
		if (1)
		{
			printf("-=-=-IDA*-=-=-\n");
			ida.SetHeuristic(f);
			ida.SetUseBDPathMax(true);
			ts.SetPruneSuccessors(true);
			timer.StartTimer();
			ida.GetPath(&ts, s, g, actionPath);
			timer.EndTimer();
			printf("%llu nodes expanded; %llu generated\n", ida.GetNodesExpanded(), ida.GetNodesTouched());
			printf("Solution path length %lu\n", actionPath.size());
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
			ts.SetPruneSuccessors(false);

//			printf("Dynamic distribution\n");
//			for (int x = 0; x < 255; x++)
//				if (f->histogram[x] != 0)
//					printf("%d\t%llu\n", x, f->histogram[x]);
		}

		if (0)
		{
			
			printf("-=-=-PIDA*-=-=-\n");
			pida.SetHeuristic(f);
			ts.SetPruneSuccessors(true);
			timer.StartTimer();
			pida.GetPath(&ts, s, g, actionPath);
			timer.EndTimer();
			printf("%llu nodes expanded; %llu generated\n", ida.GetNodesExpanded(), ida.GetNodesTouched());
			printf("Solution path length %lu\n", actionPath.size());
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
			ts.SetPruneSuccessors(false);
		}

	}
	
	exit(0);
}

void TSIDAVRC(int bits, int compressionFactor, int problem)
{
	std::vector<int> pattern1 = {0, 1, 2, 3, 4, 5, 6, 7};//, 5, 6, 7};
	std::vector<int> pattern2 = {6, 7, 8, 9, 10, 11, 12, 13};//, 5, 6, 7};
	std::vector<int> pattern3 = {12, 13, 14, 15, 16, 17, 0, 1};//, 5, 6, 7};
	
	TopSpin<N, K> ts;
	TopSpinState<N> t, d, goal;
	ts.StoreGoal(t);
	std::vector<std::vector<double>> averages;
	
	if (1)
	{
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb1(&ts, t, pattern1);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb2(&ts, t, pattern2);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb3(&ts, t, pattern3);
		if (!pdb1.Load(prefix))
		{
			pdb1.BuildPDB(t, std::thread::hardware_concurrency());
			pdb1.Save(prefix);
		}
		if (!pdb2.Load(prefix))
		{
			pdb2.BuildPDB(t, std::thread::hardware_concurrency());
			pdb2.Save(prefix);
		}
		if (!pdb3.Load(prefix))
		{
			pdb3.BuildPDB(t, std::thread::hardware_concurrency());
			pdb3.Save(prefix);
		}
		if (compressionFactor > 1)
		{
			pdb1.DivCompress(compressionFactor, true);
			pdb2.DivCompress(compressionFactor, true);
			pdb3.DivCompress(compressionFactor, true);
		}
		if (bits < 8)
		{
			std::vector<int> v;
			std::vector<uint64_t> hist;
			pdb1.GetHistogram(hist);
			pdb1.PrintHistogram();
			int interval = 1<<bits;
			float i = float(hist.size())/float(interval);
			if (i < 1) i = 1;
			for (int x = 0; x < interval; x++)
				v.push_back(int(float(x)*i));
			pdb1.ValueCompress(v, true);
			pdb2.ValueCompress(v, true);
			pdb3.ValueCompress(v, true);
		}
		
		Heuristic<TopSpinState<N>> h;
		
		h.lookups.resize(0);
		h.lookups.push_back({kMaxNode, 1, 3});
		h.lookups.push_back({kLeafNode, 0, 0});
		h.lookups.push_back({kLeafNode, 1, 1});
		h.lookups.push_back({kLeafNode, 2, 2});
		h.heuristics.resize(0);
		h.heuristics.push_back(&pdb1);
		h.heuristics.push_back(&pdb2);
		h.heuristics.push_back(&pdb3);
		PermutationPuzzle::ArbitraryGoalPermutation<TopSpinState<N>, TopSpin<N, K>> p(&h, &ts);
		//ZeroHeuristic<TopSpinState<N>> z;
		TestTSBiVRC(&h, &p, problem);
	}
}


void TSBiVRC(int bits, int compressionFactor, int problem)
{
	std::vector<int> pattern1 = {0, 1, 2, 3, 4, 5, 6, 7};//, 5, 6, 7};
	std::vector<int> pattern2 = {6, 7, 8, 9, 10, 11, 12, 13};//, 5, 6, 7};
	std::vector<int> pattern3 = {12, 13, 14, 15, 16, 17, 0, 1};//, 5, 6, 7};
	
	TopSpin<N, K> ts;
	TopSpinState<N> t, d, goal;
	ts.StoreGoal(t);
	std::vector<std::vector<double>> averages;
	
	// baseline
	if (0)
	{
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb1(&ts, t, pattern1);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb2(&ts, t, pattern2);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb3(&ts, t, pattern3);
		if (!pdb1.Load(prefix))
		{
			pdb1.BuildPDB(t, std::thread::hardware_concurrency());
			pdb1.Save(prefix);
		}
		if (!pdb2.Load(prefix))
		{
			pdb2.BuildPDB(t, std::thread::hardware_concurrency());
			pdb2.Save(prefix);
		}
		if (!pdb3.Load(prefix))
		{
			pdb3.BuildPDB(t, std::thread::hardware_concurrency());
			pdb3.Save(prefix);
		}
		
		Heuristic<TopSpinState<N>> h;

		h.lookups.resize(0);
		h.lookups.push_back({kMaxNode, 1, 3});
		h.lookups.push_back({kLeafNode, 0, 0});
		h.lookups.push_back({kLeafNode, 1, 1});
		h.lookups.push_back({kLeafNode, 2, 2});
		h.heuristics.resize(0);
		h.heuristics.push_back(&pdb1);
		h.heuristics.push_back(&pdb2);
		h.heuristics.push_back(&pdb3);
		PermutationPuzzle::ArbitraryGoalPermutation<TopSpinState<N>, TopSpin<N, K>> p(&h, &ts);
		//ZeroHeuristic<TopSpinState<N>> z;
		TestTSBiVRC(&h, &p, problem);
	}

	// first VRC test
	if (1)
	{
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb1(&ts, t, pattern1);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb2(&ts, t, pattern2);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb3(&ts, t, pattern3);
		if (!pdb1.Load(prefix))
		{
			pdb1.BuildPDB(t, std::thread::hardware_concurrency());
			pdb1.Save(prefix);
		}
		if (!pdb2.Load(prefix))
		{
			pdb2.BuildPDB(t, std::thread::hardware_concurrency());
			pdb2.Save(prefix);
		}
		if (!pdb3.Load(prefix))
		{
			pdb3.BuildPDB(t, std::thread::hardware_concurrency());
			pdb3.Save(prefix);
		}
		if (compressionFactor > 1)
		{
			pdb1.DivCompress(compressionFactor, true);
			pdb2.DivCompress(compressionFactor, true);
			pdb3.DivCompress(compressionFactor, true);
		}
		if (bits < 8)
		{
			//pdb1.ZeroLowValues(cutoff);
			pdb1.ValueRangeCompress(bits, true);
			//pdb2.ZeroLowValues(cutoff);
			pdb2.ValueRangeCompress(bits, true);
			//pdb3.ZeroLowValues(cutoff);
			pdb3.ValueRangeCompress(bits, true);
		}
		
		Heuristic<TopSpinState<N>> h;
		
		h.lookups.resize(0);
		h.lookups.push_back({kMaxNode, 1, 3});
		h.lookups.push_back({kLeafNode, 0, 0});
		h.lookups.push_back({kLeafNode, 1, 1});
		h.lookups.push_back({kLeafNode, 2, 2});
		h.heuristics.resize(0);
		h.heuristics.push_back(&pdb1);
		h.heuristics.push_back(&pdb2);
		h.heuristics.push_back(&pdb3);
		PermutationPuzzle::ArbitraryGoalPermutation<TopSpinState<N>, TopSpin<N, K>> p(&h, &ts);
		//ZeroHeuristic<TopSpinState<N>> z;
		TestTSBiVRC(&h, &p, problem);
	}
	
	if (0)
	{
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb(&ts, t, pattern1);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb2(&ts, t, pattern1);
		pdb.BuildPDB(t, std::thread::hardware_concurrency());
		pdb.PrintHistogram();
		
		averages.resize(5);
		for (int compression = 1; compression <= 16; compression*=2)
		{
			int table[] = {0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4};
			averages[table[compression]].resize(4);
			for (int bits = 1; bits <= 8; bits*=2)
			{
				printf("[lex][none][min] Compressing by a factor of %d\n", compression);
				pdb2 = pdb;
				pdb2.DivCompress(compression, false);
				pdb2.ZeroLowValues(cutoff);
				pdb2.PrintHistogram();

				printf("[lex][Delta][min] VRC compression to %d bits [factor of %d overall]\n", bits, (4/bits)*compression);
				pdb2.ZeroLowValues(cutoff);
				pdb2.ValueRangeCompress(bits, false);
				averages[table[compression]][table[bits]] = pdb2.PrintHistogram();
			}
		}
		
		for (int x = 0; x < averages.size(); x++)
		{
			for (int y = 0; y < averages[x].size(); y++)
				printf("%1.2f\t", averages[x][averages[x].size()-y-1]);
			printf("\n");
		}
	}
}


template <int N, int k>
void PrintHeuristicTables()
{
	for (int mr = 0; mr <= 1; mr++)
	{
		for (int elts = 6; elts <= 8; elts++)
		{
			PrebuildHeuristic<N, k>(elts, mr);
		}
	}
	
	printf("Top Spin N=%d, k=%d\n", N, k);
	for (int div = 0; div <= 1; div++)
	{
		for (int mr = 0; mr <= 1; mr++)
		{
			for (int elts = 6; elts <= 8; elts++)
			{
				printf("Compression: %s; ranking: %s; pattern: %d\n", div?"DIV":"MOD", mr?"MR":"LEX", elts);
				printf("\t1\t2\t3\t4\t5\t6\n");
				for (int bits = 2; bits <= 8; bits*=2)
				{
					printf("%d\t", bits);
					for (int factor = 1; factor <= 6; factor++)
					{
						printf("%1.3f\t", GetAveragePDBValue<N, k>(elts, bits, factor, div, mr));
					}
					printf("\n");
				}
			}
		}
	}
}

template <int N, int k>
float PrebuildHeuristic(int elts, bool mr)
{
	std::vector<int> pattern;
	TopSpin<N, k> ts;
	TopSpinState<N> t;
	
	for (int x = 0; x < elts; x++)
		pattern.push_back(x);
	PDBHeuristic<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> *h;
	if (mr == true)
		h = new MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>>(&ts, t, pattern);
	else
		h = new LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>>(&ts, t, pattern);
	if (!h->Load(prefix))
	{
		h->BuildPDB(t, std::thread::hardware_concurrency());
		h->Save(prefix);
	}
	delete h;
}

template <int N, int k>
float GetAveragePDBValue(int elts, int bits, int factor, bool div, bool mr)
{
	std::vector<int> pattern;
	TopSpin<N, k> ts;
	TopSpinState<N> t;

	for (int x = 0; x < elts; x++)
		pattern.push_back(x);
	PDBHeuristic<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> *h;
	if (mr == true)
		h = new MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>>(&ts, t, pattern);
	else
		h = new LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>>(&ts, t, pattern);
	if (!h->Load(prefix))
	{
		h->BuildPDB(t, std::thread::hardware_concurrency());
		h->Save(prefix);
	}
	if (factor > 1)
	{
		if (div)
			h->DivCompress(factor, false);
		else
			h->ModCompress(factor, false);
	}
	h->ValueRangeCompress(bits, true);
	float f = h->GetAverageValue();
	delete h;
	return f;
}

void TSVRC()
{
	std::vector<int> pattern = {0, 1, 2, 3, 4, 5, 6};
	std::vector<int> pattern2 = {0, 1, 2, 3, 4, 5};
	
	TopSpin<N, k> ts;
	TopSpinState<N> t;
	ts.StoreGoal(t);

	// MOD compression + LEX
	{
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> pdb2(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, k>> smallpdb(&ts, t, pattern2);
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
