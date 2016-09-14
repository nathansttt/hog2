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
#include "RubiksCube.h"
#include "MNPuzzle.h"

void TOHTest(int deltaFactor, int bits, int factor, int first = 0, int last = 50);
void STPTest(int bits, int factor);
void RubikDynamicTest();
void GetRubikLength14Instance(RubiksState &start, int which);
void GetRubikStep14Instance(RubiksState &start, int which);
void MDDeltaPlusMinTest();
void MinDeltaPlusMinTest();
void MinDeltaPlusMinTopSpinTest();
void MinDeltaPlusMinTOHTest();
void TSVRC();
void TSBiVRC(int bits, int compressionFactor, int first = 0, int last = 50);
template <int N, int k>
float GetAveragePDBValue(int elts, int bits, int factor, bool div, bool mr);
template <int N, int k>
void PrintHeuristicTables();
template <int N, int k>
float PrebuildHeuristic(int elts, bool mr);

char prefix[1024] = "";

const int N = 18;
const int K = 4; // cutoff ?
//const int K = 10; // cutoff 7
//const int K = 4; // cutoff 8
//const int cutoff = 25;

void TestTSBiVRC(Heuristic<TopSpinState<N>> *f, Heuristic<TopSpinState<N>> *b, int first = 0, int last = 50);

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
	InstallKeyboardHandler(MyDisplayHandler, "TOH", "TOH Test", kAnyModifier, 'h');
	//InstallKeyboardHandler(MyDisplayHandler, "Delta + Min", "Test what happens when we add delta over min with min compression on TOH", kAnyModifier, 'd');

	InstallKeyboardHandler(MyDisplayHandler, "Step Simulation", "If the simulation is paused, step forward .1 sec.", kAnyModifier, 'o');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step forward .1 sec in history", kAnyModifier, '}');
	InstallKeyboardHandler(MyDisplayHandler, "Step History", "If the simulation is paused, step back .1 sec in history", kAnyModifier, '{');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Increase abstraction type", kAnyModifier, ']');
	InstallKeyboardHandler(MyDisplayHandler, "Step Abs Type", "Decrease abstraction type", kAnyModifier, '[');

	InstallCommandLineHandler(MyCLHandler, "-pdb", "-pdb <dir>", "Set the directory to use for PDBs");
	InstallCommandLineHandler(MyCLHandler, "-test", "-test <bits> <factor>", "Basic test comparing A*, IDA*, MM");
	InstallCommandLineHandler(MyCLHandler, "-rubik", "-rubik", "Duplicating Zahavi et al 2007.");
	InstallCommandLineHandler(MyCLHandler, "-toh", "-toh <delta> <bits> <factor>", "Run TOH test with <delta> compression factor, <bits> VC, <factor> min.");
	
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
	else if (strcmp(argument[0], "-rubik") == 0 && maxNumArgs >= 1)
	{
		RubikDynamicTest();
		//TSBiVRC(atoi(argument[1]), atoi(argument[2]));
		exit(0);
	}
	else if (strcmp(argument[0], "-toh") == 0 && maxNumArgs >= 4)
	{
		TOHTest(atoi(argument[1]), atoi(argument[2]), atoi(argument[3]));
		exit(0);
	}
	return 0;
}

void MyDisplayHandler(unsigned long windowID, tKeyboardModifier mod, char key)
{
	switch (key)
	{
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
			//MinDeltaPlusMinTest();
//			STPTest(8, 1);
			STPTest(4, 1);
			STPTest(8, 2);
			break;
		}
			break;
		case 'h':
		{
			//TOHTest(int deltaFactor, int bits, int factor, int first, int last)
			//TOHTest(64, 8, 1, 0, 10);
			//TOHTest(64, 2, 1, 0, 10);
			//TOHTest(64, 8, 4, 0, 10);
			TOHTest(64, 1, 1, 0, 10);
			exit(0);
		}
		case 't':
		{
//			TSBiVRC(8, 1, 0, 10);
//			TSBiVRC(4, 1, 0, 10);
			TSBiVRC(3, 1, 0, 10);
//			TSBiVRC(2, 1, 0, 10);
//			TSBiVRC(4, 1);
			exit(0);
//			std::vector<int> pattern = {0,1,2,3,4,5,6,7};
//			MNPuzzle<4, 4> mnp;
//			MNPuzzleState<4, 4> t;
//			mnp.StoreGoal(t);
//			LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb1(&mnp, t, pattern);
//			LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>, 4> pdb2(&mnp, t, pattern);
//			if (pdb1.Load(prefix) == false)
//			{
//				pdb1.BuildPDB(t, std::thread::hardware_concurrency());
//				pdb1.PrintHistogram();
//			}
//			printf("Min compress factor 2\n");
//			pdb1.DivCompress(2, true);
//			printf("VRC compress into new PDB (fewer bits)");
//			pdb1.ValueRangeCompress(&pdb2, true);
//			printf("VRC compress into same PDB");
//			pdb1.ValueRangeCompress(4, true);
//			break;
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
		case 'r':
		{
			RubikDynamicTest();
			break;
		}
		case 'v':
		{
//			printf("-=-=-=8 bits=-=-=-\n");
			TSBiVRC(2, 1);
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
	
	TopSpin<N, K> ts;
	TopSpinState<N> t;
	ts.StoreGoal(t);
	
	// DIV compress + MR Ranking
	{
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb2(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> smallpdb(&ts, t, pattern2);
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
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb2(&ts, t, pattern);
		MR1PermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> smallpdb(&ts, t, pattern2);
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
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb2(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> smallpdb(&ts, t, pattern2);
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
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb2(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> smallpdb(&ts, t, pattern2);
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

void TestTSBiVRC(Heuristic<TopSpinState<N>> *f, Heuristic<TopSpinState<N>> *b, int first, int last)
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
	mm.PrintHDist();
	
//	exit(0);
}

void TSBiVRC(int bits, int compressionFactor, int first, int last)
{
//	std::vector<int> pattern1 = {0, 1, 2, 3, 4, 5, 6, 7};//, 5, 6, 7};
//	std::vector<int> pattern2 = {4, 5, 6, 7, 8, 9, 10, 11};//, 5, 6, 7};
//	std::vector<int> pattern3 = {8, 9, 10, 11, 0, 1, 2, 3};//, 5, 6, 7};
	std::vector<int> pattern1 = {0, 1, 2, 3, 4, 5, 6, 7};//, 5, 6, 7};
	std::vector<int> pattern2 = {6, 7, 8, 9, 10, 11, 12, 13};//, 5, 6, 7};
	std::vector<int> pattern3 = {12, 13, 14, 15, 16, 17, 0, 1};//, 5, 6, 7};
	
	printf("-->Testing TS(%d, %d): %d bits, %d factor<--\n", N, K, bits, compressionFactor);
	
	TopSpin<N, K> ts;
	TopSpinState<N> t, d, goal;
	ts.StoreGoal(t);
	std::vector<std::vector<double>> averages;
	

	{
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 8> pdb1(&ts, t, pattern1);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 8> pdb2(&ts, t, pattern2);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 8> pdb3(&ts, t, pattern3);

		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 2> pdb1_2(&ts, t, pattern1);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 3> pdb1_3(&ts, t, pattern1);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 4> pdb1_4(&ts, t, pattern1);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 5> pdb1_5(&ts, t, pattern1);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 2> pdb2_2(&ts, t, pattern2);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 3> pdb2_3(&ts, t, pattern2);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 4> pdb2_4(&ts, t, pattern2);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 5> pdb2_5(&ts, t, pattern2);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 2> pdb3_2(&ts, t, pattern3);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 3> pdb3_3(&ts, t, pattern3);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 4> pdb3_4(&ts, t, pattern3);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>, 5> pdb3_5(&ts, t, pattern3);
		Heuristic<TopSpinState<N>> *h1 = &pdb1;
		Heuristic<TopSpinState<N>> *h2 = &pdb2;
		Heuristic<TopSpinState<N>> *h3 = &pdb3;
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
		
		std::vector<uint64_t> dist = {1, 281,1329858,6904837,16231845,22415146,26628364,27444566,26581762,24040692,21344755,17742727,13977704,10520226,7509651,5122460,3339498,2108385,1306697,799294,481143,285533,163165,88800,42981,17298,5384,1213,127,4};
//		std::vector<int> cutoffs = {0, 9, 11, 13};
		std::vector<int> cutoffs = {0, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 14, 16, 18, 20, 22, 24};

		if (bits < 8)
		{
			if (1)
			{
				switch (bits)
				{
					case 2:
						pdb1.ValueRangeCompress(&pdb1_2, true);
						pdb2.ValueRangeCompress(&pdb2_2, true);
						pdb3.ValueRangeCompress(&pdb3_2, true);
						h1 = &pdb1_2;
						h2 = &pdb2_2;
						h3 = &pdb3_2;
						break;
					case 3:
						pdb1.ValueRangeCompress(&pdb1_3, true);
						pdb2.ValueRangeCompress(&pdb2_3, true);
						pdb3.ValueRangeCompress(&pdb3_3, true);
						h1 = &pdb1_3;
						h2 = &pdb2_3;
						h3 = &pdb3_3;
						break;
					case 4:
						pdb1.ValueRangeCompress(&pdb1_4, true);
						pdb2.ValueRangeCompress(&pdb2_4, true);
						pdb3.ValueRangeCompress(&pdb3_4, true);
						h1 = &pdb1_4;
						h2 = &pdb2_4;
						h3 = &pdb3_4;
						break;
					case 5:
						pdb1.ValueRangeCompress(&pdb1_5, true);
						pdb2.ValueRangeCompress(&pdb2_5, true);
						pdb3.ValueRangeCompress(&pdb3_5, true);
						h1 = &pdb1_5;
						h2 = &pdb2_5;
						h3 = &pdb3_5;
						break;
					default:
						printf("Unhandled number of bits\n");
						exit(0);
				}
			}
			else {
				pdb1.ValueRangeCompress(bits, true);
				pdb2.ValueRangeCompress(bits, true);
				pdb3.ValueRangeCompress(bits, true);
//				pdb1.CustomValueRangeCompress(dist, bits, true);
//				pdb2.CustomValueRangeCompress(dist, bits, true);
//				pdb3.CustomValueRangeCompress(dist, bits, true);
//				pdb1.ValueCompress(cutoffs, true);
//				pdb2.ValueCompress(cutoffs, true);
//				pdb3.ValueCompress(cutoffs, true);
			}
		}
		
		Heuristic<TopSpinState<N>> h;
		
		h.lookups.resize(0);
		h.lookups.push_back({kMaxNode, 1, 3});
		h.lookups.push_back({kLeafNode, 0, 0});
		h.lookups.push_back({kLeafNode, 1, 1});
		h.lookups.push_back({kLeafNode, 2, 2});
		h.heuristics.resize(0);
		h.heuristics.push_back(h1);
		h.heuristics.push_back(h2);
		h.heuristics.push_back(h3);
		PermutationPuzzle::ArbitraryGoalPermutation<TopSpinState<N>, TopSpin<N, K>> p(&h, &ts);
		//ZeroHeuristic<TopSpinState<N>> z;
		TestTSBiVRC(&h, &p, first, last);

		printf("Dynamic distribution\n");
		for (int x = 0; x < 255; x++)
			if (h.histogram[x] != 0)
				printf("%d\t%llu\n", x, h.histogram[x]);
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
	
	TopSpin<N, K> ts;
	TopSpinState<N> t;
	ts.StoreGoal(t);

	// MOD compression + LEX
	{
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> pdb2(&ts, t, pattern);
		LexPermutationPDB<TopSpinState<N>, TopSpinAction, TopSpin<N, K>> smallpdb(&ts, t, pattern2);
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

template <int N>
void TestTOH(Heuristic<TOHState<N>> *f, int first, int last)
{
	TemplateAStar<TOHState<N>, TOHMove, TOH<N>> astar;
	IDAStar<TOHState<N>, TOHMove> ida;
	MM<TOHState<N>, TOHMove, TOH<N>> mm;
	TOH<N> ts;
	TOHState<N> s;
	TOHState<N> g;
	std::vector<TOHState<N>> thePath;
	std::vector<TOHMove> actionPath;
	ts.StoreGoal(g);
	ZeroHeuristic<TOHState<N>> z;
	
	int table[] = {52058078,116173544,208694125,131936966,141559500,133800745,194246206,50028346,167007978,207116816,163867037,119897198,201847476,210859515,117688410,121633885};
	int table2[] = {145008714,165971878,154717942,218927374,182772845,5808407,19155194,137438954,13143598,124513215,132635260,39667704,2462244,41006424,214146208,54305743};
	for (int count = first; count < last; count++)
	{
		printf("Seed: %d\n", table[count&0xF]^table2[(count>>4)&0xF]);
		srandom(table[count&0xF]^table2[(count>>4)&0xF]);
		for (int x = 0; x < 20000; x++)
		{
			ts.GetActions(s, actionPath);
			ts.ApplyAction(s, actionPath[random()%actionPath.size()]);
		}
		Timer timer;
		
		if (1)
		{
			printf("-=-=-A*-=-=-\n");
			astar.SetUseBPMX(1);
			astar.SetHeuristic(f);
			timer.StartTimer();
			astar.GetPath(&ts, s, g, thePath);
			timer.EndTimer();
			printf("%llu nodes expanded\n", astar.GetNodesExpanded());
			printf("Solution path length %1.0f\n", ts.GetPathLength(thePath));
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
		}
		
		if (0)
		{
			printf("-=-=-IDA*-=-=-\n");
			ida.SetHeuristic(f);
			ida.SetUseBDPathMax(true);
			timer.StartTimer();
			ida.GetPath(&ts, s, g, actionPath);
			timer.EndTimer();
			printf("%llu nodes expanded; %llu generated\n", ida.GetNodesExpanded(), ida.GetNodesTouched());
			printf("Solution path length %lu\n", actionPath.size());
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
			
			//			printf("Dynamic distribution\n");
			//			for (int x = 0; x < 255; x++)
			//				if (f->histogram[x] != 0)
			//					printf("%d\t%llu\n", x, f->histogram[x]);
		}
		
	}
	//mm.PrintHDist();
	
	//	exit(0);
}


void TOHTest(int deltaFactor, int bits, int factor, int first, int last)
{
	printf("--== TOH Test ==--\n");
	printf("%d delta factor; %d min factor; %d bits\n", deltaFactor, factor, bits);
	const int numDisks = 16; // [disks - 2] (4^14 - 256 million)
	TOH<numDisks> toh;
	TOHState<numDisks> s, g;

	TOHState<numDisks> goal;
	TOH<numDisks-2> absToh1;
//	TOH<numDisks-4> absToh4;
	TOHState<numDisks-2> absTohState1;
	TOHPDB<numDisks-2, numDisks> pdb1(&absToh1);
	TOHPDB<numDisks-2, numDisks> pdb1a(&absToh1);
//	TOHPDB<numDisks-2, numDisks> pdb1b(&absToh1);
//	TOHPDB<numDisks-2, numDisks> pdbSmaller(&absToh1);
//	TOHPDB<numDisks-4, numDisks> pdbSmallest(&absToh4);

	goal.Reset();
	pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
	pdb1a = pdb1;
	
	printf("Starting TOH DIV\n");

	printf("Div compress (%d) to get delta:\n", deltaFactor);
	pdb1a.DivCompress(deltaFactor, true);
	
	printf("Delta compress:\n");
	pdb1.DeltaCompress(&pdb1a, goal, true);
	printf("Div compress (%d) delta:\n", factor);
	pdb1.DivCompress(factor, true);

	if (bits < 8)
	{
		printf("VR compress (%d bits):\n", bits);
		pdb1.ValueRangeCompress(bits, true);
	}
	Heuristic<TOHState<numDisks>> h;
	
	h.lookups.resize(0);
	h.lookups.push_back({kAddNode, 1, 2});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.heuristics.resize(0);
	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb1a);
	TestTOH<numDisks>(&h, first, last);
	
	printf("Dynamic distribution\n");
	for (int x = 0; x < 255; x++)
		if (h.histogram[x] != 0)
			printf("%d\t%llu\n", x, h.histogram[x]);

	
//	printf("Starting TOH MOD\n");
//	{
//		pdb1 = pdb1b;
//		pdbSmaller = pdb1;
//		pdbSmaller.ModCompress(pdb1.GetPDBSize()/16, true);
//
//		pdb1.PrintHistogram();
//		for (int compression = 2; compression <= 16; compression++)
//		{
//			printf("[toh][MOD] Compressing by a factor of %d\n", compression);
//			pdb1a = pdb1;
//			pdb1a.ModCompress(pdb1.GetPDBSize()/compression, true);
//		}
//		printf("Performing delta compression. New distribution:\n");
//		pdb1.DeltaCompress(&pdbSmaller, goal, true);
//		for (int compression = 2; compression <= 16; compression++)
//		{
//			printf("[toh][MOD] Compressing by a factor of %d\n", compression);
//			pdb1a = pdb1;
//			pdb1a.ModCompress(pdb1.GetPDBSize()/compression, true);
//		}
//	}
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


void RubikDynamicTest()
{
	RubiksCube cube;
	std::vector<int> blank;
	RubiksState start, goal;
	goal.Reset();
	IDAStar<RubiksState, RubiksAction> ida;
	Timer timer;
	std::vector<int> edges = {0, 1, 2, 3, 4, 5, 6};
	RubikPDB pdb1(&cube, goal, edges, blank);
	RubikPDB pdb2(&cube, goal, edges, blank);
	RubikPDB pdb3(&cube, goal, edges, blank);
	std::vector<RubiksAction> rubikPath;
	
	if (!pdb1.Load(prefix))
	{
		pdb1.BuildPDB(goal, std::thread::hardware_concurrency());
		pdb1.Save(prefix);
	}
	else {
		pdb1.PrintHistogram();
	}
	if (!pdb2.Load(prefix))
	{
		pdb2.BuildPDB(goal, std::thread::hardware_concurrency());
		pdb2.Save(prefix);
	}
	else {
		pdb2.PrintHistogram();
	}
	if (!pdb3.Load(prefix))
	{
		pdb3.BuildPDB(goal, std::thread::hardware_concurrency());
		pdb3.Save(prefix);
	}
	else {
		pdb3.PrintHistogram();
	}
	pdb1.DivCompress(2, true);
	pdb2.ValueRangeCompress(2, true);
	std::vector<uint64_t> dist = {324941267,1436796583,1486721207,1444732064,1384553281,1253754769,1034699573,712569268,335643524,63842841,375040};
	pdb3.CustomValueRangeCompress(dist, 2, true);
	
	Heuristic<RubiksState> h;
	h.lookups.push_back({kLeafNode, 0, 0});
	h.heuristics.push_back(&pdb1);
	
	cube.SetPruneSuccessors(true);

	printf("-=-=-IDA*-=-=-\n");
	ida.SetHeuristic(&h);
	ida.SetUseBDPathMax(true);
	
//	printf("-=-=-DIV2-=-=-\n");
//	h.heuristics.back() = &pdb1;
//	for (int x = 0; x < 100; x++)
//	{
//		GetRubikStep14Instance(start, x);
//		timer.StartTimer();
//		goal.Reset();
//		ida.GetPath(&cube, start, goal, rubikPath);
//		timer.EndTimer();
//		printf("%llu nodes expanded; %llu generated\n", ida.GetNodesExpanded(), ida.GetNodesTouched());
//		printf("Solution path length %lu\n", rubikPath.size());
//		printf("%1.2f elapsed\n", timer.GetElapsedTime());
//	}
//
	printf("-=-=-VRC2-REG-=-=-\n");
	h.heuristics.back() = &pdb2;
	for (int x = 0; x < 100; x++)
	{
		GetRubikStep14Instance(start, x);
		timer.StartTimer();
		goal.Reset();
		ida.GetPath(&cube, start, goal, rubikPath);
		timer.EndTimer();
		printf("%llu nodes expanded; %llu generated\n", ida.GetNodesExpanded(), ida.GetNodesTouched());
		printf("Solution path length %lu\n", rubikPath.size());
		printf("%1.2f elapsed\n", timer.GetElapsedTime());
	}

//	printf("-=-=-VRC2-CUST-=-=-\n");
//	h.heuristics.back() = &pdb3;
//	for (int x = 0; x < 100; x++)
//	{
//		GetRubikStep14Instance(start, x);
//		timer.StartTimer();
//		goal.Reset();
//		ida.GetPath(&cube, start, goal, rubikPath);
//		timer.EndTimer();
//		printf("%llu nodes expanded; %llu generated\n", ida.GetNodesExpanded(), ida.GetNodesTouched());
//		printf("Solution path length %lu\n", rubikPath.size());
//		printf("%1.2f elapsed\n", timer.GetElapsedTime());
//	}

//	printf("Dynamic distribution\n");
//	for (int x = 0; x < 255; x++)
//		if (h.histogram[x] != 0)
//			printf("%d\t%llu\n", x, h.histogram[x]);
}


void GetRubikLength14Instance(RubiksState &start, int which)
{
	RubiksCube c;
	int instances[100][14] = {
		{4, 10, 5, 10, 17, 6, 1, 11, 0, 10, 16, 9, 14, 4},
		{15, 0, 12, 16, 1, 6, 17, 8, 17, 8, 2, 11, 4, 15},
		{6, 12, 2, 16, 9, 13, 8, 2, 15, 11, 4, 8, 4, 16},
		{9, 13, 16, 1, 14, 0, 11, 13, 4, 10, 12, 6, 5, 15},
		{1, 9, 4, 6, 0, 4, 15, 5, 12, 0, 14, 9, 4, 16},
		{5, 8, 4, 8, 5, 9, 5, 11, 0, 7, 11, 12, 8, 17},
		{14, 11, 4, 14, 5, 6, 4, 10, 1, 4, 15, 9, 13, 7},
		{17, 1, 4, 10, 4, 16, 4, 16, 8, 13, 15, 2, 4, 17},
		{7, 12, 11, 4, 15, 1, 17, 2, 5, 14, 9, 5, 8, 10},
		{10, 13, 15, 1, 16, 3, 7, 9, 17, 4, 9, 17, 6, 15},
		{0, 5, 10, 16, 8, 14, 7, 2, 6, 1, 4, 12, 8, 13},
		{4, 6, 5, 17, 7, 4, 17, 9, 17, 3, 9, 3, 17, 3},
		{15, 7, 13, 17, 11, 3, 10, 4, 12, 10, 0, 16, 2, 9},
		{6, 10, 16, 2, 4, 7, 0, 14, 16, 9, 2, 17, 2, 9},
		{9, 16, 0, 5, 16, 7, 1, 6, 0, 3, 10, 2, 12, 11},
		{1, 7, 3, 10, 2, 10, 2, 8, 4, 11, 13, 17, 5, 7},
		{5, 11, 16, 4, 9, 13, 4, 15, 8, 15, 0, 11, 12, 7},
		{14, 17, 11, 4, 8, 3, 10, 4, 10, 0, 5, 7, 13, 3},
		{17, 8, 16, 11, 14, 5, 13, 2, 16, 10, 3, 12, 0, 5},
		{8, 9, 2, 3, 11, 15, 0, 17, 9, 2, 13, 10, 3, 17},
		{10, 17, 2, 17, 10, 2, 11, 16, 8, 16, 10, 5, 16, 1},
		{0, 6, 17, 1, 15, 9, 4, 15, 3, 14, 1, 11, 1, 3},
		{4, 7, 17, 9, 17, 9, 0, 9, 17, 3, 13, 5, 15, 11},
		{13, 15, 1, 5, 16, 6, 15, 6, 1, 10, 2, 14, 3, 7},
		{6, 16, 1, 16, 3, 9, 1, 13, 8, 12, 7, 4, 9, 14},
		{9, 13, 2, 5, 16, 5, 9, 17, 7, 4, 6, 9, 14, 3},
		{1, 13, 11, 5, 15, 3, 7, 12, 5, 7, 12, 5, 14, 0},
		{5, 17, 10, 4, 9, 4, 13, 1, 5, 6, 0, 6, 3, 17},
		{14, 8, 10, 13, 11, 12, 6, 11, 0, 5, 12, 0, 14, 1},
		{17, 10, 0, 11, 16, 9, 5, 6, 17, 8, 3, 11, 13, 5},
		{8, 17, 10, 15, 4, 8, 0, 15, 1, 16, 10, 13, 15, 6},
		{10, 14, 15, 6, 13, 5, 13, 0, 3, 11, 5, 7, 5, 6},
		{0, 12, 11, 17, 9, 1, 11, 16, 7, 13, 5, 14, 17, 5},
		{4, 15, 11, 3, 17, 7, 13, 6, 17, 7, 12, 0, 5, 6},
		{13, 16, 2, 3, 11, 14, 2, 11, 17, 7, 2, 13, 8, 1},
		{6, 12, 3, 14, 8, 5, 12, 5, 17, 4, 14, 1, 12, 17},
		{9, 13, 7, 9, 5, 7, 2, 3, 6, 2, 15, 2, 13, 1},
		{1, 17, 7, 9, 14, 0, 3, 11, 15, 10, 0, 4, 15, 5},
		{3, 9, 2, 17, 5, 10, 17, 10, 1, 12, 9, 3, 8, 4},
		{14, 4, 15, 11, 5, 15, 11, 0, 14, 10, 1, 10, 12, 11},
		{17, 4, 11, 13, 4, 14, 9, 13, 11, 17, 1, 6, 16, 0},
		{8, 5, 8, 15, 6, 4, 15, 11, 14, 7, 4, 7, 4, 9},
		{10, 14, 8, 3, 15, 6, 5, 17, 3, 12, 11, 16, 6, 2},
		{0, 15, 3, 17, 0, 9, 3, 14, 16, 2, 9, 14, 0, 4},
		{4, 16, 5, 7, 2, 9, 4, 12, 6, 17, 2, 13, 7, 14},
		{13, 6, 9, 3, 16, 8, 9, 13, 15, 4, 12, 0, 15, 3},
		{6, 13, 6, 5, 7, 4, 11, 15, 10, 13, 3, 8, 10, 2},
		{9, 16, 0, 6, 1, 9, 12, 7, 9, 5, 16, 5, 15, 4},
		{1, 4, 9, 3, 8, 4, 8, 4, 6, 14, 5, 14, 16, 3},
		{3, 14, 17, 11, 13, 4, 8, 15, 6, 15, 8, 4, 10, 2},
		{14, 17, 0, 13, 9, 3, 17, 10, 3, 8, 3, 17, 3, 16},
		{17, 5, 8, 15, 2, 9, 3, 17, 6, 17, 0, 4, 13, 4},
		{8, 14, 2, 17, 7, 10, 5, 13, 0, 9, 16, 7, 17, 0},
		{11, 15, 2, 11, 17, 11, 4, 17, 1, 11, 12, 3, 14, 1},
		{0, 3, 15, 0, 7, 10, 3, 12, 6, 17, 9, 0, 7, 1},
		{4, 12, 15, 7, 3, 7, 15, 7, 13, 3, 15, 11, 5, 10},
		{13, 1, 6, 11, 16, 3, 14, 5, 16, 1, 9, 0, 16, 0},
		{6, 3, 7, 2, 15, 6, 2, 15, 6, 11, 17, 7, 0, 16},
		{9, 4, 13, 9, 5, 13, 11, 17, 9, 15, 11, 3, 14, 9},
		{1, 14, 9, 14, 4, 9, 1, 16, 10, 16, 11, 13, 17, 1},
		{3, 11, 13, 15, 9, 16, 8, 13, 10, 15, 0, 11, 5, 15},
		{14, 5, 11, 0, 13, 6, 1, 11, 3, 9, 14, 8, 10, 12},
		{17, 2, 9, 14, 11, 12, 10, 4, 9, 14, 0, 5, 17, 6},
		{8, 2, 14, 10, 0, 17, 7, 5, 8, 12, 7, 12, 10, 3},
		{11, 3, 14, 7, 0, 6, 13, 0, 6, 15, 5, 7, 3, 12},
		{0, 12, 9, 3, 8, 9, 14, 15, 0, 4, 6, 5, 10, 17},
		{4, 9, 16, 8, 1, 4, 12, 4, 15, 2, 4, 6, 3, 15},
		{13, 3, 12, 1, 7, 0, 5, 8, 17, 10, 17, 5, 9, 17},
		{16, 4, 12, 9, 4, 13, 9, 1, 8, 2, 3, 12, 10, 0},
		{9, 1, 7, 4, 14, 2, 13, 8, 4, 13, 5, 11, 13, 10},
		{1, 7, 13, 16, 10, 16, 8, 5, 11, 0, 14, 8, 5, 7},
		{3, 15, 8, 13, 3, 10, 3, 12, 16, 11, 17, 0, 5, 12},
		{14, 10, 5, 10, 2, 13, 17, 7, 16, 7, 0, 3, 8, 0},
		{17, 2, 8, 12, 2, 16, 5, 10, 0, 17, 5, 16, 10, 3},
		{8, 5, 8, 0, 7, 3, 12, 10, 12, 6, 17, 3, 16, 7},
		{11, 17, 6, 11, 2, 6, 4, 14, 0, 11, 17, 0, 6, 13},
		{0, 3, 8, 2, 9, 17, 5, 7, 12, 8, 0, 14, 4, 16},
		{4, 14, 7, 5, 11, 13, 4, 6, 1, 15, 1, 13, 1, 7},
		{13, 9, 16, 7, 12, 9, 4, 12, 8, 15, 1, 4, 11, 17},
		{16, 1, 6, 14, 15, 8, 16, 4, 11, 13, 2, 7, 1, 3},
		{9, 0, 14, 5, 6, 1, 15, 2, 12, 4, 7, 17, 10, 16},
		{1, 7, 0, 13, 1, 14, 0, 14, 7, 13, 9, 0, 6, 1},
		{3, 10, 17, 11, 1, 15, 2, 8, 11, 16, 5, 14, 16, 7},
		{12, 2, 17, 8, 2, 6, 10, 0, 12, 6, 12, 0, 11, 12},
		{17, 7, 1, 6, 9, 15, 9, 5, 9, 14, 15, 7, 10, 3},
		{8, 11, 2, 17, 6, 1, 7, 11, 3, 6, 10, 16, 3, 14},
		{11, 1, 14, 8, 0, 6, 5, 8, 2, 4, 14, 9, 3, 17},
		{2, 8, 15, 7, 4, 16, 8, 5, 16, 0, 10, 13, 2, 6},
		{4, 9, 15, 5, 12, 1, 15, 1, 5, 10, 5, 13, 5, 9},
		{13, 1, 10, 16, 3, 13, 9, 15, 0, 14, 3, 6, 16, 8},
		{16, 6, 10, 13, 4, 12, 3, 14, 11, 15, 0, 9, 12, 17},
		{9, 13, 9, 1, 13, 4, 17, 6, 4, 16, 4, 14, 1, 10},
		{1, 10, 13, 4, 16, 8, 10, 17, 5, 13, 8, 0, 15, 9},
		{3, 16, 11, 2, 8, 16, 4, 9, 5, 16, 11, 16, 5, 12},
		{12, 7, 11, 15, 11, 17, 6, 0, 16, 0, 17, 9, 15, 4},
		{17, 11, 12, 10, 2, 13, 2, 7, 2, 12, 4, 16, 0, 7},
		{8, 17, 8, 5, 14, 11, 14, 17, 8, 11, 1, 12, 7, 4},
		{11, 14, 8, 4, 11, 4, 7, 5, 9, 12, 10, 1, 14, 16},
		{2, 9, 16, 2, 5, 14, 10, 16, 2, 11, 0, 5, 6, 2},
		{4, 17, 9, 1, 12, 5, 17, 8, 11, 13, 15, 6, 12, 4}
	};
	
	start.Reset();
	for (int x = 13; x >= 0; x--)
	{
		c.UndoAction(start, instances[which][x]);
		printf("%d ", instances[which][x]);
	}
	printf("\n");
}

void GetRubikStep14Instance(RubiksState &start, int which)
{
	RubiksCube c;
	int table[] = {52058078,116173544,208694125,131936966,141559500,133800745,194246206,50028346,167007978,207116816,163867037,119897198,201847476,210859515,117688410,121633885};
	int table2[] = {145008714,165971878,154717942,218927374,182772845,5808407,19155194,137438954,13143598,124513215,132635260,39667704,2462244,41006424,214146208,54305743};
	int first = 0, last = 50;
	srandom(table[which&0xF]^table2[(which>>4)&0xF]);
	
	start.Reset();
	for (int x = 0; x < 14; x++)
	{
		c.ApplyAction(start, random()%18);
	}
}

MNPuzzleState<4, 4> GetSTPInstance(int which);
void STPTest(int bits, int factor)
{
	printf("Running with %d bits, %d compression\n", bits, factor);
	MNPuzzle<4, 4> mnp;
	MNPuzzleState<4, 4> s;
	MNPuzzleState<4, 4> g;

	std::vector<int> p1 = {0,1,2,3,4,5,6,7};
	std::vector<int> p2 = {0,8,9,10,11,12,13,14,15};
	std::vector<int> p3 = {0,1,4,5,8,9,12,13};
	std::vector<int> p4 = {0,2,3,6,7,10,11,14,15};
	mnp.StoreGoal(g);
	
	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb1(&mnp, g, p1);
	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb2(&mnp, g, p2);
//	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb3(&mnp, g, p3);
//	LexPermutationPDB<MNPuzzleState<4, 4>, slideDir, MNPuzzle<4, 4>> pdb4(&mnp, g, p4);
	if (pdb1.Load(prefix) == false)
	{
		pdb1.BuildPDB(t, std::thread::hardware_concurrency());
		pdb1.Save(prefix);
	}
	else {
		pdb1.PrintHistogram();
	}
	
	if (pdb2.Load(prefix) == false)
	{
		pdb2.BuildPDB(t, std::thread::hardware_concurrency());
		pdb2.Save(prefix);
	}
	else {
		pdb2.PrintHistogram();
	}

//	if (pdb3.Load(prefix) == false)
//	{
//		pdb3.BuildPDB(t, std::thread::hardware_concurrency());
//		pdb3.Save(prefix);
//	}
//	else {
//		pdb3.PrintHistogram();
//	}
//
//	if (pdb4.Load(prefix) == false)
//	{
//		pdb4.BuildPDB(t, std::thread::hardware_concurrency());
//		pdb4.Save(prefix);
//	}
//	else {
//		pdb4.PrintHistogram();
//	}

	Heuristic<MNPuzzleState<4, 4>> h;
	
	h.lookups.resize(0);
	h.lookups.push_back({kMaxNode, 1, 3});
	h.lookups.push_back({kLeafNode, 0, 0});
	h.lookups.push_back({kLeafNode, 1, 1});
	h.lookups.push_back({kLeafNode, 1, 2});
//	h.lookups.push_back({kLeafNode, 1, 3});
	h.heuristics.resize(0);
	h.heuristics.push_back(&pdb1);
	h.heuristics.push_back(&pdb2);
	h.heuristics.push_back(&mnp);
//	h.heuristics.push_back(&pdb3);
//	h.heuristics.push_back(&pdb4);
	if (factor > 1)
	{
		pdb1.DivCompress(factor, true);
		pdb2.DivCompress(factor, true);
	}
//	pdb3.DivCompress(factor, true);
//	pdb4.DivCompress(factor, true);
	if (bits < 8)
	{
		pdb1.ValueRangeCompress(bits, true);
		pdb2.ValueRangeCompress(bits, true);
	}
//	pdb3.ValueRangeCompress(bits, true);
//	pdb4.ValueRangeCompress(bits, true);
	g.Reset();
	
	{
		IDAStar<MNPuzzleState<4, 4>, slideDir> ida;
		ida.SetUseBDPathMax(true);
		ida.SetHeuristic(&h);
		std::vector<slideDir> path1;
		MNPuzzleState<4, 4> start;
		Timer t1;
		t1.StartTimer();
		uint64_t nodesExpanded = 0;
		uint64_t nodesGenerated = 0;
		double totaltime = 0;
		
		
		
		g.Reset();
		mnp.StoreGoal(g);
		for (int x = 0; x < 100; x++)
		{
			s = GetSTPInstance(x);
			g.Reset();
			printf("Problem %d of %d\n", x+1, 100);
			std::cout << "Searching from: " << std::endl << s << std::endl << g << std::endl;
			Timer timer;
			timer.StartTimer();
			ida.GetPath(&mnp, s, g, path1);
			timer.EndTimer();
			totaltime += timer.GetElapsedTime();
			printf("%llu nodes expanded; %llu generated\n", ida.GetNodesExpanded(), ida.GetNodesTouched());
			printf("Solution path length %lu\n", path1.size());
			printf("%1.2f elapsed\n", timer.GetElapsedTime());
			nodesExpanded += ida.GetNodesExpanded();
			nodesGenerated += ida.GetNodesTouched();
		}
		printf("Sequential: %1.2fs elapsed; %llu nodes expanded; %llu nodes generated\n", t1.EndTimer(), nodesExpanded, nodesGenerated);
	}
}

MNPuzzleState<4, 4> GetSTPInstance(int which)
{
	int instances[100][16] =
	{{14, 13, 15, 7, 11, 12, 9, 5, 6, 0, 2, 1, 4, 8, 10, 3},
		{13, 5, 4, 10, 9, 12, 8, 14, 2, 3, 7, 1, 0, 15, 11, 6},
		{14, 7, 8, 2, 13, 11, 10, 4, 9, 12, 5, 0, 3, 6, 1, 15},
		{5, 12, 10, 7, 15, 11, 14, 0, 8, 2, 1, 13, 3, 4, 9, 6},
		{4, 7, 14, 13, 10, 3, 9, 12, 11, 5, 6, 15, 1, 2, 8, 0},
		{14, 7, 1, 9, 12, 3, 6, 15, 8, 11, 2, 5, 10, 0, 4, 13},
		{2, 11, 15, 5, 13, 4, 6, 7, 12, 8, 10, 1, 9, 3, 14, 0},
		{12, 11, 15, 3, 8, 0, 4, 2, 6, 13, 9, 5, 14, 1, 10, 7},
		{3, 14, 9, 11, 5, 4, 8, 2, 13, 12, 6, 7, 10, 1, 15, 0},
		{13, 11, 8, 9, 0, 15, 7, 10, 4, 3, 6, 14, 5, 12, 2, 1},
		{5, 9, 13, 14, 6, 3, 7, 12, 10, 8, 4, 0, 15, 2, 11, 1},
		{14, 1, 9, 6, 4, 8, 12, 5, 7, 2, 3, 0, 10, 11, 13, 15},
		{3, 6, 5, 2, 10, 0, 15, 14, 1, 4, 13, 12, 9, 8, 11, 7},
		{7, 6, 8, 1, 11, 5, 14, 10, 3, 4, 9, 13, 15, 2, 0, 12},
		{13, 11, 4, 12, 1, 8, 9, 15, 6, 5, 14, 2, 7, 3, 10, 0},
		{1, 3, 2, 5, 10, 9, 15, 6, 8, 14, 13, 11, 12, 4, 7, 0},
		{15, 14, 0, 4, 11, 1, 6, 13, 7, 5, 8, 9, 3, 2, 10, 12},
		{6, 0, 14, 12, 1, 15, 9, 10, 11, 4, 7, 2, 8, 3, 5, 13},
		{7, 11, 8, 3, 14, 0, 6, 15, 1, 4, 13, 9, 5, 12, 2, 10},
		{6, 12, 11, 3, 13, 7, 9, 15, 2, 14, 8, 10, 4, 1, 5, 0},
		{12, 8, 14, 6, 11, 4, 7, 0, 5, 1, 10, 15, 3, 13, 9, 2},
		{14, 3, 9, 1, 15, 8, 4, 5, 11, 7, 10, 13, 0, 2, 12, 6},
		{10, 9, 3, 11, 0, 13, 2, 14, 5, 6, 4, 7, 8, 15, 1, 12},
		{7, 3, 14, 13, 4, 1, 10, 8, 5, 12, 9, 11, 2, 15, 6, 0},
		{11, 4, 2, 7, 1, 0, 10, 15, 6, 9, 14, 8, 3, 13, 5, 12},
		{5, 7, 3, 12, 15, 13, 14, 8, 0, 10, 9, 6, 1, 4, 2, 11},
		{14, 1, 8, 15, 2, 6, 0, 3, 9, 12, 10, 13, 4, 7, 5, 11},
		{13, 14, 6, 12, 4, 5, 1, 0, 9, 3, 10, 2, 15, 11, 8, 7},
		{9, 8, 0, 2, 15, 1, 4, 14, 3, 10, 7, 5, 11, 13, 6, 12},
		{12, 15, 2, 6, 1, 14, 4, 8, 5, 3, 7, 0, 10, 13, 9, 11},
		{12, 8, 15, 13, 1, 0, 5, 4, 6, 3, 2, 11, 9, 7, 14, 10},
		{14, 10, 9, 4, 13, 6, 5, 8, 2, 12, 7, 0, 1, 3, 11, 15},
		{14, 3, 5, 15, 11, 6, 13, 9, 0, 10, 2, 12, 4, 1, 7, 8},
		{6, 11, 7, 8, 13, 2, 5, 4, 1, 10, 3, 9, 14, 0, 12, 15},
		{1, 6, 12, 14, 3, 2, 15, 8, 4, 5, 13, 9, 0, 7, 11, 10},
		{12, 6, 0, 4, 7, 3, 15, 1, 13, 9, 8, 11, 2, 14, 5, 10},
		{8, 1, 7, 12, 11, 0, 10, 5, 9, 15, 6, 13, 14, 2, 3, 4},
		{7, 15, 8, 2, 13, 6, 3, 12, 11, 0, 4, 10, 9, 5, 1, 14},
		{9, 0, 4, 10, 1, 14, 15, 3, 12, 6, 5, 7, 11, 13, 8, 2},
		{11, 5, 1, 14, 4, 12, 10, 0, 2, 7, 13, 3, 9, 15, 6, 8},
		{8, 13, 10, 9, 11, 3, 15, 6, 0, 1, 2, 14, 12, 5, 4, 7},
		{4, 5, 7, 2, 9, 14, 12, 13, 0, 3, 6, 11, 8, 1, 15, 10},
		{11, 15, 14, 13, 1, 9, 10, 4, 3, 6, 2, 12, 7, 5, 8, 0},
		{12, 9, 0, 6, 8, 3, 5, 14, 2, 4, 11, 7, 10, 1, 15, 13},
		{3, 14, 9, 7, 12, 15, 0, 4, 1, 8, 5, 6, 11, 10, 2, 13},
		{8, 4, 6, 1, 14, 12, 2, 15, 13, 10, 9, 5, 3, 7, 0, 11},
		{6, 10, 1, 14, 15, 8, 3, 5, 13, 0, 2, 7, 4, 9, 11, 12},
		{8, 11, 4, 6, 7, 3, 10, 9, 2, 12, 15, 13, 0, 1, 5, 14},
		{10, 0, 2, 4, 5, 1, 6, 12, 11, 13, 9, 7, 15, 3, 14, 8},
		{12, 5, 13, 11, 2, 10, 0, 9, 7, 8, 4, 3, 14, 6, 15, 1},
		{10, 2, 8, 4, 15, 0, 1, 14, 11, 13, 3, 6, 9, 7, 5, 12},
		{10, 8, 0, 12, 3, 7, 6, 2, 1, 14, 4, 11, 15, 13, 9, 5},
		{14, 9, 12, 13, 15, 4, 8, 10, 0, 2, 1, 7, 3, 11, 5, 6},
		{12, 11, 0, 8, 10, 2, 13, 15, 5, 4, 7, 3, 6, 9, 14, 1},
		{13, 8, 14, 3, 9, 1, 0, 7, 15, 5, 4, 10, 12, 2, 6, 11},
		{3, 15, 2, 5, 11, 6, 4, 7, 12, 9, 1, 0, 13, 14, 10, 8},
		{5, 11, 6, 9, 4, 13, 12, 0, 8, 2, 15, 10, 1, 7, 3, 14},
		{5, 0, 15, 8, 4, 6, 1, 14, 10, 11, 3, 9, 7, 12, 2, 13},
		{15, 14, 6, 7, 10, 1, 0, 11, 12, 8, 4, 9, 2, 5, 13, 3},
		{11, 14, 13, 1, 2, 3, 12, 4, 15, 7, 9, 5, 10, 6, 8, 0},
		{6, 13, 3, 2, 11, 9, 5, 10, 1, 7, 12, 14, 8, 4, 0, 15},
		{4, 6, 12, 0, 14, 2, 9, 13, 11, 8, 3, 15, 7, 10, 1, 5},
		{8, 10, 9, 11, 14, 1, 7, 15, 13, 4, 0, 12, 6, 2, 5, 3},
		{5, 2, 14, 0, 7, 8, 6, 3, 11, 12, 13, 15, 4, 10, 9, 1},
		{7, 8, 3, 2, 10, 12, 4, 6, 11, 13, 5, 15, 0, 1, 9, 14},
		{11, 6, 14, 12, 3, 5, 1, 15, 8, 0, 10, 13, 9, 7, 4, 2},
		{7, 1, 2, 4, 8, 3, 6, 11, 10, 15, 0, 5, 14, 12, 13, 9},
		{7, 3, 1, 13, 12, 10, 5, 2, 8, 0, 6, 11, 14, 15, 4, 9},
		{6, 0, 5, 15, 1, 14, 4, 9, 2, 13, 8, 10, 11, 12, 7, 3},
		{15, 1, 3, 12, 4, 0, 6, 5, 2, 8, 14, 9, 13, 10, 7, 11},
		{5, 7, 0, 11, 12, 1, 9, 10, 15, 6, 2, 3, 8, 4, 13, 14},
		{12, 15, 11, 10, 4, 5, 14, 0, 13, 7, 1, 2, 9, 8, 3, 6},
		{6, 14, 10, 5, 15, 8, 7, 1, 3, 4, 2, 0, 12, 9, 11, 13},
		{14, 13, 4, 11, 15, 8, 6, 9, 0, 7, 3, 1, 2, 10, 12, 5},
		{14, 4, 0, 10, 6, 5, 1, 3, 9, 2, 13, 15, 12, 7, 8, 11},
		{15, 10, 8, 3, 0, 6, 9, 5, 1, 14, 13, 11, 7, 2, 12, 4},
		{0, 13, 2, 4, 12, 14, 6, 9, 15, 1, 10, 3, 11, 5, 8, 7},
		{3, 14, 13, 6, 4, 15, 8, 9, 5, 12, 10, 0, 2, 7, 1, 11},
		{0, 1, 9, 7, 11, 13, 5, 3, 14, 12, 4, 2, 8, 6, 10, 15},
		{11, 0, 15, 8, 13, 12, 3, 5, 10, 1, 4, 6, 14, 9, 7, 2},
		{13, 0, 9, 12, 11, 6, 3, 5, 15, 8, 1, 10, 4, 14, 2, 7},
		{14, 10, 2, 1, 13, 9, 8, 11, 7, 3, 6, 12, 15, 5, 4, 0},
		{12, 3, 9, 1, 4, 5, 10, 2, 6, 11, 15, 0, 14, 7, 13, 8},
		{15, 8, 10, 7, 0, 12, 14, 1, 5, 9, 6, 3, 13, 11, 4, 2},
		{4, 7, 13, 10, 1, 2, 9, 6, 12, 8, 14, 5, 3, 0, 11, 15},
		{6, 0, 5, 10, 11, 12, 9, 2, 1, 7, 4, 3, 14, 8, 13, 15},
		{9, 5, 11, 10, 13, 0, 2, 1, 8, 6, 14, 12, 4, 7, 3, 15},
		{15, 2, 12, 11, 14, 13, 9, 5, 1, 3, 8, 7, 0, 10, 6, 4},
		{11, 1, 7, 4, 10, 13, 3, 8, 9, 14, 0, 15, 6, 5, 2, 12},
		{5, 4, 7, 1, 11, 12, 14, 15, 10, 13, 8, 6, 2, 0, 9, 3},
		{9, 7, 5, 2, 14, 15, 12, 10, 11, 3, 6, 1, 8, 13, 0, 4},
		{3, 2, 7, 9, 0, 15, 12, 4, 6, 11, 5, 14, 8, 13, 10, 1},
		{13, 9, 14, 6, 12, 8, 1, 2, 3, 4, 0, 7, 5, 10, 11, 15},
		{5, 7, 11, 8, 0, 14, 9, 13, 10, 12, 3, 15, 6, 1, 4, 2},
		{4, 3, 6, 13, 7, 15, 9, 0, 10, 5, 8, 11, 2, 12, 1, 14},
		{1, 7, 15, 14, 2, 6, 4, 9, 12, 11, 13, 3, 0, 8, 5, 10},
		{9, 14, 5, 7, 8, 15, 1, 2, 10, 4, 13, 6, 12, 0, 11, 3},
		{0, 11, 3, 12, 5, 2, 1, 9, 8, 10, 14, 15, 7, 4, 13, 6},
		{7, 15, 4, 0, 10, 9, 2, 5, 12, 11, 13, 6, 1, 3, 14, 8},
		{11, 4, 0, 8, 6, 10, 5, 13, 12, 7, 14, 3, 1, 2, 9, 15}};
	
	MNPuzzleState<4, 4> s;
	for (int x = 0; x < 16; x++)
	{
		s.puzzle[x] = instances[which][x];
		if (s.puzzle[x] == 0)
			s.blank = x;
	}
	return s;
}
