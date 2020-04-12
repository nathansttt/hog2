#include <cstdlib>
#include "MNPuzzleRunner.h"
#include "Common.h"


void InstallHandlers();
int MyCLHandler(char *argument[], int maxNumArgs);
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void*);

MNPuzzleRunner runner;

int main(int argc, char* argv[]) {

	InstallHandlers();
  RunHOGGUI(argc, argv, 640, 640);

  return 0;
}

void InstallHandlers() {
  InstallCommandLineHandler(MyCLHandler, "-a", "-a", "Run all tests");
	InstallCommandLineHandler(MyCLHandler, "-bfs", "-bfs", "Runs BFS");
	InstallCommandLineHandler(MyCLHandler, "-dfs", "-dfs", "Runs DFS");
	InstallCommandLineHandler(MyCLHandler, "-p", "-p <start> <end>", "Hash two integers to get a start and end state to path find in");
  InstallCommandLineHandler(MyCLHandler, "-o", "-o <fileanem>", "Name of log file");
  InstallFrameHandler(MyFrameHandler, 0, NULL);
}

void MyFrameHandler(unsigned long windowID, unsigned int viewport, void*) {
  runner.runTests();
  exit(0);
}

int MyCLHandler(char *argument[], int maxNumArgs) {
  if (strcmp(argument[0], "-a") == 0) {
    runner.addAllTests();
  } else if (strcmp(argument[0], "-bfs") == 0) {
    runner.addTest("bfs");
  } else if (strcmp(argument[0], "-dfs") == 0) {
    runner.addTest("dfs");
  } else if (strcmp(argument[0], "-o") == 0) {
    if (maxNumArgs > 1) {
      runner.setLogFile(argument[1]);
      return 2;
    } else {
      std::cout << "failed -o <file>: missing file name\n";
      exit(0);
    }
  } else if (strcmp(argument[0], "-p") == 0) {
    if (maxNumArgs > 2) {
      char* end1 = argument[1];
      char* end2 = argument[2];
      while (*end1 != '\0') {
        ++end1;
      }
      while (*end2 != '\0') {
        ++end2;
      }
      runner.addStates(std::strtoull(argument[1], &end1, 10), std::strtoull(argument[2], &end2, 10));

      return 3;
    } else {
      std::cout << "failed -p <start> <end>; missing hash values\n";
      exit(0);
    }
  }

  return 1;
}
