#include "Common.h"
#include "Map2DEnvironment.h"
#include "Graph.h"

void output_syntax();

void MyWindowHandler(unsigned long windowID, tWindowEventType eType);
void MyFrameHandler(unsigned long windowID, unsigned int viewport, void *data);
void MyDisplayHandler(unsigned long windowID, tKeyboardModifier, char key);
int MyCLHandler(char *argument[], int maxNumArgs);
bool MyClickHandler(unsigned long windowID, int x, int y, point3d loc, tButtonType, tMouseEventType);
void InstallHandlers();


void parseCommandLineParameters( int argc, char* argv[], Map* &m, xyLoc &pos_cop, xyLoc &pos_robber, int &max_recursion_level );

Graph *readGraphFromCommandLine( int offset, int argc, char* argv[] );
Graph* readGraph( FILE *fhandler );
void writeGraph( FILE *fhandler, Graph *g );
