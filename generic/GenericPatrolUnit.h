#ifndef GENERICPATROLUNIT_H
#define GENERICPATROLUNIT_H

#include <cstdlib> // for random number
#include "Unit.h"
#include "GenericSearchUnit.h"
#include "ReservationProvider.h"
//#include "SearchEnvironment.h"

template <class state, class action, class environment>
class GenericPatrolUnit : public GenericSearchUnit<state,action, environment> {
public:
	GenericPatrolUnit(state &s, GenericSearchAlgorithm<state,action,environment>* alg);
	GenericPatrolUnit(state &s, GenericSearchAlgorithm<state,action,environment>* alg, GLfloat _r, GLfloat _g, GLfloat _b);
	virtual const char *GetName() { return name; } 
	void SetName(char* myname) { strncpy(name, myname,128); }
	virtual void GetLocation(state& s) { s=loc;}
	virtual bool MakeMove(environment *env, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *si, action &dir);
	
	virtual void OpenGLDraw(const environment *, const SimulationInfo<state,action,environment> *) const;
	void AddPatrolLocation(state &s); 
	state& GetGoal(); // get CURRENT goal? 
	virtual bool Done(); 
	void UpdateLocation(environment *, state &l, bool success, SimulationInfo<state,action,environment> *si);
	void LogStats(StatCollection *stats);
	void LogFinalStats(StatCollection *stats);
	void SetNumPatrols(int num) {numPatrols = num;}
	
	/** Set whether we want to trim the planned path
	 * 
	 * @author Renee Jansen
	 * @date 10/2007
	 */
	void SetTrimPath(bool b) {trimPath = b;}
	
	/** Set the window at which we want to trim the planned path
	* 
	* If trimPath is set to true, this will determine where the path is
	* trimmed
	* 
	* @author Renee Jansen
	* @date 10/2007
	*/
	void SetTrimWindow(double d) {trimWindow = d;}
	
	/** Set whether we want to draw the unit. Default "true".
	*	
	* @author Renee Jansen
	* @date 10/30/2007
	*/
	
	void SetDrawUnit(bool b) {drawUnit = b;}
// 	void SetColor(GLfloat _r, GLfloat _g, GLfloat _b) {Unit::Set		GLFloat _r,_g,_b;
// 		GetColor(_r,_g,_b);
// 		glColor3f(_r,_g,_b);this->r=_r; this->g=_g; this->b=_b;}
	
//private:
private:
	//GLfloat r, g, b;
	state loc;
	int numPatrols;
	int counter;
	action oldDir, oldDirColl;
	double totalDistance;
	double lastFailedMove;
	
	// this used to return path cost, but that wasn't used... 
	/*double*/ void GoToLoc(environment *env, int which);
 	void AddPathToCache(environment *env, std::vector<state> &path);
	std::vector<action> moves;
	std::vector<state> locs;

	GenericSearchAlgorithm<state,action,environment> *algorithm;
	
	int currTarget;
	int nodesExpanded;
	int nodesTouched;
	int numFailedMoves;
	int numDirectionChanges;
	int numDirectionChangesCollisions;
	
	
	std::vector<int> nodesExpandedPatrols;
	std::vector<int> nodesTouchedPatrols;
	std::vector<int> numFailedMovesPatrols;
	std::vector<int> numDirectionChangesPatrols;
	std::vector<int> numDirectionChangesCollisionsPatrols;
	std::vector<double> totalDistancePatrols;
	
	char name[128];
	
	bool trimPath;
	double trimWindow;
	
	bool drawUnit; 
};

template <class state, class action, class environment>
GenericPatrolUnit<state,action,environment>::GenericPatrolUnit(state &s, GenericSearchAlgorithm<state,action,environment>* alg) 
:GenericSearchUnit<state,action,environment>(s,s,alg)
{
	lastFailedMove = -10;
	counter = 0; 
	numPatrols = -1; // infinitely patrol
	
	locs.push_back(s);
	loc = s;
	//setObjectType(kWorldObject);
	currTarget = -1;
	nodesExpanded = nodesTouched = numFailedMoves = numDirectionChanges = 0;
	numDirectionChangesCollisions = 0;
	algorithm = alg;
	
	GLfloat _r,_g,_b;
	_r = (double)rand() / RAND_MAX;
	_g = (double)rand() / RAND_MAX;
	_b = (double)rand() / RAND_MAX;
	this->SetColor(_r,_g,_b);
	strncpy(name, "GenericPatrolUnit",128);
	
	totalDistance = 0; 
	oldDir = oldDirColl;// = kStay;
	
	trimPath = false;
	trimWindow = 5; 
	drawUnit = true;
}

template <class state, class action, class environment>
GenericPatrolUnit<state,action,environment>::GenericPatrolUnit(state &s, GenericSearchAlgorithm<state,action,environment>* alg, GLfloat _r, GLfloat _g, GLfloat _b):GenericSearchUnit<state,action,environment>(s,s,alg)
{
	lastFailedMove = -10;
	counter = 0; 
	numPatrols = -1; // infinitely patrol
	
	locs.push_back(s);
	loc = s;

	currTarget = -1;
	nodesExpanded = nodesTouched = numFailedMoves = numDirectionChanges = 0;
	numDirectionChangesCollisions = 0;
	algorithm = alg;
	
	this->r = _r;
	this->g = _g;
	this->b = _b;
	
	strncpy(name, "GenericPatrolUnit",128);
	
	totalDistance = 0;
	oldDir = oldDirColl = kStay;
	
	trimPath = false;
	trimWindow = 5;
	drawUnit = true;
}

template <class state, class action, class environment>
bool GenericPatrolUnit<state,action,environment>::MakeMove(environment *env, OccupancyInterface<state,action> *oi, SimulationInfo<state,action,environment> *si, action& dir)
{
	if (Done())
	{
 		//printf("Done is true (%d)\n", Done());
		return false;
	}
		
	// Return a stored move
	if (moves.size() > 0)
	{
		//dir = moves.back();
		//moves.pop_back();
		dir = moves.front();
		moves.erase(moves.begin());
		return true;
	}
			
	if (currTarget != -1)
	{
		// update current target and patrol counter if we're at our target location
		if (loc == locs[currTarget])
		{
			currTarget = (currTarget+1)%locs.size();
			if((numPatrols != -1)&&(currTarget == 1))
			{	
				//log stats for this patrol & reset
				nodesExpandedPatrols.push_back(nodesExpanded);
				nodesExpanded = 0;
				numFailedMovesPatrols.push_back(numFailedMoves);
				numFailedMoves = 0; 
				numDirectionChangesPatrols.push_back(numDirectionChanges);
				numDirectionChanges = 0;
				numDirectionChangesCollisionsPatrols.push_back(numDirectionChangesCollisions);
				numDirectionChangesCollisions = 0; 
				totalDistancePatrols.push_back(totalDistance);
				totalDistance = 0; 
				
				counter++;
			}
		}
 		
 		// If we're right beside our goal and it's blocked, make a random move
//		else if(env->GetAction(locs[currTarget], loc) != kTeleport)// && 
//				  //(env->GetAction(locs[currTarget],loc) != kStay))
//		{
//			if(env->GetOccupancyInfo()->GetStateOccupied(locs[currTarget]))
//			{
////				std::cout<<"Random move\n";
//				srand(time(0));
//				std::vector<tDirection> actions;
// 				env->GetActions(loc, actions);
//				dir = actions[(rand())%(actions.size())];
//				return true;
//			}
//		}
 		
 		if((numPatrols == -1 ) || (counter < numPatrols))
	 	{
	 		GoToLoc(env, currTarget);
	 	}
		else
		{
			//std::cout<<"else\n";
 			return false; 
		}
		
		if (moves.size() > 0)
		{
			dir = moves.front();
			moves.erase(moves.begin());
			return true;
		}
	}
	//Stay where you are
	//std::cout<<"at end\n";
	return false;
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action,environment>::GoToLoc(environment *env, int which)
{
	std::vector<state> path; 
	
//	std::cout << "Planning between " << loc << " and " << locs[which] << std::endl;
	//printf("%s ",GetName());
	algorithm->GetPath(env, loc, locs[which], path);
	//	std::cout<<"At ("<<loc.x<<","<<loc.y<<") pathing to ("<<locs[which].x<<","<<locs[which].y<<")"<<std::endl;
	nodesExpanded += algorithm->GetNodesExpanded();
	nodesTouched += algorithm->GetNodesTouched();
	std::cout << algorithm->GetNodesExpanded() << " nodes expanded\n";
//	std::cout << "Path length: " << env->GetPathLength(path) << " (" << path.size() << " moves)" << std::endl;
//	std::cout << "Initial Heuristic: " << env->HCost(loc, locs[which]) << std::endl;
//	//  	std::cout<<"Path ";
//	for(int i=0; i<path.size();i++)
//	{
//		std::cout<<"{"<< path[i] << "} ";
//	}
//	std::cout<<std::endl;
	if (path.size() > 0)
	{
		AddPathToCache(env, path);
	}
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action,environment>::AddPatrolLocation(state &s)
{
 currTarget = 1;
 locs.push_back(s);
}
 
template <class state, class action, class environment>
void GenericPatrolUnit<state,action,environment>::AddPathToCache(environment* env, std::vector<state> &p)
{
	for(unsigned int i=0; i<p.size()-1; i++)
	{
		if (trimPath && (env->HCost(p[i+1], loc) > trimWindow))
		break;
		moves.push_back(env->GetAction(p[i], p[i+1]));
	}
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action, environment>::OpenGLDraw(const environment *env, const SimulationInfo<state,action,environment> *si) const
{
	if (drawUnit)
	{
		//printf("Drawing %p at \n", this);
		PublicUnitInfo<state, action, environment> i;
		si->GetPublicUnitInfo(si->GetCurrentUnit(), i);
//		printf("(%f-%f)/(%f-%f)\n", 
//			   si->GetSimulationTime(), i.lastTime, i.nextTime, i.lastTime);
		env->SetColor(1.0, 0.25, 0.25, 1.0);
//		std::cout << si->GetCurrentUnit() << " is at " << i.currentState << std::endl;
		if (fgreater(si->GetSimulationTime(), i.nextTime))
			env->OpenGLDraw(i.currentState);
		else
			env->OpenGLDraw(i.lastState, i.currentState,
							(si->GetSimulationTime()-i.lastTime)/(i.nextTime-i.lastTime));
		algorithm->OpenGLDraw();
	}
// 	if ((0)&&(drawUnit))
//	{
//
//   		state current = loc; 
//   		state next;
//			for(unsigned int i=0; i<moves.size(); i++)
//			{
//				env->OpenGLDraw(current, moves[i]/*,1.0,0,0*/); // draw in red
//				env->GetNextState(current, moves[i], next);
//				current = next;
//			}	
//	}
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action,environment>::UpdateLocation(environment *env, state &l, bool success, SimulationInfo<state,action,environment> *si)
{ 
	if (!success)
	{
		//printf("%s ",GetName());
		//std::cout<<"Move failed\n";
		lastFailedMove = si->GetSimulationTime();
	}	
	// Occupancy interface stuff should be done in UnitSimulation
	//Update occupancy interface
	//env->GetOccupancyInterface()->SetStateOccupied(loc,false);
	//env->GetOccupancyInterface()->SetStateOccupied(l, true);
	
	//if(!success)
	
	if(!(l==loc))
	{
		action dir = env->GetAction(loc,l);
		
		if (!(dir == oldDir))
		{
			numDirectionChanges++;
			oldDir = dir;
		}
		if (!(dir == oldDirColl))
		{
			numDirectionChangesCollisions++;
			oldDirColl = dir;
		}

		totalDistance += env->HCost(loc, l);
	}
	
	if((!success))//||(l == loc))
	{ 
		numFailedMoves++;
		moves.resize(0); 
		//oldDirColl = kStay;
		//if(currTarget != -1) 
		//	currTarget = 0; 
	} 
	loc = l; 
	
}

template <class state, class action, class environment>
bool GenericPatrolUnit<state,action,environment>::Done()
{
	if (numPatrols == -1)
	{
		return false;
	}
	else if (counter >= numPatrols)
	{
		//printf("Done\n");
		return true;
	}
	return false;
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action, environment>::LogStats(StatCollection *sc)
{

}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action,environment>::LogFinalStats(StatCollection *sc)
{
		//Report per loop
	for(int i=1; (i<=numPatrols)&&(i<nodesExpandedPatrols.size()); i++)
	{
		char num[8];
 		sprintf(num,"%d",i);
 		char* name = new char[256];
  		strcpy(name,GetName());
  		strcat(name,"_");
  		strcat(name, num);
  		
  		sc->AddStat("nodesExpanded", name, (long)(nodesExpandedPatrols[i-1]));
		sc->AddStat("distanceTravelled", name, totalDistancePatrols[i-1]);
		sc->AddStat("directionChanges",name, (long)(numDirectionChangesPatrols[i-1]));
		sc->AddStat("directionChangesCollision",name, (long)(numDirectionChangesCollisionsPatrols[i-1]));
		sc->AddStat("failedMoves",name, (long)(numFailedMovesPatrols[i-1]));
	}

	
	//Want: 
	// * nodesExpanded
	// * distance travelled
	// * failedMoves
	// * change in direction
	
/*	sc->AddStat("distanceTravelled", GetName(), totalDistance);
	sc->AddStat("directionChanges",GetName(), (long)(numDirectionChanges));
	sc->AddStat("nodesExpanded", GetName(), (long)(nodesExpanded));
	sc->AddStat("directionChangesCollision",GetName(), (long)(numDirectionChangesCollisions));
	sc->AddStat("failedMoves",GetName(), (long)(numFailedMoves));*/
	
}

#endif
