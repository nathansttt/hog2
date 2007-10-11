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
	virtual void GetLocation(xyLoc& s) { s=loc;}
	virtual bool MakeMove(environment *env, OccupancyInterface<state,action> *, SimulationInfo *si, action &dir);
	
	virtual void OpenGLDraw(int window, environment *, SimulationInfo *);
	void AddPatrolLocation(state &s); 
	state& GetGoal(); // get CURRENT goal? 
	virtual bool Done(); 
	void UpdateLocation(environment *, state &l, bool success, SimulationInfo *si);
	void LogStats(StatCollection *stats);
	void LogFinalStats(StatCollection *stats);
	void SetNumPatrols(int num) {numPatrols = num;}
	
private:
	//GLfloat r, g, b;
	xyLoc loc;
	int numPatrols;
	int counter;
	tDirection oldDir, oldDirColl;
	double totalDistance;
	
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
	
	char name[128];
};

template <class state, class action, class environment>
GenericPatrolUnit<state,action,environment>::GenericPatrolUnit(state &s,GenericSearchAlgorithm<state,action,environment>* alg) 
:GenericSearchUnit<state,action,environment>(s,s,alg)
{
	counter = 0; 
	numPatrols = -1; // infinitely patrol
	
	locs.push_back(s);
	loc = s;
	//setObjectType(kWorldObject);
	currTarget = -1;
	nodesExpanded = nodesTouched = numFailedMoves = numDirectionChanges = 0;
	numDirectionChangesCollisions = 0;
	algorithm = alg;
	
	this->r = (double)rand() / RAND_MAX;
	this->g = (double)rand() / RAND_MAX;
	this->b = (double)rand() / RAND_MAX;

	strncpy(name, "GenericPatrolUnit",128);
	
	totalDistance = 0; 
	oldDir = oldDirColl = kStay;
}

template <class state, class action, class environment>
GenericPatrolUnit<state,action,environment>::GenericPatrolUnit(state &s, GenericSearchAlgorithm<state,action,environment>* alg, GLfloat _r, GLfloat _g, GLfloat _b):GenericSearchUnit<state,action,environment>(s,s,alg)
{
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
}

template <class state, class action, class environment>
bool GenericPatrolUnit<state,action,environment>::MakeMove(environment *env, OccupancyInterface<state,action> *oi, SimulationInfo *si, action& dir)
{
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

		
		// if we're not yet at our current target, then we don't need to update 
		// current target
		//std::cout<<"currTarget "<<currTarget<<" loc "<<loc<<" locs[currtarget] "<<locs[currTarget]<<std::endl;
		if(loc == locs[currTarget])
	 	{
	 		currTarget = (currTarget+1)%locs.size();
	 		if((numPatrols != -1)&&(currTarget == 1))
	 		{	
	 			counter++;
	 			//std::cout<<"Updating counter : "<<counter<<std::endl;
	 		}
 		}
 				// If we're right beside our goal and it's blocked, make a random move
		else if(env->GetAction(locs[currTarget], loc) != kTeleport)
		{
			if(env->GetOccupancyInfo()->GetStateOccupied(locs[currTarget]))
			{
				//std::cout<<"Doing random action\n";
				//std::cout<<"My location: "<<loc<<" my goal: "<<locs[currTarget]<<std::endl;
				srand(time(0));
				std::vector<tDirection> actions;
				env->GetActions(loc, actions);
				dir = actions[(rand())%(actions.size())];
				return true;
			}
		}
 		
 		if((numPatrols == -1 ) || (counter < numPatrols))
	 	{
	 		GoToLoc(env, currTarget);
	 	}
		else
		{
 			return false; // don't move - we're 
//  			

		}
		
		if (moves.size() > 0)
		{
			//dir = moves.back();
			//moves.pop_back();
			dir = moves.front();
			moves.erase(moves.begin());
			return true;
		}
	}
	//Stay where you are
	return false;
}

template <class state, class action, class environment>
/*double*/void GenericPatrolUnit<state,action,environment>::GoToLoc(environment *env, int which)
{
	std::vector<state> path; 

	algorithm->GetPath(env, loc, locs[which],path);
	
	nodesExpanded += algorithm->GetNodesExpanded();
	nodesTouched += algorithm->GetNodesTouched();
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
	// should call superclass implementation / use it somehow...
	 	for(unsigned int i=0; i<p.size()-1; i++)
	 		moves.push_back(env->GetAction(p[i], p[i+1]));
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action, environment>::OpenGLDraw(int window, environment *env, SimulationInfo *si)
{
	if(!Done())
		env->OpenGLDraw(window, loc,this->r,this->g,this->b);	
	else
		env->OpenGLDraw(window, loc, 0,0,0);
		
	for(unsigned int i=0; i<locs.size(); i++)
	{
		state l = locs[i];
		GLdouble xx, yy, zz, rad;
		env->GetMapAbstraction()->GetMap()->getOpenGLCoord(l.x, l.y, xx, yy, zz, rad);
		glColor3f(this->r,this->g,this->b);
		DrawPyramid(xx, yy, zz, 1.1*rad, 0.75*rad);
		//		env->OpenGLDraw(window, locs[i], r, g, b);
	}	
	xyLoc current = loc; 
	xyLoc next;
	  	for(unsigned int i=0; i<moves.size(); i++)
 	{
 		env->OpenGLDraw(window,current, moves[i],1.0,0,0); // draw in red
 		env->GetNextState(current, moves[i],next);
 		current = next;
 	}	
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action,environment>::UpdateLocation(environment *env, state &l, bool success, SimulationInfo *si)
{ 
	// Occupancy interface stuff should be done in UnitSimulation
	//Update occupancy interface
	//env->GetOccupancyInterface()->SetStateOccupied(loc,false);
	//env->GetOccupancyInterface()->SetStateOccupied(l, true);
	
	//if(!success)
	
	if(!(l==loc))
	{
		tDirection dir = env->GetAction(loc,l);
		
// 		if(totalDistance == 0)
// 		{
// 			oldDir = dir;
// 		}
// 		else
// 		{

			if(dir != oldDir)
			{
				numDirectionChanges++;

				oldDir = dir;
			}
			if(dir != oldDirColl)
			{
				numDirectionChangesCollisions++;
				oldDirColl = dir;
			}
//		}
		totalDistance += env->HCost(loc,l);
	}
	
	if((!success))//||(l == loc))
	{ 
		numFailedMoves++;
		moves.resize(0); 
		oldDirColl = kStay;
		//if(currTarget != -1) 
		//	currTarget = 0; 
	} 
	loc = l; 
	
}

template <class state, class action, class environment>
bool GenericPatrolUnit<state,action,environment>::Done()
{
	if(numPatrols == -1)
		return true;
	else if(counter >= numPatrols)
		return true;
	else
		return false;
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action, environment>::LogStats(StatCollection *sc)
{

}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action,environment>::LogFinalStats(StatCollection *sc)
{
	//Want: 
	// * nodesExpanded
	// * distance travelled
	// * failedMoves
	// * change in direction
	
	sc->AddStat("nodesExpanded", GetName(), (long)(nodesExpanded));
	sc->AddStat("distanceTravelled", GetName(), totalDistance);
	sc->AddStat("directionChanges",GetName(), (long)(numDirectionChanges));
	sc->AddStat("directionChangesCollision",GetName(), (long)(numDirectionChangesCollisions));
	sc->AddStat("failedMoves",GetName(), (long)(numFailedMoves));
	
}

#endif
