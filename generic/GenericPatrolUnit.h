#ifndef GENERICPATROLUNIT_H
#define GENERICPATROLUNIT_H

#include "Unit.h"
#include "GenericSearchUnit.h"
#include "ReservationProvider.h"
//#include "SearchEnvironment.h"

template <class state, class action, class environment>
class GenericPatrolUnit : public GenericSearchUnit<state,action, environment> {
public:
	GenericPatrolUnit(state &s, GenericSearchAlgorithm<state,action,environment>* alg);

	virtual const char *GetName() { return "AbsMapPatrolUnit"; } // want alg name as well?
	
	virtual bool MakeMove(environment *env, OccupancyInterface<state,action> *, SimulationInfo *si, action &dir);
	
	virtual void OpenGLDraw(int window, environment *, SimulationInfo *);
	void AddPatrolLocation(state &s); // by ref or not?
	state& GetGoal(); // get CURRENT goal? 
	virtual bool done() {return false;}
	void UpdateLocation(environment *, state &l, bool success, SimulationInfo *si)
	{ loc = l; if(!success){ moves.resize(0); if(currTarget != -1) currTarget = 0; } }
//	void updateLocation(int _x, int _y, bool worked, SimulationInfo *)
//	{ loc.x = _x; loc.y = _y; if (!worked) { moves.resize(0); if (currTarget != -1) currTarget = 0; } }
	void LogStats(StatCollection *stats);
	void LogFinalStats(StatCollection *stats);
private:
	GLfloat r, g, b;
	xyLoc loc;

	// this used to return path cost, but that wasn't used... 
	/*double*/ void GoToLoc(environment *env, int which);
 	void AddPathToCache(environment *env, std::vector<state> &path);
	std::vector<action> moves;
	std::vector<state> locs;

	GenericSearchAlgorithm<state,action,environment> *algorithm;
	
	int currTarget;
	int nodesExpanded;
	int nodesTouched;
};

template <class state, class action, class environment>
GenericPatrolUnit<state,action,environment>::GenericPatrolUnit(state &s,GenericSearchAlgorithm<state,action,environment>* alg) 
:GenericSearchUnit<state,action,environment>(s,s,alg)
{
	locs.push_back(s);
	
	//setObjectType(kWorldObject);
	currTarget = -1;
	nodesExpanded = nodesTouched = 0;
	
	algorithm = alg;
}

template <class state, class action, class environment>
bool GenericPatrolUnit<state,action,environment>::MakeMove(environment *env, OccupancyInterface<state,action> *oi, SimulationInfo *si, action& dir)
{
	if (moves.size() > 0)
	{
		dir = moves.back();
		moves.pop_back();
		return true;
	}
	
	if (currTarget != -1)
	{
 		GoToLoc(env, currTarget);
		currTarget = (currTarget+1)%locs.size();
		if (moves.size() > 0)
		{
			dir = moves.back();
			moves.pop_back();
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
	env->OpenGLDraw(window, loc);	
	
	for(unsigned int i=0; i<locs.size(); i++)
		env->OpenGLDraw(window, locs[i]);
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action, environment>::LogStats(StatCollection *sc)
{

}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action,environment>::LogFinalStats(StatCollection *sc)
{

}

#endif