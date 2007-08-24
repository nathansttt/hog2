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
	GenericPatrolUnit(state &s, GenericSearchAlgorithm<state,action,environment>* alg, GLfloat _r, GLfloat _g, GLfloat _b);
	virtual const char *GetName() { return "AbsMapPatrolUnit"; } // want alg name as well?
	
	virtual bool MakeMove(environment *env, OccupancyInterface<state,action> *, SimulationInfo *si, action &dir);
	
	virtual void OpenGLDraw(int window, environment *, SimulationInfo *);
	void AddPatrolLocation(state &s); 
	state& GetGoal(); // get CURRENT goal? 
	virtual bool done() {return false;}
	void UpdateLocation(environment *, state &l, bool success, SimulationInfo *si);
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
	loc = s;
	//setObjectType(kWorldObject);
	currTarget = -1;
	nodesExpanded = nodesTouched = 0;
	
	algorithm = alg;
	
	r = 0.5;
	g = 0.5;
	b = 0.5;
}

template <class state, class action, class environment>
GenericPatrolUnit<state,action,environment>::GenericPatrolUnit(state &s, GenericSearchAlgorithm<state,action,environment>* alg, GLfloat _r, GLfloat _g, GLfloat _b):GenericSearchUnit<state,action,environment>(s,s,alg)
{
	locs.push_back(s);
	loc = s;

	currTarget = -1;
	nodesExpanded = nodesTouched = 0;
	
	algorithm = alg;
	
	r = _r;
	g = _g;
	b = _b;
}

template <class state, class action, class environment>
bool GenericPatrolUnit<state,action,environment>::MakeMove(environment *env, OccupancyInterface<state,action> *oi, SimulationInfo *si, action& dir)
{
	//std::cout<<"before makemove, patrolunit is at "<<loc<<std::endl;
	if (moves.size() > 0)
	{
		dir = moves.back();
		moves.pop_back();
		return true;
	}
	
	if (currTarget != -1)
	{
		// if we're not yet at our current target, then we don't need to update 
		// current target

		if(loc == locs[currTarget])
	 	{
	 		currTarget = (currTarget+1)%locs.size();
 		}
 		GoToLoc(env, currTarget);

		
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
	env->OpenGLDraw(window, loc,r,g,b);	
	
	for(unsigned int i=0; i<locs.size(); i++)
		env->OpenGLDraw(window, locs[i]);
}

template <class state, class action, class environment>
void GenericPatrolUnit<state,action,environment>::UpdateLocation(environment *env, state &l, bool success, SimulationInfo *si)
{ 
	//Update occupancy interface
	env->GetOccupancyInterface()->SetStateOccupied(loc,false);
	env->GetOccupancyInterface()->SetStateOccupied(l, true);
	
	loc = l; 
	if(!success)
	{ 
		moves.resize(0); 
		if(currTarget != -1) 
			currTarget = 0; 
	} 
	
	
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