/*
 * $Id: LRTAStarUnit.h,v 1.5 2005/12/09 23:49:44 nathanst Exp $
 *
 *  Hierarchical Open Graph File
 *
 *  Created by Shanny (based on LRTS) on 10/03/05.
 *  Ported to HOG2 9/2/09
 */

#ifndef LRTAStarUnit_H
#define LRTAStarUnit_H

#include "Unit.h"
#include <deque>
#include "FPUtil.h"
#include "LRTAStar.h"

template <class state, class action, class environment>
class LRTAStarUnit : public Unit<state, action, environment> {

public:
	LRTAStarUnit(state &s, state &target, LRTAStar<state, action, environment> *alg)
	{
		currentLoc = s;
		goalLoc = target;
		//goal = target;
		algorithm = alg;
		totalLearned = 0;
		StartNewTrial(0);
	}
	virtual ~LRTAStarUnit() {}
	const char *GetName() { return algorithm->GetName(); }
	bool Done() { return (currentLoc==goalLoc) && fequal(algorithm->GetAmountLearned(), totalLearned); }
	void StartNewTrial(StatCollection *) {
		printf("New trial: %f learned this trial; %f total\n",
			   algorithm->GetAmountLearned()-totalLearned, algorithm->GetAmountLearned());
		totalLearned = algorithm->GetAmountLearned();
	}
	
	bool MakeMove(environment *, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *, action& a);
	//  virtual void printRoundStats(FILE *f) { fprintf(f,"%8.2f",amountLearned); }
	
	void OpenGLDraw(const environment *e, const SimulationInfo<state,action,environment> *) const;
	void UpdateLocation(environment *, state &s, bool success, SimulationInfo<state,action,environment> *)
	{
		if (success) 
		{
			//std::cout << "Updating location from " << currentLoc << " to " << s << std::endl;
			currentLoc = s;
		}
		else {
			//std::cout << "Move failed, not updating location" << std::endl;
		}
	}
	void GetGoal(state &s) { s = goalLoc; }
	void GetLocation(state &s) { s = currentLoc; }
private:
	LRTAStar<state, action, environment> *algorithm;			// pointer to the LRTA* algorithm
	double totalLearned;			// Actual amount learned
	std::vector<state> path;
	state currentLoc, goalLoc;
	//Unit<state, action, environment> *goal;
};


//
//template <class state, class action, class environment>
//LRTAStarUnit<state, action, environment>::LRTAStarUnit(state &s, unit *_target, LRTAStar *alg)
//:searchUnit(_x, _y, _target, alg)
//{
//	algorithm = alg;			// set the algorithm
//	startNewTrial();
//}
//
//template <class state, class action, class environment>
//LRTAStarUnit<state, action, environment>::~LRTAStarUnit(void)
//{
//}
//
//// Reset the unit for new trial
//template <class state, class action, class environment>
//void LRTAStarUnit<state, action, environment>::startNewTrial() {
//	totalLearned = algorithm->GetAmountLearned();
//}	

template <class state, class action, class environment>
bool LRTAStarUnit<state, action, environment>::MakeMove(environment *e, OccupancyInterface<state,action> *, SimulationInfo<state,action,environment> *, action& a)
{
	if (currentLoc == goalLoc)
		return false;
//	if (GetUnitGroup() == 0)
//		return false;
	if (path.size() <= 1)
	{
		algorithm->GetPath(e, currentLoc, goalLoc, path);
		if (path.size() <= 1)
			return false;
		std::reverse(path.begin(), path.end());
	}
	a = e->GetAction(path[path.size()-1], path[path.size()-2]);
	path.pop_back();
	return true;
}

template <class state, class action, class environment>
void LRTAStarUnit<state, action, environment>::OpenGLDraw(const environment *e, const SimulationInfo<state,action,environment> *si) const
{
	PublicUnitInfo<state, action, environment> i;
	si->GetPublicUnitInfo(si->GetCurrentUnit(), i);
	e->SetColor(0.5, 0.5, 0.5, 1.0);
	if (fgreater(si->GetSimulationTime(), i.nextTime))
		e->OpenGLDraw(i.currentState);
	else
		e->OpenGLDraw(i.lastState, i.currentState,
					  (si->GetSimulationTime()-i.lastTime)/(i.nextTime-i.lastTime));
	//e->OpenGLDraw(currentLoc);
	
	e->SetColor(0.0, 1.0, 0.0, 1.0);
	e->OpenGLDraw(goalLoc);
	e->SetColor(1.0, 1.0, 0.0, 1.0);
	e->OpenGLDraw(currentLoc);

	algorithm->OpenGLDraw(e);
	
//	e->SetColor(0.0, 0.0, 0.5, 0.25);
//	for (typename LSStateStorage::const_iterator it = hashTable.begin(); it != hashTable.end(); it++)
//	{
//		if ((*it).second.theState == currentLoc)
//		{
//		}
//		else {
//			e->OpenGLDraw((*it).second.theState);
//		}
//	}
}

#endif
