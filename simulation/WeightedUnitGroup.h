/**
*
* @file WeightedUnitGroup.h
* @author Renee Jansen
* @date 09/19/2007
*/
#include "UnitGroup.h"
#include "WeightedMap2DEnvironment.h"

template <class state, class action, class environment>
class WeightedUnitGroup : public UnitGroup<state,action,environment>
{
	public:
		WeightedUnitGroup(){wt = -1; prop = -1;}
//		WeightedUnitGroup(double weight, double proportion) {wt = weight; prop = proportion;}
		
		void SetWeight(double weight) {wt = weight;}
		void SetProportion(double proportion) { prop=proportion;}
		virtual ~WeightedUnitGroup() {delete wme;}	
		virtual const char *GetName() { return "WeightedUnitGroup"; }
	
		virtual bool MakeMove(Unit<state, action, environment> *u, environment *e, SimulationInfo *si, action& a)
		{
			if(myE != e)
			{
				wme = new WeightedMap2DEnvironment(e);
				if(wt != -1)
					wme->SetWeight(wt);
				if(prop != -1)
					wme->SetProportionOld(prop);
					
				env = wme;
				myE = e;
			}
			return (u->MakeMove(env, env->GetOccupancyInfo(), si,a));
		}

		virtual void UpdateLocation(Unit<state, action, environment> *u, environment *e, state &loc, bool success, SimulationInfo *si)
		{
			if(myE != e)
			{
				wme = new WeightedMap2DEnvironment(e);
				if(wt != -1)
					wme->SetWeight(wt);
				if(prop != -1)
					wme->SetProportionOld(prop);
					
				env = wme;
				myE = e;
			}
			xyLoc s;
			u->GetLocation(s);
			wme->UpdateAngle(s, loc);
			u->UpdateLocation(env, loc, success, si);
		}
		
		virtual void OpenGLDraw(int window, environment *, SimulationInfo *) 
		{ 
			//std::cout<<"weighted opengldraw\n";
			wme->OpenGLDraw(window);
		}
		
		
	private:
		WeightedMap2DEnvironment* wme;
		environment* env; 
		environment* myE;
		double wt; 
		double prop;
};

