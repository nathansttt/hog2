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
		WeightedUnitGroup(){wt = -1; prop = -1; useWindow = false; windowSize = -1; localWeights = false; currDrawEnv = 0; }
//		WeightedUnitGroup(double weight, double proportion) {wt = weight; prop = proportion;}
		
		void SetWeight(double weight) {wt = weight;}
		void SetProportion(double proportion) { prop=proportion;}
		void SetUseWindow(bool b) {useWindow = b;}
		void SetWindowSize(double d) {windowSize = d;}
		void UseLocalWeights(bool b) {localWeights = b;}
		void SetLocalWeightRadius (double r) {localRadius = r;}
		
		virtual ~WeightedUnitGroup() 
		{	
			delete wme; 
			for(unsigned int i=0; i<unitWme.size(); i++) 
			delete unitWme[i];
		}
			
		virtual const char *GetName() { return "WeightedUnitGroup"; }
	
		virtual bool MakeMove(Unit<state, action, environment> *u, environment *e, SimulationInfo *si, action& a)
		{
			if(myE != e)
			{
				wme = new WeightedMap2DEnvironment(e);
				
				if(localWeights)
				{	
					for (unsigned int x = 0; x < this->GetMembers().size(); x++)
					{
						unitWme.push_back(new WeightedMap2DEnvironment(e));
						if(wt != -1)
							unitWme[unitWme.size()-1]->SetWeight(wt);
						if(prop != -1)
							unitWme[unitWme.size()-1]->SetProportionOld(prop);
						if((useWindow)&&(windowSize >= 0))
						{
							unitWme[unitWme.size()-1]->UseWindow(true);
							unitWme[unitWme.size()-1]->SetWindowSize(windowSize);
						}
						environment* uenv = unitWme[unitWme.size()-1];
						unitEnv.push_back(uenv);
					}
					currDrawEnv = unitWme.size();
				}
				
				if(wt != -1)
					wme->SetWeight(wt);
				if(prop != -1)
					wme->SetProportionOld(prop);
				if((useWindow)&&(windowSize >= 0))
				{
					wme->UseWindow(true);
					wme->SetWindowSize(windowSize);
				}
					
				env = wme;
				myE = e;
			}
			if(useWindow) // Set the unit's current location as center for the window 
			{
				xyLoc x;
				u->GetLocation(x);
				wme->SetWindowCenter(x);
			}
			if(localWeights)
			{
				while(unitWme.size() < this->GetMembers().size())
				{
					unitWme.push_back(new WeightedMap2DEnvironment(e));
					environment* uenv = unitWme[unitWme.size()-1];
					unitEnv.push_back(uenv);
				}
				// Update local weights in this unit's environment
				UpdateLocalWeights(u);
				
				environment* thisUnitsEnv = unitEnv[GetUnitIndex(u)];
								
				return (u->MakeMove(thisUnitsEnv, thisUnitsEnv->GetOccupancyInfo(), si,a));
			}
			
			return (u->MakeMove(env, env->GetOccupancyInfo(), si,a));
		}

		void UpdateLocalWeights(Unit<state,action,environment> *u)
		{
			WeightedMap2DEnvironment* uwme = unitWme[GetUnitIndex(u)];
			xyLoc currLoc;
			u->GetLocation(currLoc);
			
			//std::cout<<std::endl<<"I'm at "<<currLoc<<std::endl;
			for(int x = currLoc.x - (int)localRadius; x<=currLoc.x + (int)localRadius; x++)
			{
			 	if((x<0)||(x>=uwme->GetMap()->getMapWidth()))
			 		continue;
				for(int y = currLoc.y - (int)localRadius; y<=currLoc.y + (int)localRadius; y++)
			 	{
			 		if((y<0)||(y>=uwme->GetMap()->getMapHeight()))
			 			continue;
			 			
			 		// if this location is within localRadius, update angle
			 		xyLoc thisloc;
			 		thisloc.x = x;
			 		thisloc.y = y;
			 		
			 		if(uwme->HCost(currLoc, thisloc) <= localRadius)
			 		{
			 			//std::cout<<"updating "<<thisloc<<std::endl;
			 			Vector2D angle = wme->GetAngle(thisloc);
			 			uwme->SetAngle(thisloc, angle);
			 		}
			 		
			 	}
			}
		}

		int GetUnitIndex(Unit<state,action,environment> *u)
		{
			for (unsigned int x = 0; x < this->GetMembers().size(); x++)
			{
				if (this->GetMembers()[x] == u)
					return x;
			}
		}

		virtual void UpdateLocation(Unit<state, action, environment> *u, environment *e, state &loc, bool success, SimulationInfo *si)
		{
			if(myE != e)
			{
				if(localWeights)
				{	
					for (unsigned int x = 0; x < this->GetMembers().size(); x++)
					{
						unitWme.push_back(new WeightedMap2DEnvironment(e));
						if(wt != -1)
							unitWme[unitWme.size()-1]->SetWeight(wt);
						if(prop != -1)
							unitWme[unitWme.size()-1]->SetProportionOld(prop);
						if((useWindow)&&(windowSize >= 0))
						{
							unitWme[unitWme.size()-1]->UseWindow(true);
							unitWme[unitWme.size()-1]->SetWindowSize(windowSize);
						}
						environment* uenv = unitWme[unitWme.size()-1];
						unitEnv.push_back(uenv);
					}
					currDrawEnv = unitWme.size();
				}
				
				wme = new WeightedMap2DEnvironment(e);
				if(wt != -1)
					wme->SetWeight(wt);
				if(prop != -1)
					wme->SetProportionOld(prop);
				if((useWindow)&&(windowSize >= 0))
				{
					wme->UseWindow(true);
					wme->SetWindowSize(windowSize);
				}	
					
				env = wme;
				myE = e;
			}
			xyLoc s;
			u->GetLocation(s);
			wme->UpdateAngle(s, loc);
			u->UpdateLocation(env, loc, success, si);
		}
		
		void DrawNextEnvironment()
		{
			if(currDrawEnv != unitWme.size())
				this->GetMembers()[currDrawEnv]->SetColor(1,0,0);
			currDrawEnv = (currDrawEnv + 1)%(unitWme.size()+1);
		}
		
		virtual void OpenGLDraw(int window, environment *, SimulationInfo *) 
		{ 
			//std::cout<<"weighted opengldraw\n";
			if(currDrawEnv == unitWme.size())
				wme->OpenGLDraw(window);
			else
			{
				this->GetMembers()[currDrawEnv]->SetColor(0,0,1);
				unitWme[currDrawEnv]->OpenGLDraw(window);
			}
		}
		
		
	private:
		WeightedMap2DEnvironment* wme;
		std::vector<WeightedMap2DEnvironment*> unitWme;
		std::vector<environment*> unitEnv;
		environment* env; 
		environment* myE;
		double wt; 
		double prop;
		bool useWindow;
		double windowSize;
		bool localWeights;
		double localRadius;
		unsigned int currDrawEnv;
};

