#include "Unit.h"
#include "WeightedMap2DEnvironment.h"

template <class environment>
class GreedyDMUnit : public Unit<xyLoc,tDirection,environment>
{
	public:
		GreedyDMUnit(xyLoc &startLoc) :loc(startLoc) {}
		~GreedyDMUnit() {}
		
		virtual const char *GetName() { return "Greedy DM following unit"; }
		
		virtual bool MakeMove(environment *env, OccupancyInterface<xyLoc, tDirection> *, SimulationInfo *, tDirection& a)
		{
			std::vector<tDirection> directions;
			directions.push_back(kN);
			directions.push_back(kNE);
			directions.push_back(kE);
			directions.push_back(kSE);
			directions.push_back(kS);
			directions.push_back(kSW);
			directions.push_back(kW);
			directions.push_back(kNW);
	
			Vector2D nodeVec = wme->GetAngle(loc);
		
			// if no angle stored, pick random
			if(nodeVec.x == 0 && nodeVec.y == 0)
			{ 	
				a = directions[random()%directions.size()];
				return true;
			}		
					
			double bestDotProd = -1;
			tDirection bestDir;		
				
			for(unsigned int i=0; i<directions.size(); i++)
			{			
				xyLoc nextThisDir;
				wme->GetNextState(loc,directions[i],nextThisDir);
				if(prevLoc == nextThisDir)
					continue;	
				if(env->GetMap()->canStep(loc.x, loc.y, nextThisDir.x, nextThisDir.y))
				{
					Vector2D dirVec = wme->GetAngleFromDirection(directions[i]);
					Vector2D nextVec = wme->GetAngle(nextThisDir);
					
					//double dotProd = (nodeVec.x * dirVec.x) + (nodeVec.y * dirVec.y);
					double dotProd = (nodeVec.x * nextVec.x) + (nodeVec.y * nextVec.x);
					if(dotProd > bestDotProd)
					{
						bestDotProd = dotProd;
						bestDir = directions[i];
					}	
				}		
			}
			
// 			xyLoc next;
// 			wme->GetNextState(loc, bestDir, next);
// 			
// 			//std::cout<<"loc "<<loc<<" next "<<next<<std::endl;
// 			if(!(env->GetMap()->canStep(loc.x, loc.y, next.x, next.y))) // make best possible move
// 			{
// 				
// 				//std::cout<<"can't go here - random\n";
// 				a = directions[random()%directions.size()];
// 			}	
// 			else
	
			if(bestDir != -1)
				a = bestDir;
			
// 			else
// 				a = directions[random()%directions.size()];
			
			return true;
		}
		
		
		virtual void UpdateLocation(environment *, xyLoc &newloc, bool success, SimulationInfo *)
		{
			if (success)
			{
				prevLoc = loc;
				loc = newloc;
			}
		}
		
		virtual void GetLocation(xyLoc &l)
		{ l = loc; }
	
		virtual void OpenGLDraw(int window, environment *env, SimulationInfo *)
		{ env->OpenGLDraw(window, loc); }
		
		virtual void GetGoal(xyLoc &s)
		{ s = loc; }
		
		void SetEnvironment(WeightedMap2DEnvironment *w){wme = w;}
		
	private:
		xyLoc loc, prevLoc;
		WeightedMap2DEnvironment *wme;
};
