#include "WeightedMap2DEnvironment.h"

WeightedMap2DEnvironment::WeightedMap2DEnvironment(MapAbstraction *ma)
:AbsMapEnvironment(ma)
{
	// initialize all edge counters to 0
	Graph* g = ma->GetAbstractGraph(0);
	for(int i=0; i<g->getNumEdges(); i++)
	{
		// do counter reset here
		// --> move into separate fn? Will need to redo sometimes? 
	}
}

void WeightedMap2DEnvironment::ApplyAction(xyLoc &s, tDirection dir)
{
	xyLoc old = s;
	switch (dir)
	{
		case kN: s.y-=1; break;
		case kS: s.y+=1; break;
		case kE: s.x+=1; break;
		case kW: s.x-=1; break;
		case kNW: s.y-=1; s.x-=1; break;
		case kSW: s.y+=1; s.x-=1; break;
		case kNE: s.y-=1; s.x+=1; break;
		case kSE: s.y+=1; s.x+=1; break;
		default: break;
	}
	if (map->canStep(s.x, s.y, old.x, old.y) && !(MOI->GetStateOccupied(s))) 
	// ADD CHECK: IF OK WITH MAPOCCUPANCY INTERFACE --> enough (for now) with just GetStateOccupied?
		return;
	s = old;

}

double WeightedMap2DEnvironment::GCost(xyLoc &l1, xyLoc &l2)
{
	double h = HCost(l1, l2);
	if (fgreater(h, ROOT_TWO))
		return DBL_MAX;
		
	// Add a weight 
	return h;
}


/**********************************************************/

/** 
* Constructor for MapOccupancyInterface
* @author Renee Jansen
* @date 06/21/2007
*/
//template <class state, class action>
MapOccupancyInterface::MapOccupancyInterface(Map* m)
{
 	mapWidth = m->getMapWidth();
 	mapHeight = m->getMapHeight();
	bitvec = new bitVector(mapWidth * mapHeight);
}

//template <class state, class action>
MapOccupancyInterface::~MapOccupancyInterface()
{
	delete bitvec;
	bitvec = 0;
}

//template <class state, class action>
void MapOccupancyInterface::SetStateOccupied(xyLoc s, bool occupied)
{
	// Make sure the location is valid
	assert(s.x>0 && s.x<=mapWidth && s.y>0 && s.y<=mapHeight);
	bitvec->set(CalculateIndex(s.x,s.y), occupied);
}
//template <class state, class action>
bool MapOccupancyInterface::GetStateOccupied(xyLoc s)
{
	// Make sure the location is valid
	assert(s.x>0 && s.x<=mapWidth && s.y>0 && s.y<=mapHeight);
	return bitvec->get(CalculateIndex(s.x,s.y));
}

// template <class state, class action>
// bool MapOccupancyInterface::CanMove(state s, state t)
// {
// 	if(bv->get(CalculateIndex(t.x, t.y)) && )
// 		return false;
// 	else
// 		return true;
// }


/** Gets the index into the bitvector. 
*
* We need to be able to convert (x,y) locations to a position in the bitvector. 
*
* @author Renee Jansen
* @date 06/22/2007
*
* @param x The x-coordinate of the location
* @param y The y-coordinate of the location
* @return The index into the bit vector
*/
//template <class state, class action>
long MapOccupancyInterface::CalculateIndex(uint16_t x, uint16_t y)
{
	return (y * mapWidth) + x;
}
