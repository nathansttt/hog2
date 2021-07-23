//
//  Racetrack.cpp
//  hog2
//

#include "Racetrack.h"


std::ostream &operator<<(std::ostream &out, const RacetrackState &s)
{
	out << "{" << s.loc << "}:(" << s.xVelocity << ", " << s.yVelocity << ")";
	return out;
}

bool operator==(const RacetrackState &l1, const RacetrackState &l2) {
	return
	((l1 == l2) &&
	 (l1.xVelocity == l2.xVelocity) &&
	 (l1.yVelocity == l2.yVelocity));
}

bool operator!=(const RacetrackState &l1, const RacetrackState &l2) {
	return !(l1 == l2);
}

std::ostream &operator<<(std::ostream &out, const RacetrackMove &m)
{
	out << "(" << +m.xDelta << ", " << +m.yDelta << ")";
	return out;
}

bool operator==(const RacetrackMove &m1, const RacetrackMove &m2) {
	return (m1.xDelta == m2.xDelta) && (m1.yDelta == m2.yDelta);
}


Racetrack::Racetrack(Map *map)
{
	me = new MapEnvironment(map);
	this->map = map;
} // Location should always be in the screen


Racetrack::~Racetrack()
{
	delete me;
}

void Racetrack::Reset(RacetrackState &s) const
{
	s.xVelocity = 0;
	s.yVelocity = 0;
	for (int x = 0; x < map->GetMapWidth(); x++ )
	{
		for (int y = 0; y < map->GetMapHeight(); y++ )
		{
			if (map->GetTerrainType(x, y) == kStartTerrain)
			{
				s.loc.x = x;
				s.loc.y = y;
				return;
			}
		}
	}
	std::cout << "No start terrain found! \n";
	exit(1);
}


void Racetrack::GetSuccessors(const RacetrackState &nodeID, std::vector<RacetrackState> &neighbors) const
{
	
}

void Racetrack::GetActions(const RacetrackState &nodeID, std::vector<RacetrackMove> &actions) const
{ 




	
}



void Racetrack::ApplyAction(RacetrackState &s, RacetrackMove a) const
{ // When x y velocity and action is applied -- location changes when velocity changes
	RacetrackState temps = s;
	s.xVelocity += a.xDelta;
	s.yVelocity += a.yDelta;
	s.loc.x = s.loc.x + s.xVelocity;
	s.loc.y = s.loc.y + s.yVelocity;

	this->Boundaries(s, a);
	
	this->GoalTest(s, s);
	
}

// ------------ Boundaries ----------- //

void Racetrack::Boundaries(RacetrackState &s, RacetrackMove &v) const
{
	if (s.loc.x > 60000)
	{
		std::cout << "Too far left!! D: \n";
		// makes the agent stop when it hits the wall
		s.loc.x = 0;
		s.xVelocity = 0;
		v.xDelta = 0;
	}
	else if (s.loc.x >= map->GetMapWidth() - 1)
	{
		std::cout << "Too far right! \n";
		s.loc.x = map->GetMapWidth()-1;
		s.xVelocity = 0;
		v.xDelta = 0;
	}
	if (s.loc.y > 60000)
	{
		std::cout << "Too high up!! \n";
		// makes the agent stop
		s.loc.y = 0;
		s.yVelocity = 0;
		v.yDelta = 0;
	}
	
	else if (s.loc.y >= map->GetMapHeight() - 1)
	{
		std::cout << "Too far down! \n";
		s.loc.y = map->GetMapHeight()-1;
		s.yVelocity = 0;
		v.yDelta = 0;
	}
	// std::cout << s.loc.x << ", " << s.loc.y << std::endl;
	// std::cout << m->GetMapWidth() << ", " << m->GetMapHeight() << std::endl;
	
}




bool Racetrack::InvertAction(RacetrackMove &a) const
{
	// TODO: implement
	return false;
}

/*
 * The goal is implicit: We reach the goal if we have reached kEndTerrain.
 * So, we ignore the speicfic goal state
 */
bool Racetrack::GoalTest(const RacetrackState &node, const RacetrackState &goal) const
{
	// TODO: implement
	// Use the node to see if the location matches the goal location

	
	int temp_x = node.loc.x + node.xVelocity;
	int temp_y = node.loc.y + node.yVelocity;
	int initialx = temp_x;
	int initialy = temp_y;

	std::cout << "Tempx is " << temp_x << " \n";
	std::cout << "Tempy is " << temp_y << " \n";

	if (map->GetTerrainType(node.loc.x, node.loc.y) == kEndTerrain)
	{
		std::cout << "Touched the goal! \n";
		return true;
	}
	else
	{
		if (abs(node.loc.x - initialx) != 0)
		{	
			for (int x=0;x <= abs(node.loc.x - initialx); x++)
			{	
				
				if (node.loc.x < initialx)
				{
					temp_x = temp_x - 1;
				}
				else
				{
					temp_x = temp_x + 1;
				}
				
				if (map->GetTerrainType(temp_x, temp_y) == kEndTerrain)
				{
					std::cout << "X TOUCHED GOAL \n";
					return true;
				}
			}
		}
		if (abs(node.loc.y - temp_y) != 0)
		{

			
			for (int y = 0; y <= abs(node.loc.y - temp_y); y++)
			{
				
				if (map->GetTerrainType(temp_x, temp_y) == kEndTerrain)
				{
					std::cout << "Y TOUCHED THE GOAL \n";
					
					return true;
				}
				
				if (node.loc.y < initialy)
				{
					temp_y = temp_y - 1;
				}
				else
				{
					temp_y = temp_y + 1;
				}
			
			}
			
		}
		return false;
	}
	
}

// --- The legal function, which checks whether an action is legal --- //
bool Legal(const RacetrackState &node1, const action &act)
{

}




uint64_t Racetrack::GetStateHash(const RacetrackState &node) const
{
	return 0;
}

uint64_t Racetrack::GetActionHash(RacetrackMove act) const
{
	return 0;
}

void Racetrack::Draw(Graphics::Display &display) const
{
	me->Draw(display);
}

void Racetrack::Draw(Graphics::Display &display, const RacetrackState &s) const
{
	// TODO: draw agent location and movement vector
// draw a line
	
	xyLoc temp(s.loc.x + s.xVelocity, s.loc.y + s.yVelocity);
	me->DrawLine(display, s.loc, temp);
	me->Draw(display, s.loc);
}

void Racetrack::DrawLine(Graphics::Display &display, const RacetrackState &x, const RacetrackState &y, float width) const
{
	// TODO: draw line between to locations

}
