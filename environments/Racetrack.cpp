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
	std::vector<RacetrackMove> actions;
	RacetrackState temp = nodeID;
	this->GetActions(temp, actions);

	for (int i=0; i <= actions.size(); i++)
	{
		this->ApplyAction(temp, actions[i]); // applies the action to get the state
		neighbors.push_back(temp); // adds the state to the vector
		temp = nodeID; //resets the temporary state to its original
	}
	return;
	
}

void Racetrack::GetActions(const RacetrackState &nodeID, std::vector<RacetrackMove> &actions) const
{ 
	actions.clear();
	RacetrackMove up;
	RacetrackMove down;
	RacetrackMove left;
	RacetrackMove right;

	up.yDelta = -1;
	down.yDelta = 1;
	left.xDelta = -1;
	right.xDelta = 1;

	if (this->Legal(nodeID, up) == true)
	{
		std::cout << "Agent can go up! \n";
		actions.push_back(up);
	}
	if (this->Legal(nodeID, down) == true)
	{
		std::cout << "Agent can go down! \n";
		actions.push_back(down);
	}
	if (this->Legal(nodeID, left)==true)
	{
		std::cout << "Agent can go left! \n";
		actions.push_back(left);
	}
	if (this->Legal(nodeID, right)==true)
	{
		std::cout << "Agent can go right! \n";
		actions.push_back(right);
	}
	
	return;
	



	
}



void Racetrack::ApplyAction(RacetrackState &s, RacetrackMove a) const
{ // When x y velocity and action is applied -- location changes when velocity changes
	s.xVelocity += a.xDelta;
	s.yVelocity += a.yDelta;
	s.loc.x = s.loc.x + s.xVelocity;
	s.loc.y = s.loc.y + s.yVelocity;
	

	this->Boundaries(s, a);
	
	/*
	if (this->GoalTest(s, s) == true && map->GetTerrainType(s.loc.x, s.loc.y) != kEndTerrain)
	{
		std::cout << "Went too far past the goal! \n";
	}
	*/
	this->GoalTest(s, s);
}

// ------------ Boundaries ----------- //

void Racetrack::Boundaries(RacetrackState &s, RacetrackMove &v) const
{
	if (s.loc.x > 60000)
	{
		s.loc.x = 0;
		s.xVelocity = 0;
		//v.xDelta = 0;
	}
	else if (s.loc.x >= map->GetMapWidth() - 1)
	{
		s.loc.x = map->GetMapWidth()-1;
		s.xVelocity = 0;
		//v.xDelta = 0;
	}
	if (s.loc.y > 60000)
	{
		// makes the agent stop
		s.loc.y = 0;
		s.yVelocity = 0;
		//v.yDelta = 0;
	}
	
	else if (s.loc.y >= map->GetMapHeight() - 1)
	{
		s.loc.y = map->GetMapHeight()-1;
		s.yVelocity = 0;
		//v.yDelta = 0;
	}
	
	
	
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

	
	int temp_x = node.loc.x - node.xVelocity;
	int temp_y = node.loc.y - node.yVelocity;
	int initialx = temp_x;
	int initialy = temp_y;
	//std::cout << temp_x << "\n";
	//std::cout << temp_y << "\n";

	if (map->GetTerrainType(node.loc.x, node.loc.y) == kEndTerrain)
	{
		std::cout << "Touched the goal! \n";
		return true;
	}
	/*
	// --- The following code tests to see if the agent passed the goal --- //
	else
	{
		if (abs(node.loc.x - initialx) != 0)
		{	
			for (int x=0;x <= abs(node.loc.x - initialx); x++)
			{	
				if (map->GetTerrainType(temp_x, temp_y) == kEndTerrain)
				{
					std::cout << "X PASSED GOAL \n";
					return true;
				}
				if (node.loc.x < initialx)
				{
					temp_x = temp_x - 1;
				}
				else
				{
					temp_x = temp_x + 1;
				}
				
				
			}
		}
		if (abs(node.loc.y - temp_y) != 0)
		{

			
			for (int y = 0; y <= abs(node.loc.y - temp_y); y++)
			{
				
				if (map->GetTerrainType(temp_x, temp_y) == kEndTerrain)
				{
					std::cout << "Y PASSED THE GOAL \n";
					
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
	*/
	
	
}

// --- The legal function, which checks whether an action is legal --- //
bool Racetrack::Legal(const RacetrackState &node1, RacetrackMove &act) const
{

	RacetrackState temp = node1;
	temp.xVelocity += act.xDelta;
	temp.yVelocity += act.yDelta;
	temp.loc.x = temp.loc.x + temp.xVelocity;
	temp.loc.y = temp.loc.y + temp.yVelocity;
	int initialx = temp.loc.x;
	int initialy = temp.loc.y;

	if (abs(initialx -node1.loc.x) != 0)
	{
		for (int x=0; x < abs(temp.loc.x-node1.loc.x); x++)
		{
			if (this->GoalTest(temp, node1) == true)
			{	
				
				return true;
			}
			if (map->GetTerrainType(temp.loc.x, temp.loc.y) == kObstacle)
			{	
				std::cout << "Agent will hit obstacle! \n";
				return false;
			}
			if (temp.loc.x > 60000)
			{
				std::cout << "Agent will hit the wall on the left! \n";
				return false;
				
			}
			else if (temp.loc.x >= map->GetMapWidth() - 1)
			{
				std::cout << "Agent will hit the wall on the right! \n";
				return false;
				
				
			}
			if (initialx > node1.loc.x)
			{
				temp.loc.x = temp.loc.x - 1;
				
			}
			if (initialx < node1.loc.x)
			{
				temp.loc.x = temp.loc.x + 1;
				
			}
		}
		return true;
	}
	if (abs(initialy-node1.loc.y)!=0)
	{
		for (int y=0; y <= abs(initialy-node1.loc.y); y++)
		{
			if (this->GoalTest(temp, node1) == true)
			{
				return true;
			}
			if (map->GetTerrainType(temp.loc.x, temp.loc.y) == kObstacle)
			{	
				std::cout << "Agent will hit obstacle! \n";
				return false;
			}
			if (temp.loc.y > 60000)
			{
				std::cout << "Agent will hit the wall on top! \n";
				return false;
			}
			
			else if (temp.loc.y >= map->GetMapHeight() - 1)
			{
				std::cout << "Agent will hit wall on the bottom! \n";
				return false;
			}
			if (initialy > node1.loc.y)
			{
				temp.loc.y = temp.loc.y - 1;
				
			}
			if (initialy < node1.loc.y)
			{
				temp.loc.y = temp.loc.y + 1;
			}
		}
		return true;
	}
	
	
	return true;
	
	
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
	
	//xyLoc temp(s.loc.x + s.xVelocity, s.loc.y + s.yVelocity);
	xyLoc temp2(s.loc.x-s.xVelocity, s.loc.y-s.yVelocity);
	//me->DrawLine(display, s.loc, temp);
	me->Draw(display, s.loc);
	me->DrawLine(display, s.loc, temp2);
}

void Racetrack::DrawLine(Graphics::Display &display, const RacetrackState &c, const RacetrackState &d, float width) const
{
	// TODO: draw line between two locations
	
	int dx = d.loc.x - c.loc.x;
	int dy = d.loc.y - c.loc.y;
	int D = 2*dy - dx;
	int y = c.loc.y;

	for (int x = c.loc.x; x <= d.loc.x; x++)
	{
		//std::cout << "(" << x << "," << y << ")\n";
		//std::cout << D << "\n";
		if (D > 0)
		{
			y = y + 1;
			D = D - 2*dy;
		}
		D = D + 2*dy;
		
	}
	

	

}
