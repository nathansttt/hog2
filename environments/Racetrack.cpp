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
<<<<<<< HEAD
} // Location should always be in the screen
=======
	this->map = map;
}
>>>>>>> 7f796d409e520b96a44987fbf179310cce58d4d4

Racetrack::~Racetrack()
{
	delete me;
}
<<<<<<< HEAD
//  --- First Step
=======

void Racetrack::Reset(RacetrackState &s)
{
	
}

>>>>>>> 7f796d409e520b96a44987fbf179310cce58d4d4
void Racetrack::GetSuccessors(const RacetrackState &nodeID, std::vector<RacetrackState> &neighbors) const
{
	
}

void Racetrack::GetActions(const RacetrackState &nodeID, std::vector<RacetrackMove> &actions) const
{ // Search up vector -- initialize structs




	
}

void Racetrack::ApplyAction(RacetrackState &s, RacetrackMove a) const
{ // When x y velocity and action is applied -- location changes when veolocity changes
	s.x = s.x + a.xDelta*s.xVelocity;
	s.y = s.y + a.yDelta*s.yVelocity;
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
	return false;
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
// hog2 graph -- 0,0 in the middle -- upper left -1, -1 bottom left 1, 1 -- window can be scaled -- Coordinates have to be floats
	std::cout << "...Drawing a car!" << std::endl;

void Racetrack::Draw(Graphics::Display &display, const RacetrackState&s) const
{
	// TODO: draw agent location and movement vector
}

void Racetrack::DrawLine(Graphics::Display &display, const RacetrackState &x, const RacetrackState &y, float width) const
{
	// TODO: draw line between to locations
}
