//
//  Racetrack.cpp
//  hog2
//

#include "Racetrack.h"

std::ostream &operator<<(std::ostream &out, const RacetrackState &s)
{
	out << "{" << s.x << ", " << s.y << "}:(" << s.xVelocity << ", " << s.yVelocity << ")";
	return out;
}

bool operator==(const RacetrackState &l1, const RacetrackState &l2) {
	return
	((l1.x == l2.x) &&
	 (l1.y == l2.y) &&
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
}

Racetrack::~Racetrack()
{
	delete me;
}

void Racetrack::GetSuccessors(const RacetrackState &nodeID, std::vector<RacetrackState> &neighbors) const
{
	
}

void Racetrack::GetActions(const RacetrackState &nodeID, std::vector<RacetrackMove> &actions) const
{
	
}

void Racetrack::ApplyAction(RacetrackState &s, RacetrackMove a) const
{
	
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

void Racetrack::Draw(Graphics::Display &display, const RacetrackState&) const
{
	// TODO: draw agent location and movement vector

}
void Racetrack::DrawLine(Graphics::Display &display, const RacetrackState &x, const RacetrackState &y, float width) const
{
	// TODO: draw line between to locations
}
