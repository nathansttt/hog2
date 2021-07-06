//
//  Racetrack.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 7/5/21.
//

#ifndef RACETRACK_H
#define RACETRACK_H

#include <stdio.h>
#include <cstdint>
#include <math.h>
#include "SearchEnvironment.h"
#include "Map2DEnvironment.h"
#include "Map.h"

// TODO: Need to move code into its own namespace

struct RacetrackMove {
	int xDelta, yDelta;
};

struct RacetrackState {
	int x, y;
	int xVelocity, yVelocity;
};

std::ostream &operator<<(std::ostream &out, const RacetrackState &s);
bool operator==(const RacetrackState &l1, const RacetrackState &l2);
bool operator!=(const RacetrackState &l1, const RacetrackState &l2);
std::ostream &operator<<(std::ostream &out, const RacetrackMove &m);
bool operator==(const RacetrackMove &m1, const RacetrackMove &m2);


const tTerrain kStartTerrain = kSwamp;
const tTerrain kEndTerrain = kGrass;

class Racetrack : public SearchEnvironment<RacetrackState, RacetrackMove> {
public:
	Racetrack(Map *map);
	~Racetrack();
	void GetSuccessors(const RacetrackState &nodeID, std::vector<RacetrackState> &neighbors) const;
	void GetActions(const RacetrackState &nodeID, std::vector<RacetrackMove> &actions) const;
	int GetNumSuccessors(const RacetrackState &stateID) const
	{ std::vector<RacetrackState> neighbors; GetSuccessors(stateID, neighbors); return (int)neighbors.size(); }
	
	void ApplyAction(RacetrackState &s, RacetrackMove a) const;
	bool InvertAction(RacetrackMove &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const RacetrackState &node1, const RacetrackState &node2) const { return 0; }
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	double HCost(const RacetrackState &node) const { return 0; }
	
	double GCost(const RacetrackState &node1, const RacetrackState &node2) const { return 1; };
	double GCost(const RacetrackState &node, const RacetrackMove &act) const { return 1; };
	bool GoalTest(const RacetrackState &node, const RacetrackState &goal) const;
	
	uint64_t GetStateHash(const RacetrackState &node) const;
	uint64_t GetActionHash(RacetrackMove act) const;
	
	// Deprecated
	void OpenGLDraw() const {};
	void OpenGLDraw(const RacetrackState&) const {};
	void OpenGLDraw(const RacetrackState&, const RacetrackMove&) const {};

	void Draw(Graphics::Display &display) const;
	void Draw(Graphics::Display &display, const RacetrackState&) const;
	void DrawLine(Graphics::Display &display, const RacetrackState &x, const RacetrackState &y, float width) const;
protected:
private:
	MapEnvironment *me;
};


#endif /* TOH_hpp */
