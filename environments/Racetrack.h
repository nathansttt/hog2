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

struct RacetrackMove { // -1, 1
	int xDelta, yDelta;
};


struct RacetrackState {
	xyLoc loc;
	int xVelocity, yVelocity;
};

std::ostream &operator<<(std::ostream &out, const RacetrackState &s);
bool operator==(const RacetrackState &l1, const RacetrackState &l2);
bool operator!=(const RacetrackState &l1, const RacetrackState &l2);
std::ostream &operator<<(std::ostream &out, const RacetrackMove &m);
bool operator==(const RacetrackMove &m1, const RacetrackMove &m2); // comparing 

//Different types of domain
const tTerrain kStartTerrain = kSwamp; // start loc
const tTerrain kEndTerrain = kGrass;

class Racetrack : public SearchEnvironment<RacetrackState, RacetrackMove> {
public:
	Racetrack(Map *map); 
	~Racetrack();
	void GetSuccessors(const RacetrackState &nodeID, std::vector<RacetrackState> &neighbors) const; //current state --> pass in a vector (array lists) __> fill in -- no modification
	void GetActions(const RacetrackState &nodeID, std::vector<RacetrackMove> &actions) const; // no modification
	int GetNumSuccessors(const RacetrackState &stateID) const
	{ std::vector<RacetrackState> neighbors; GetSuccessors(stateID, neighbors); return (int)neighbors.size(); }
	
	void Reset(RacetrackState &s) const;
	
	void ApplyAction(RacetrackState &s, RacetrackMove a) const;// a = action s(state) gets changed
	bool InvertAction(RacetrackMove &a) const;
	
	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const RacetrackState &node1, const RacetrackState &node2) const { return 0; } //Later
	/** Heuristic value between node and the stored goal. Asserts that the
	 goal is stored **/
	double HCost(const RacetrackState &node) const { return 0; }
	
	double GCost(const RacetrackState &node1, const RacetrackState &node2) const { return 1; };
	double GCost(const RacetrackState &node, const RacetrackMove &act) const { return 1; };
	bool GoalTest(const RacetrackState &node, const RacetrackState &goal) const; // Goal reached?
	// 
	uint64_t GetStateHash(const RacetrackState &node) const; // turn into a number
	uint64_t GetActionHash(RacetrackMove act) const;
	
	// Deprecated
	void OpenGLDraw() const {};
	void OpenGLDraw(const RacetrackState&) const {};
	void OpenGLDraw(const RacetrackState&, const RacetrackMove&) const {};

	void Draw(Graphics::Display &display) const;
	void Draw(Graphics::Display &display, const RacetrackState&) const;
	void DrawLine(Graphics::Display &display, const RacetrackState &x, const RacetrackState &y, float width) const;
protected: // take two states and draw a line
private:
	MapEnvironment *me;
	Map *map;
};


#endif /* TOH_hpp */
