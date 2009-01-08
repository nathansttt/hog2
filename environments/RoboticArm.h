/*
 *  RoboticArm.h
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/15/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#ifndef ROBOTICARM_H
#define ROBOTICARM_H

#include <stdint.h>
#include <iostream>
#include "Map.h"
#include "MapAbstraction.h"
#include "SearchEnvironment.h"
#include "UnitSimulation.h"
#include "ReservationProvider.h"
#include "BitVector.h"

#include <cassert>

//#include "BaseMapOccupancyInterface.h"

class line2d {
public:
	line2d() {}
	line2d(recVec a, recVec b) :start(a), end(b) {}
	bool crosses(line2d which) const;
	recVec start;
	recVec end;
};

static bool operator==(const recVec &l1, const recVec &l2) {
	return (fequal(l1.x, l2.x) && fequal(l1.y, l2.y));
}

class armAngles {
public:
	armAngles() {}
	int GetAngle(int which) const;
	void SetAngle(int which, int value);
	int GetNumArms() const;
	void SetNumArms(int count);
	void SetGoal(double x, double y);
	void GetGoal(double &x, double &y) const;
	bool IsGoalState() const;
	uint64_t angles;
};

static std::ostream& operator <<(std::ostream & out, const armAngles &loc)
{
	if (loc.IsGoalState())
	{
		double x, y;
		loc.GetGoal(x, y);
		out << "(" << x << ", " << y << ")" << std::endl;
	}
	else {
		out << "[";
		for (int x = 0; x < loc.GetNumArms()-1; x++)
			out << loc.GetAngle(x) << ", ";
		out << loc.GetAngle(loc.GetNumArms()-1) << "]";
	}
	return out;
}

static bool operator==(const armAngles &l1, const armAngles &l2) {
	return (l1.angles == l2.angles);
}

enum tRotation {
	kRotateCCW = -1,
	kNoRotation = 0,
	kRotateCW = 1
};

class armRotations {
public:
	armRotations()
	:rotations() {}
	uint16_t rotations;
	tRotation GetRotation(int which) const;
	void SetRotation(int which, tRotation dir);
};

static bool operator==(const armRotations &l1, const armRotations &l2) {
	return (l1.rotations == l2.rotations);
}

class RoboticArm : public SearchEnvironment<armAngles, armRotations>
{
public:
	RoboticArm(int DOF, double armLength, double tolerance = 0.01);
	virtual ~RoboticArm();

	double GetTolerance() const { return tolerance; }
	void GetTipPosition( armAngles &s, double &x, double &y );

	void AddObstacle(line2d obs) { obstacles.push_back(obs); }
	void GetSuccessors(armAngles &nodeID, std::vector<armAngles> &neighbors);
	void GetActions(armAngles &nodeID, std::vector<armRotations> &actions);
	armRotations GetAction(armAngles &s1, armAngles &s2);
	virtual void ApplyAction(armAngles &s, armRotations dir);
	
	virtual bool InvertAction(armRotations &a);
	
	virtual double HCost(armAngles &node1, armAngles &node2);
	virtual double GCost(armAngles &node1, armAngles &node2) { return 1; }
	virtual double GCost(armAngles &node1, armRotations &act) { return 1; }
	bool GoalTest(armAngles &node, armAngles &goal);
	uint64_t GetStateHash(armAngles &node);
	uint64_t GetActionHash(armRotations act);

	virtual void OpenGLDraw(int window);
	virtual void OpenGLDraw(int window, armAngles &l);
	virtual void OpenGLDraw(int, armAngles &, armRotations &);
	virtual void OpenGLDraw(int, armAngles &, armRotations &, GLfloat r, GLfloat g, GLfloat b);
	virtual void OpenGLDraw(int, armAngles &l, GLfloat r, GLfloat g, GLfloat b);
	
	virtual void GetNextState(armAngles &currents, armRotations dir, armAngles &news);

	bool LegalState(armAngles &a);
	void GenerateLegalStateTable( armAngles &legalArm );
	void GenerateTipPositionTables( armAngles &sampleArm );

	void GenerateRandomHeuristic( const armAngles &sampleArm );
	int GenerateHeuristic( const armAngles &sampleArm, armAngles &goal );
	int GenerateMaxDistHeuristics( const armAngles &sampleArm,
				       const int numHeuristics );

	// Returns true if the position is guaranteed to be valid.
	// Returns false if the position is either unvalid,
	// or possibly unvalid
	bool ValidGoalPosition( double goalX, double goalY );
private:
	void DrawLine(line2d l);
	bool LegalArmConfig(armAngles &a);
	void GenerateLineSegments(armAngles &a, std::vector<line2d> &armSegments);

	void GenerateCPDB();
	int DOF;
	double armLength, tolerance;
	std::vector<std::vector<bool> > legals;
	bool m_TableComplete;
	double GetSin(int angle);
	double GetCos(int angle);
	void BuildSinCosTables();
	std::vector<double> sinTable;
	std::vector<double> cosTable;
	std::vector<line2d> obstacles;
	std::vector<line2d> armSegments;

	uint8_t *legalStateTable;
	uint8_t *legalGoalTable;

	std::vector<uint16_t *> distancesTables;
	std::vector<uint16_t *> minTipDistancesTables;
	std::vector<uint16_t *> maxTipDistancesTables;
	std::vector<uint16_t> tablesNumArms;

	std::vector<armAngles> *tipPositionTables;


	// convert an arm configuration into an index
	uint64_t ArmAnglesIndex( const armAngles &arm );
	uint64_t NumArmAnglesIndices( const armAngles &arm ) const {
	  return 1 << ( 9 * arm.GetNumArms() );
	}

	// convert a tip position into an index
	int TipPositionIndex( const double x, const double y,
			      const double minX, const double minY,
			      const double width );
	int NumTipPositionIndices() const {
	  int count = (int)ceil( 2.0 / GetTolerance() );
	  return count * count;
	}

	// write/read a binary representation of a configuration
	// DOES NOT WRITE/READ THE ARM LENGTHS!
	int WriteArmAngles(FILE *file, armAngles &a);
	int ReadArmAngles(FILE *file, armAngles &a);

	void UpdateTipDistances( armAngles &arm, uint16_t distance,
				 uint16_t *minTipDistances,
				 uint16_t *maxTipDistances );

	// common subroutine for all the different heuristic
	// generation functions
	int GenerateHeuristicSub( const armAngles &sampleArm, const bool quiet,
				  armAngles *goals, const int numGoals,
				  uint16_t *distances,
				  uint16_t *minTipDistances,
				  uint16_t *maxTipDistances,
				  armAngles &lastAdded );
	// function needed for GenerateHeuristic
	// given a file curFile of the frontier of positions at depth
	// curDistance, generates a file of the positions at curDistance+1
	// uses and updates distances[] to avoid re-visiting positions
	// if XXXTipDistances are not NULL, it will update XXXTipDistances
	// to be the min/maximum distance for all positions with that
	// tip position
	uint64_t GenerateNextDepth( FILE *curFile, FILE *nextFile,
				    uint16_t curDistance, uint16_t *distances,
				    uint16_t *minTipDistances,
				    uint16_t *maxTipDistances,
				    armAngles &lastAdded );

	// use a heuristic table
	uint16_t UseHeuristic( armAngles &s, armAngles &g,
			       uint16_t *distances );
	uint16_t UseHeuristic( armAngles &arm, double goalX, double goalY,
			       uint16_t *distances, uint16_t *minTipDistances,
			       uint16_t *maxTipDistances );
};

//typedef UnitSimulation<armAngles, armRotations, MapEnvironment> UnitMapSimulation;

#endif
