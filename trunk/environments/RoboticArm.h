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
#include "ConfigEnvironment.h"

#include <cassert>

//#include "BaseMapOccupancyInterface.h"



class armAngles {
public:
	armAngles() { angles = 0; }
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

class RoboticArmHeuristic {
public:
	virtual ~RoboticArmHeuristic() {}
	virtual double HCost(const armAngles &node1, const armAngles &node2) = 0;
};

class RoboticArm : public SearchEnvironment<armAngles, armRotations>
{
public:
	RoboticArm(int DOF, double armLength, double tolerance = 0.01);
	virtual ~RoboticArm();

	double GetTolerance() const { return tolerance; }
	void GetTipPosition( armAngles &s, double &x, double &y );

	void AddObstacle(line2d obs);
	void GetSuccessors(const armAngles &nodeID, std::vector<armAngles> &neighbors) const;
	void GetActions(const armAngles &nodeID, std::vector<armRotations> &actions) const;
	armRotations GetAction(const armAngles &s1, const armAngles &s2) const;
	virtual void ApplyAction(armAngles &s, armRotations dir) const;
	armAngles GetRandomState();

	virtual bool InvertAction(armRotations &a) const;

	void AddHeuristic(RoboticArmHeuristic *h) { heuristics.push_back(h); }

	virtual double HCost(const armAngles &){
		printf("Single State HCost Failure: method not implemented for RoboticArm\n");
		exit(0); return -1.0;}

	virtual double HCost(const armAngles &node1, const armAngles &node2);

	virtual double GCost(const armAngles &, const armAngles &) { return 1; }
	virtual double GCost(const armAngles &, const armRotations &) { return 1; }
	bool GoalTest(armAngles &node, armAngles &goal);
	uint64_t GetStateHash(const armAngles &node) const;
	uint64_t GetActionHash(armRotations act) const;

	virtual void OpenGLDraw() const;
	virtual void OpenGLDraw(const armAngles &l) const;
	virtual void OpenGLDraw(const armAngles &, const armRotations &) const;
	virtual void OpenGLDraw(const armAngles&, const armAngles&, float) const {}
//	virtual void OpenGLDraw(const armAngles &, const armRotations &, GLfloat r, GLfloat g, GLfloat b) const;
//	virtual void OpenGLDraw(const armAngles &l, GLfloat r, GLfloat g, GLfloat b) const;

	virtual void GetNextState(armAngles &currents, armRotations dir, armAngles &news) const;

	bool LegalState(armAngles &a) const;
	bool LegalArmConfig(armAngles &a) const;

	void StoreGoal(armAngles &) {}
	void ClearGoal(){}
	bool IsGoalStored(){return false;}
	virtual bool GoalTest(armAngles &){
		printf("Single State Goal Test Failure: method not implemented for RoboticArm\n");
		exit(0); return false;}

private:
	void DrawLine(line2d l) const;
	void GenerateLineSegments(const armAngles &a, std::vector<line2d> &armSegments) const;

	int DOF;
	double armLength, tolerance;
	std::vector<std::vector<bool> > legals;

	double GetSin(int angle) const;
	double GetCos(int angle) const;
	void BuildSinCosTables();
	std::vector<double> sinTable;
	std::vector<double> cosTable;
	std::vector<line2d> obstacles;
	mutable std::vector<line2d> armSegments;

	std::vector<recVec> states;

	std::vector<RoboticArmHeuristic *> heuristics;
	ConfigEnvironment *ce;
};

class ArmToArmHeuristic : public RoboticArmHeuristic {
public:
	ArmToArmHeuristic(RoboticArm *r, armAngles &initial, bool optimize = false);
	virtual ~ArmToArmHeuristic() {}
	double HCost(const armAngles &node1, const armAngles &node2);
	void AddDiffTable();
	bool IsLegalState(armAngles &arm);
	const std::vector<armAngles> &GetTipPositions(double x, double y)
	{ return tipPositionTables[TipPositionIndex(x, y)]; }
private:
	armAngles SelectStartNode();
	int TipPositionIndex(const double x, const double y,
						 const double minX = -1.0, const double minY = -1.0,
						 const double width = 2.0);
	void GenerateLegalStates(armAngles &init);
	bool optimizeLocations;
	RoboticArm *ra;
	std::vector<std::vector<uint16_t> > distances;
	std::vector<armAngles> canonicalStates;
	std::vector<std::vector<armAngles> > tipPositionTables;
	std::vector<bool> legalStates;
};

class ArmToTipHeuristic : public RoboticArmHeuristic {
public:
	ArmToTipHeuristic(RoboticArm *r);
	virtual ~ArmToTipHeuristic() {}
	double HCost(const armAngles &node1, const armAngles &node2);

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
	void GenerateCPDB();


	RoboticArm *ra;
	bool m_TableComplete;

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
		int count = (int)ceil( 2.0 / ra->GetTolerance() );
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
	uint16_t UseHeuristic(const armAngles &s, armAngles &g,
						  uint16_t *distances );
	uint16_t UseHeuristic(const armAngles &arm, double goalX, double goalY,
						  uint16_t *distances, uint16_t *minTipDistances,
						  uint16_t *maxTipDistances );
};

//typedef UnitSimulation<armAngles, armRotations, MapEnvironment> UnitMapSimulation;

#endif
