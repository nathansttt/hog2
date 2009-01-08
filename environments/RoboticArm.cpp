/*
 *  RoboticArm.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 11/15/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "RoboticArm.h"

bool line2d::crosses(line2d which) const
{
	//input x1,y1 input x2,y2
	//input u1,v1 input u2,v2
	line2d here(start, end);
	double maxx1, maxx2, maxy1, maxy2;
	double minx1, minx2, miny1, miny2;
	if (here.start.x > here.end.x)
	{ maxx1 = here.start.x; minx1 = here.end.x; }
	else
	{ maxx1 = here.end.x; minx1 = here.start.x; }

	if (here.start.y > here.end.y)
	{ maxy1 = here.start.y; miny1 = here.end.y; }
	else
	{ maxy1 = here.end.y; miny1 = here.start.y; }

	if (which.start.x > which.end.x)
	{ maxx2 = which.start.x; minx2 = which.end.x; }
	else
	{ maxx2 = which.end.x; minx2 = which.start.x; }

	if (which.start.y > which.end.y)
	{ maxy2 = which.start.y; miny2 = which.end.y; }
	else
	{ maxy2 = which.end.y; miny2 = which.start.y; }
	
	if (fless(maxx1,minx2) || fless(maxx2, minx1) || fless(maxy1, miny2) || fless(maxy2, miny1))
		return false;
	
	if (fequal(maxx1, minx1)) // this is "here"
	{
		// already know that they share bounding boxes
		// here, they must cross
		if ((maxy2 < maxy1) && (miny2 > miny1))
			return true;
		
		// y = mx + b
		double m = (which.end.y-which.start.y)/(which.end.x-which.start.x);
		double b = which.start.y - m*which.start.x;
		double y = m*here.start.x+b;
		if (fless(y, maxy1) && fgreater(y, miny1)) // on the line
			return true;
		return false;
	}
	if (fequal(maxx2, minx2)) // this is "which"
	{
		// already know that they share bounding boxes
		// here, they must cross
		if ((maxy1 < maxy2) && (miny1 > miny2))
			return true;
		
		// y = mx + b
		double m = (here.end.y-here.start.y)/(here.end.x-here.start.x);
		double b = here.start.y - m*here.start.x;
		double y = m*which.start.x+b;
		if (fless(y, maxy2) && fgreater(y, miny2)) // on the line
			return true;
		return false;
	}
	
	double b1 = (which.end.y-which.start.y)/(which.end.x-which.start.x);// (A)
	double b2 = (here.end.y-here.start.y)/(here.end.x-here.start.x);// (B)
	
	double a1 = which.start.y-b1*which.start.x;
	double a2 = here.start.y-b2*here.start.x;

	if (fequal(b1, b2))
		return false;
	double xi = - (a1-a2)/(b1-b2); //(C)
	double yi = a1+b1*xi;
	// these are actual >= but we exempt points
	if ((fgreater((which.start.x-xi)*(xi-which.end.x), 0)) &&
		(fgreater((here.start.x-xi)*(xi-here.end.x), 0)) &&
		(!fless((which.start.y-yi)*(yi-which.end.y), 0)) &&
		(fgreater((here.start.y-yi)*(yi-here.end.y), 0)))
	{
		//printf("lines cross at (%f, %f)\n",xi,yi);
		return true;
	}
	else {
		return false;
		//print "lines do not cross";
	}
	assert(false);
}

tRotation armRotations::GetRotation(int which) const
{
	switch ((rotations>>(2*which))&0x3)
	{
		case 1: return kRotateCCW;
		case 0: return kNoRotation;
		case 2: return kRotateCW;
	}
	return kNoRotation;
}

void armRotations::SetRotation(int which, tRotation dir)
{
	int value = 0;
	switch (dir)
	{
		case kRotateCW: value = 2; break;
		case kRotateCCW: value = 1; break;
		case kNoRotation: value = 0; break;
	}
	rotations = rotations&((~0x3)<<(2*which))|(value<<2*which);
}

int armAngles::GetAngle(int which) const
{
	return (angles>>(10*which))&0x3FF;
}

void armAngles::SetAngle(int which, int value)
{
	value %= 1024; if( value < 0 ) { value += 1024; }
	value -= value & 1;
	uint64_t mask = 1023ll<<(10*which);
	uint64_t val = ((uint64_t)value)<<(10*which);
	angles = (angles&~mask)|val;
}

int armAngles::GetNumArms() const
{
	return angles>>60;
}

void armAngles::SetNumArms(int count)
{
	uint64_t mask = 0xFFFFFFFFFFFFFFFll;
	uint64_t newCnt = count;
	newCnt = newCnt << 60;
	assert(count <= 6);
	angles = newCnt|(angles&mask);
}

void armAngles::SetGoal(double x, double y)
{
	uint64_t fixedDecx, fixedFracx;
	uint64_t fixedDecy, fixedFracy;
	x += 512; // avoid unsigned issues...
	y += 512;
	fixedDecx = ((uint32_t)x)&0x3FF; // 10 bits
	fixedFracx = (int)((x-(double)fixedDecx)*1024.0*1024.0); // 20 bits
	fixedDecy = ((uint32_t)y)&0x3FF; // 10 bits
	fixedFracy = (int)((y-(double)fixedDecy)*1024.0*1024.0); // 20 bits
	angles = (0xFll<<60)|(fixedDecx<<50)|(fixedFracx<<30)|(fixedDecy<<20)|fixedFracy;
}

void armAngles::GetGoal(double &x, double &y) const
{
	assert(IsGoalState());
	
	x = ((angles>>50)&0x3FF) + (double)((angles>>30)&0xFFFFF)/(1024.0*1024.0);
	y = ((angles>>20)&0x3FF) + (double)((angles)&0xFFFFF)/(1024.0*1024.0);
	x-= 512;
	y-= 512;
}


bool armAngles::IsGoalState() const
{
	return ((angles>>60) == 0xF);
}


RoboticArm::RoboticArm(int dof, double armlength, double fTolerance)
:DOF(dof), armLength(armlength), tolerance(fTolerance)
{
	m_TableComplete = false;
	legalStateTable = NULL;
	legalGoalTable = NULL;
	tipPositionTables = NULL;
	BuildSinCosTables();
	GenerateCPDB();
}

RoboticArm::~RoboticArm()
{
}

void RoboticArm::GetTipPosition( armAngles &s, double &x, double &y )
{
	std::vector<line2d> armSegments;
	GenerateLineSegments( s, armSegments );
	recVec a = armSegments.back().end;
	x = a.x;
	y = a.y;
}

void RoboticArm::GetSuccessors(armAngles &nodeID, std::vector<armAngles> &neighbors)
{
	neighbors.resize(0);
	for (int x = 0; x < nodeID.GetNumArms(); x++)
	{
		armAngles s = nodeID;
		armRotations a;
		a.SetRotation(x, kRotateCW);
		ApplyAction(s, a);

		if (LegalState(s))
			neighbors.push_back(s);
		
		s = nodeID;
		a.SetRotation(x, kRotateCCW);
		ApplyAction(s, a);

		if (LegalState(s))
			neighbors.push_back(s);
	}
}

void RoboticArm::GetActions(armAngles &nodeID, std::vector<armRotations> &actions)
{
	actions.resize(0);
	for (int x = 0; x < nodeID.GetNumArms(); x++)
	{
		armAngles s = nodeID;
		armRotations a;
		a.SetRotation(x, kRotateCW);
		ApplyAction(s, a);
		
		if (LegalState(s))
			actions.push_back(a);
		
		s = nodeID;
		a.SetRotation(x, kRotateCCW);
		ApplyAction(s, a);
		
		if (LegalState(s))
			actions.push_back(a);
	}

//	actions.resize(0);
//	for (int x = 0; x < nodeID.GetNumArms(); x++)
//	{
//		armAngles a(nodeID);
//		a.SetAngle(x, nodeID.GetAngle(x)+2);
//		if (LegalState(a))
//		{
//			armRotations rot;
//			rot.SetRotation(x, kRotateCW);
//			actions.push_back(rot);
//		}
//		a.SetAngle(x, nodeID.GetAngle(x)-2);
//		if (LegalState(a))
//		{
//			armRotations rot;
//			rot.SetRotation(x, kRotateCCW);
//			actions.push_back(rot);
//		}
//	}
}

armRotations RoboticArm::GetAction(armAngles &s1, armAngles &s2)
{
	armRotations ar;
	for (int x = 0; x < s1.GetNumArms(); x++)
	{
		ar.SetRotation(x, (tRotation)(s2.GetAngle(x)-s1.GetAngle(x)));
	}
	return ar;
}

void RoboticArm::ApplyAction(armAngles &s, armRotations dir)
{
	armAngles newState = s;
	for (int x = 0; x < newState.GetNumArms(); x++) {
		newState.SetAngle(x, newState.GetAngle(x)+2*dir.GetRotation(x));
	}
	//if (LegalState(newState))
	s = newState;
}

bool RoboticArm::InvertAction(armRotations &a)
{
	for (int x = 0; x < 6; x++)
		a.SetRotation(x, (tRotation)(-(int)a.GetRotation(x)));
	return true;
}

double RoboticArm::HCost(armAngles &node1, armAngles &node2)
{
	double h;

	if (node1.IsGoalState()) return HCost(node2, node1);
	assert(node2.IsGoalState());

	std::vector<line2d> armSegments1;
	GenerateLineSegments(node1, armSegments1);
	recVec a = armSegments1.back().end;
	double x, y;
	node2.GetGoal(x, y);
	double actDistance = sqrt((x-a.x)*(x-a.x)+(y-a.y)*(y-a.y));
	double movementAmount = (node1.GetNumArms()*armLength*sin(TWOPI*4.0/1024.0));

	h = actDistance / movementAmount;

	uint16_t tableH;
	for( unsigned i = 0; i < distancesTables.size(); ++i ) {
		if( node1.GetNumArms() == tablesNumArms[ i ] ) {
			tableH = UseHeuristic( node1, x, y,
					       distancesTables[ i ],
					       minTipDistancesTables[ i ],
					       maxTipDistancesTables[ i ] );
			if( (double)tableH > h ) {
				h = (double)tableH;
			}
		}
	}

	return h;

#if 0
	recVec a, b;
	if (node1.IsGoalState())
		node1.GetGoal(a.x, a.y);
	else {
		GenerateLineSegments(node1, armSegments);
		a = armSegments.back().end;
	}
	
	if (node2.IsGoalState())
		node2.GetGoal(b.x, b.y);
	else {
		GenerateLineSegments(node2, armSegments);
		b = armSegments.back().end;
	}
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
#endif
}

#if 0
double RoboticArm::GCost(armAngles &node1, armAngles &node2)
{
	recVec a, b;
	if (node1.IsGoalState())
		node1.GetGoal(a.x, a.y);
	else {
		GenerateLineSegments(node1, armSegments);
		a = armSegments.back().end;
	}

	if (node2.IsGoalState())
		node2.GetGoal(b.x, b.y);
	else {
		GenerateLineSegments(node2, armSegments);
		b = armSegments.back().end;
	}
	return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}
#endif

#if 0
double RoboticArm::GCost(armAngles &node1, armRotations &act)
{
	armAngles node2;
	GetNextState(node1, act, node2);
	return GCost(node1, node2);
}
#endif

bool RoboticArm::GoalTest(armAngles &node, armAngles &goal)
{
	assert(goal.IsGoalState());
	GenerateLineSegments(node, armSegments);

	recVec a;
	a = armSegments.back().end;
	double x, y;
	goal.GetGoal(x, y);
	return x-a.x <= tolerance && a.x-x < tolerance
	  && y-a.y <= tolerance && a.y-y < tolerance;
}

uint64_t RoboticArm::GetStateHash(armAngles &node)
{
	return node.angles;
}

uint64_t RoboticArm::GetActionHash(armRotations act)
{
	return act.rotations;
}

void RoboticArm::OpenGLDraw(int)
{
	glBegin(GL_QUADS);
	glColor3f(0, 0, 0.1);
	glVertex3f(-1, -1, 0.1);
	glVertex3f(1, -1, 0.1);
	glVertex3f(1, 1, 0.1);
	glVertex3f(-1, 1, 0.1);
	glEnd();
	
	glColor3f(1, 0, 0);
	for (unsigned int x = 0; x < obstacles.size(); x++)
	{
		DrawLine(obstacles[x]);
	}
}

void RoboticArm::OpenGLDraw(int , armAngles &a)
{
	recVec e;
	if (a.IsGoalState())
	{
		e.z = 0;
		a.GetGoal(e.x, e.y);
	}
	else {
		GenerateLineSegments(a, armSegments);
		
		for (unsigned int x = 0; x < armSegments.size(); x++)
		{
			glColor3f(1, 1, 1);
			//printf("Drawing line segment %d of %d\n", x, armSegments.size());
			DrawLine(armSegments[x]);
		}
		e = armSegments.back().end;
	}
	
	glBegin(GL_LINE_LOOP);
	glColor3f(0, 1.0, 0);
	glVertex3f(e.x+tolerance, e.y+tolerance, 0);
	glVertex3f(e.x-tolerance, e.y+tolerance, 0);
	glVertex3f(e.x-tolerance, e.y-tolerance, 0);
	glVertex3f(e.x+tolerance, e.y-tolerance, 0);
	glEnd();
	
}

void RoboticArm::OpenGLDraw(int, armAngles &, armRotations &)
{
}

void RoboticArm::OpenGLDraw(int, armAngles &, armRotations &, GLfloat, GLfloat, GLfloat)
{
}

void RoboticArm::OpenGLDraw(int, armAngles &, GLfloat, GLfloat, GLfloat)
{
}

void RoboticArm::DrawLine(line2d l)
{
	glBegin(GL_LINES);
	glVertex3f(l.start.x, l.start.y, 0);
	glVertex3f(l.end.x, l.end.y, 0);
	glEnd();
}

void RoboticArm::GetNextState(armAngles &currents, armRotations dir, armAngles &news)
{
	news = currents;
	ApplyAction(news, dir);
}

bool RoboticArm::LegalState(armAngles &a)
{
	if( legalStateTable != NULL ) {
		uint64_t idx;

		idx = ArmAnglesIndex( a );
		if( legalStateTable[ idx >> 3 ]  & ( 1 << ( idx & 7 ) ) ) {
			return true;
		}
		return false;
	}

	GenerateLineSegments(a, armSegments);
	for (unsigned int x = 0; x < armSegments.size(); x++)
	{
		for (unsigned int y = 0; y < obstacles.size(); y++)
		{
			if (armSegments[x].crosses(obstacles[y]))
				return false;
		}
		for (unsigned int y = x+2; y < armSegments.size(); y++)
			if (armSegments[x].crosses(armSegments[y]))
				return false;

		if ((x > 0) && (a.GetAngle(x) == 0))
			return false;
	}
	return true;
}

bool RoboticArm::LegalArmConfig(armAngles &a)
{
	if (m_TableComplete)
		return legals[a.GetAngle(1)][a.GetAngle(2)];
	GenerateLineSegments(a, armSegments);
	for (unsigned int x = 0; x < armSegments.size(); x++)
	{
		for (unsigned int y = x+2; y < armSegments.size(); y++)
			if (armSegments[x].crosses(armSegments[y]))
				return false;
		
		if ((x > 0) && (a.GetAngle(x) == 0))
			return false;
	}
	return true;
}

void RoboticArm::GenerateLineSegments(armAngles &a, std::vector<line2d> &armSegments1)
{
	armSegments1.resize(0);
	for (int x = 0; x < a.GetNumArms(); x++)
	{
		recVec prev;
		recVec start;
		recVec end;

		//double angle = TWOPI*a.GetAngle(x)/1024.0;
		int angle = a.GetAngle(x);
		if (x == 0)
		{
			start.x = 0;
			start.y = 0;
			start.z = 0;
			prev = start;
			prev.y = -armLength;
		}
		else {
			start = armSegments1.back().end;
			prev = armSegments1.back().start;
		}

		// offset to origin
		prev.x -= start.x;
		prev.y -= start.y;
		
		end.x = prev.x*GetCos(angle) - prev.y*GetSin(angle) + start.x;
		end.y = prev.x*GetSin(angle) + prev.y*GetCos(angle) + start.y;
		
		armSegments1.push_back(line2d(start, end));
	}
	assert(armSegments1.size() > 0);
}

double RoboticArm::GetSin(int angle)
{
	return sinTable[angle];
}

double RoboticArm::GetCos(int angle)
{
	return cosTable[angle];
}

void RoboticArm::BuildSinCosTables()
{
	sinTable.resize(1024);
	cosTable.resize(1024);
	for (int x = 0; x < 1024; x++)
	{
		sinTable[x] = sin(TWOPI*(double)x/1024.0);
		cosTable[x] = cos(TWOPI*(double)x/1024.0);
	}
}

void RoboticArm::GenerateCPDB()
{
	int numArms = 3;
	//void GenerateLegalArmConfigs();
	armAngles a;
	a.SetNumArms(numArms);
	a.SetAngle(0, 0);
	// 0, 512, 512
	std::vector<std::vector<bool> > legals;
	legals.resize(512);
	for (unsigned int x = 0; x < legals.size(); x++)
	{
		legals[x].resize(512);
		printf("Building table for %d\n", x);
		for (unsigned int y = 0; y < legals[x].size(); y++)
		{
			a.SetAngle(1, 2*x);
			a.SetAngle(2, 2*y);
			legals[x][y] = LegalArmConfig(a);
		}
	}
	m_TableComplete = true;
}


uint64_t RoboticArm::ArmAnglesIndex( const armAngles &arm )
{
	uint64_t idx;
	int s;

	idx = 0;
	for( s = 0; s < arm.GetNumArms(); ++s ) {
		idx <<= 9;
		idx |= arm.GetAngle( s ) >> 1;
	}

	return idx;
}

int RoboticArm::TipPositionIndex( const double x, const double y,
				  const double minX, const double minY,
				  const double width )
{
	int idx;

	// if we had a guarantee that width was a multiple of
	// tolerance, we could do a bunch of simplification

	idx = (int)floor( ( y - minY ) / tolerance );
	idx *= (int)floor( width / tolerance );
	idx += (int)floor( ( x - minX ) / tolerance );

	return idx;
}

int RoboticArm::WriteArmAngles(FILE *file, armAngles &a)
{
	int s;
	int16_t v;

	v = a.GetNumArms();
	if( fwrite( &v, sizeof( v ), 1, file ) < 1 ) {
		return 0;
	}
	for( s = 0; s < a.GetNumArms(); ++s ) {
		v = a.GetAngle( s );
		if( fwrite( &v, sizeof( v ), 1, file ) < 1 ) {
			return 0;
		}
	}
	return 1;
}

int RoboticArm::ReadArmAngles(FILE *file, armAngles &a)
{
	armAngles angles;
	int s;
	int16_t v;

	if( fread( &v, sizeof( v ), 1, file ) < 1 ) {
		return 0;
	}
	angles.SetNumArms( v );
	for( s = 0; s < angles.GetNumArms(); ++s ) {
		if( fread( &v, sizeof( v ), 1, file ) < 1 ) {
			return 0;
		}
		angles.SetAngle( s, v );
	}

	a = angles;
	return 1;
}

void RoboticArm::UpdateTipDistances( armAngles &arm, uint16_t distance,
				     uint16_t *minTipDistances,
				     uint16_t *maxTipDistances )
{
	int idx;
	double x, y;

	GetTipPosition( arm, x, y );
	idx = TipPositionIndex( x, y, -1.0, -1.0, 2.0 );
	if( distance < minTipDistances[ idx ] ) {
	  minTipDistances[ idx ] = distance;
	}
	if( distance > maxTipDistances[ idx ] ) {
	  maxTipDistances[ idx ] = distance;
	}
}

// this works because of two things
// a) clockwise and counterclockwise are each others inverses
// and both actions are (potentially) legal moves for every arm segment
// b) action costs are uniform, so all children which have not
// been examined are in the next frontier
// returns the number of states in nextFile
uint64_t RoboticArm::GenerateNextDepth( FILE *curFile, FILE *nextFile,
					uint16_t curDistance,
					uint16_t *distances,
					uint16_t *minTipDistances,
					uint16_t *maxTipDistances,
					armAngles &lastAdded )
{
	uint64_t count = 0, idx;
	unsigned i;
	armAngles arm;
	std::vector<armRotations> actions;

	while( ReadArmAngles( curFile, arm ) ) {
		GetActions( arm, actions );
		for( i = 0; i < actions.size(); ++i ) {
			armAngles child = arm;
			ApplyAction( child, actions[ i ] );

			idx = ArmAnglesIndex( child );
			if( distances[ idx ]
			    > curDistance + 1 ) {
				// new arm configuration

				distances[ idx ] = curDistance + 1;
				WriteArmAngles( nextFile, child );
				if( minTipDistances ) {
				  UpdateTipDistances( child, curDistance + 1,
						      minTipDistances,
						      maxTipDistances );
				}
				++count;
			}
		}
	}

	lastAdded = arm;

	return count;
}


/* returns 1 if goal will generate a reasonable heuristic, 0 otherwise */
int RoboticArm::GenerateHeuristicSub( const armAngles &sampleArm,
				      const bool quiet,
				      armAngles *goals, const int numGoals,
				      uint16_t *distances,
				      uint16_t *minTipDistances,
				      uint16_t *maxTipDistances,
				      armAngles &lastAdded )
{
	uint64_t count, i, total;
	int numArms = sampleArm.GetNumArms(), segment, g;
	uint16_t distance;
	armAngles arm;
	FILE *curFile, *nextFile;

	arm.SetNumArms( numArms );

	count = NumArmAnglesIndices( arm );
	for( i = 0; i < count; ++i ) {
		distances[ i ] = 65535;
	}

	if( minTipDistances ) {
		count = NumTipPositionIndices();
		for( i = 0; i < count; ++i ) {
			minTipDistances[ i ] = 65535;
			maxTipDistances[ i ] = 0;
		}
	}

	nextFile = tmpfile();
	assert( nextFile != NULL );

	count = 0;
	total = 0;

	if( !quiet ) {
		printf( "populating depth 0\n" );
	}
	for( segment = 0; segment < numArms; ++segment ) {
		arm.SetAngle( segment, 0 );
	}
	while( 1 ) {
		if( LegalState( arm ) ) {
		  ++total;

		  for( g = 0; g < numGoals; ++g ) {
		    if( GoalTest( arm, goals[ g ] ) ) {
		      // new distance 0 (goal) arm configuration

		      distances[ ArmAnglesIndex( arm ) ] = 0;
		      WriteArmAngles( nextFile, arm );
		      UpdateTipDistances( arm, 0, minTipDistances,
					  maxTipDistances );
		      ++count;
		      break;
		    }
		  }
		}

		// next configuration
		segment = 0;
		do {
			armRotations action;
			action.SetRotation( segment, kRotateCW );
			ApplyAction( arm, action );
			if( arm.GetAngle( segment ) != 0 ) {
				break;
			}
		} while( ++segment < numArms );
		if( segment == numArms ) {
			// tried all configurations
			break;
		}
	}

	printf( "%lu legal states\n", total );

	if( !count ) {
	  fclose( nextFile );
	  return 0;
	}

	distance = 0;
	total = 0;
	do {
		total += count;
		if( !quiet ) {
			printf( "%lu states at distance %u\n",
				count, distance );
		}
		curFile = nextFile;
		rewind( curFile );
		nextFile = tmpfile();

		count = GenerateNextDepth( curFile, nextFile, distance,
					   distances, minTipDistances,
					   maxTipDistances, lastAdded );

		++distance;
		fclose( curFile );
	} while( count );
	fclose( nextFile );

	if( !quiet ) {
		printf( "%lu total states\n", total );
	}

	// there are a number of small disconnected subspaces,
	// so make sure goal doesn't correspond to this subspace!
	if( distances[ ArmAnglesIndex( sampleArm ) ] == 65535 ) {
		// sample arm not reachable!
		return 0;
	}

	return 1;
}

/* this function generates a table-based heuristic function for
   the current environment */
void RoboticArm::GenerateRandomHeuristic( const armAngles &sampleArm )
{
	uint16_t *distances, *minTipDistances, *maxTipDistances;
	armAngles goal, last;

	distances = new uint16_t[ NumArmAnglesIndices( sampleArm ) ];
	minTipDistances = new uint16_t[ NumTipPositionIndices() ];
	maxTipDistances = new uint16_t[ NumTipPositionIndices() ];

	do {
		goal.SetGoal( (double)random()/(double)RAND_MAX * 2.0 - 1.0,
			      (double)random()/(double)RAND_MAX * 2.0 - 1.0 );
	} while( !GenerateHeuristicSub( sampleArm, false, &goal, 1, distances,
					minTipDistances, maxTipDistances,
					last ) );

	distancesTables.push_back( distances );
	minTipDistancesTables.push_back( minTipDistances );
	maxTipDistancesTables.push_back( maxTipDistances );
	tablesNumArms.push_back( sampleArm.GetNumArms() );
}

/* this function generates a table-based heuristic function for
   the current environment */
int RoboticArm::GenerateHeuristic( const armAngles &sampleArm,
				   armAngles &goal )
{
	uint16_t *distances, *minTipDistances, *maxTipDistances;
	armAngles last;

	distances = new uint16_t[ NumArmAnglesIndices( sampleArm ) ];
	minTipDistances = new uint16_t[ NumTipPositionIndices() ];
	maxTipDistances = new uint16_t[ NumTipPositionIndices() ];

	if( !GenerateHeuristicSub( sampleArm, false, &goal, 1, distances,
				   minTipDistances, maxTipDistances, last ) ) {
		delete[] maxTipDistances;
		delete[] minTipDistances;
		delete[] distances;
		return 0;
	}

	distancesTables.push_back( distances );
	minTipDistancesTables.push_back( minTipDistances );
	maxTipDistancesTables.push_back( maxTipDistances );
	tablesNumArms.push_back( sampleArm.GetNumArms() );

	return 1;
}

/* this function generates a table-based heuristic function for
   the current environment */
int RoboticArm::GenerateMaxDistHeuristics( const armAngles &sampleArm,
					   const int numHeuristics )
{
	int i, ret;
	uint16_t *distances, *minTipDistances, *maxTipDistances;
	armAngles goals[ numHeuristics ], last;
	double x, y;

	last = sampleArm;
	GetTipPosition( last, x, y );
	last.SetGoal( x, y );
	for( i = 0; i < numHeuristics; ++i ) {
		distances = new uint16_t[ NumArmAnglesIndices( sampleArm ) ];
		minTipDistances = new uint16_t[ NumTipPositionIndices() ];
		maxTipDistances = new uint16_t[ NumTipPositionIndices() ];

		printf( "generating next goal position\n" );
		if( i ) {
		  ret = GenerateHeuristicSub( sampleArm, true, goals, i,
					      distances, minTipDistances,
					      maxTipDistances, goals[ i ] );
		} else {
		  ret = GenerateHeuristicSub( sampleArm, true, &last, 1,
					      distances, minTipDistances,
					      maxTipDistances, goals[ i ] );
		}
		if( !ret ) {
			delete[] maxTipDistances;
			delete[] minTipDistances;
			delete[] distances;
			return i;
		}
		GetTipPosition( goals[ i ], x, y );
		printf( "new goal position: (%lf,%lf)\n", x, y );
		goals[ i ].SetGoal( x, y );


		if( !GenerateHeuristicSub( sampleArm, false, &goals[ i ], 1,
					   distances, minTipDistances,
					   maxTipDistances, last ) ) {
			delete[] maxTipDistances;
			delete[] minTipDistances;
			delete[] distances;
			return i;
		}

		distancesTables.push_back( distances );
		minTipDistancesTables.push_back( minTipDistances );
		maxTipDistancesTables.push_back( maxTipDistances );
		tablesNumArms.push_back( sampleArm.GetNumArms() );
	}

	return i;
}

uint16_t RoboticArm::UseHeuristic( armAngles &s, armAngles &g,
				   uint16_t *distances )
{
	uint16_t d_s, d_g;

	d_s = distances[ ArmAnglesIndex( s ) ];
	d_g = distances[ ArmAnglesIndex( g ) ];
	if( d_s < d_g ) {
	  return d_g - d_s;
	}
	return d_s - d_g;
}

uint16_t RoboticArm::UseHeuristic( armAngles &arm,
				   double goalX, double goalY,
				   uint16_t *distances,
				   uint16_t *minTipDistances,
				   uint16_t *maxTipDistances )
{
	int32_t mind, maxd, t;	// 32 bits because of the subtraction below
	int i, j, index;
	double x, y;

	mind = 65535;
	maxd = 0;
	for( y = goalY - tolerance, i = 0; i < 3; y += tolerance, ++i ) {
	  for( x = goalX - tolerance, j = 0; j < 3; x += tolerance, ++j ) {
	    index =  TipPositionIndex( x, y, -1.0, -1.0, 2.0 );
	    if( minTipDistances[ index ] < mind ) {
	      mind = minTipDistances[ index ];
	    }
	    if( maxTipDistances[ index ] > maxd ) {
	      maxd = maxTipDistances[ index ];
	    }
	  }
	}

	t = distances[ ArmAnglesIndex( arm ) ];
//printf( "dist(s->g)=%u - max(dist(c->g))=%u\n", (unsigned)t, (unsigned)maxd );
//printf( "min(dist(c->g))=%u - dist(s->g)=%u\n", (unsigned)mind, (unsigned)t );
	maxd = t - maxd;
	mind = mind - t;
	if( mind > maxd ) {
	  maxd = mind;
	}
	if( maxd < 0 ) {
	  maxd = 0;
	}

//if( maxd > 0 ) printf( "%u\n", (unsigned)maxd );

	return maxd;
}

bool RoboticArm::ValidGoalPosition( double goalX, double goalY )
{
	int i, j, index;
	double x, y, tx, ty;

	if( legalGoalTable == NULL ) {
		// no table to use, can't prove anything
		return false;
	}

	for( y = goalY - tolerance, i = 0; i < 3; y += tolerance, ++i ) {
	  for( x = goalX - tolerance, j = 0; j < 3; x += tolerance, ++j ) {
	    tx = x;
	    if( tx < -1.0 ) {
	      tx = -1.0;
	    } else if( tx >= 1.0 ) {
	      tx = 0.999999;
	    }
	    ty = y;
	    if( ty < -1.0 ) {
	      ty = -1.0;
	    } else if( ty >= 1.0 ) {
	      ty = 0.999999;
	    }

	    index =  TipPositionIndex( tx, ty, -1.0, -1.0, 2.0 );
	    if( !( legalGoalTable[ index >> 3 ] & ( 1 << ( index & 7 ) ) ) ) {
	      // given one unreachable square, there's a possibility
	      // that even if the rest are reachable, they're
	      // only reachable because of states that are
	      // actually further than tolerance away from (x,y)
	      return false;
	    }
	  }
	}

	return true;
}

void RoboticArm::GenerateLegalStateTable( armAngles &legalArm )
{
	uint64_t numStates, numTip, i;
	uint16_t *distances, *minTipDistances, *maxTipDistances, distance;
	uint8_t *lst, *lgt;
	armAngles arm, lastAdded;
	FILE *curFile, *nextFile;

	printf( "generating legal state table\n" );

	numStates = NumArmAnglesIndices( legalArm );
	numTip = NumTipPositionIndices();

	lst = new uint8_t[ ( numStates + 7 ) >> 3 ];
	memset( lst, 0, ( numStates + 7 ) >> 3 );
	lgt = new uint8_t[ ( numTip + 7 ) >> 3 ];
	memset( lgt, 0, ( numTip + 7 ) >> 3 );
	distances = new uint16_t[ numStates ];
	for( i = 0; i < numStates; ++i ) {
		distances[ i ] = 65535;
	}
	minTipDistances = new uint16_t[ numTip ];
	maxTipDistances = new uint16_t[ numTip ];
	for( i = 0; i < numTip; ++i ) {
		minTipDistances[ i ] = 65535;
		maxTipDistances[ i ] = 0;
	}

	nextFile = tmpfile();
	assert( nextFile != NULL );

	distances[ ArmAnglesIndex( legalArm ) ] = 0;
	UpdateTipDistances( legalArm, 0, minTipDistances, maxTipDistances );
	WriteArmAngles( nextFile, legalArm );

	distance = 0;
	do {
		curFile = nextFile;
		rewind( curFile );
		nextFile = tmpfile();

		i = GenerateNextDepth( curFile, nextFile, distance,
				       distances, minTipDistances,
				       maxTipDistances, lastAdded );

		++distance;
		fclose( curFile );
	} while( i );
	fclose( nextFile );
	printf( "done: maximum distance of %d from chosen state\n",
		(int)distance );

	for( i = 0; i < numStates; ++i ) {
		if( distances[ i ] < 65535 ) {
			lst[ i >> 3 ] |= 1 << ( i & 7 );
		}
	}

	for( i = 0; i < numTip; ++i ) {
		if( minTipDistances[ i ]
		    <= maxTipDistances[ i ] ) {
			lgt[ i >> 3 ] |= 1 << ( i & 7 );
		}
	}

	legalStateTable = lst;
	legalGoalTable = lgt;
	delete[] maxTipDistances;
	delete[] minTipDistances;
	delete[] distances;
}

void RoboticArm::GenerateTipPositionTables( armAngles &sampleArm )
{
	int numArms = sampleArm.GetNumArms(), segment;
	double x, y;
	armAngles arm;

	printf( "generating tip position table\n" );

	if( tipPositionTables != NULL ) {
		return;
	}

	tipPositionTables = new std::vector<armAngles>
	  [ NumTipPositionIndices() ];

	arm.SetNumArms( numArms );

	for( segment = 0; segment < numArms; ++segment ) {
		arm.SetAngle( segment, 0 );
	}
	while( 1 ) {
		if( LegalState( arm ) ) {

		  GetTipPosition( arm, x, y );
		  tipPositionTables
		    [ TipPositionIndex( x, y, -1.0, -1.0, 2.0 ) ]
		    .push_back( arm );
		}

		// next configuration
		segment = 0;
		do {
			armRotations action;
			action.SetRotation( segment, kRotateCW );
			ApplyAction( arm, action );
			if( arm.GetAngle( segment ) != 0 ) {
				break;
			}
		} while( ++segment < numArms );
		if( segment == numArms ) {
			// tried all configurations
			break;
		}
	}

}
