//
//  RC.cpp (Alternate Rubik's Cube implementation)
//
//  Created by Nathan Sturtevant on 7/9/21.
//

#include "RC.h"
#include <cassert>
#include <cstdio>
#include <algorithm>
#include <string>

// Helpers (for calculation)
void EditPointMult(float (&vec)[3][3], Graphics::point & p);
void MakeMatrixRotX(float (&a)[3][3], float angle);
void MakeMatrixRotY(float (&a)[3][3], float angle);
void MakeMatrixRotZ(float (&a)[3][3], float angle);
float array3 [3] = {0,0,0};
float rotation [3][3] = { {0,0,0}, {0,0,0}, {0,0,0} };
std::vector<int> furthestZ(0);
//std::vector<std::vector<float>> rotation(3,std::vector<float>(3));

/*===============================================================================================================================================================================
 * 			CUBIE CLASS
 ===============================================================================================================================================================================*/

void Cubie::Initialize(int ind)
{
	index = ind;
	int xPos = 0, yPos = 0, zPos = 0;
	//Cube index (out of 26)

	int in = index;
	if (in >= 13) in++; //Readd middle cubie for init ease
		//Initialize face colors, determine position based on index
		for (int i = 0; i < 6; i++)
		{
			faceCols[i] = inCol;
			facesShown[i] = false;
		}
	//If its cube #0,1,2,9,10,11,18,19,20, make face 0 white
	for (int i = 0; i < 3; i++)
	{
		if (in >= i*9 && in <= i*9 + 2)
		{
			faceCols[0] = faceColors[0];
			facesShown[0] = true;
			yPos--;
		}
	}
	//If its cube #0-8, make face 1 blue
	if (in >= 0 && in <= 8)
	{
		faceCols[1] = faceColors[1];
		facesShown[1] = true;
		zPos--;
	}
	//If its cube # is a multiple of 3 (including 0), make face 2 red
	if (in % 3 == 0)
	{
		faceCols[2] = faceColors[2];
		facesShown[2] = true;
		xPos--;
	}
	//If its cube #18-26, make face 3 green
	if (in >= 18 && in <= 26)
	{
		faceCols[3] = faceColors[3];
		facesShown[3] = true;
		zPos++;
		//std::cout << in << '\n';
	}
	//If its cube # - 2 is multiple of 3, make face 4 orange
	if ((in-2) % 3 == 0)
	{
		faceCols[4] = faceColors[4];
		facesShown[4] = true;
		xPos++;
	}
	//If its cube #6,7,8,15,16,17,24,25,26, make face 5 yellow
	for (int i = 0; i < 3; i++)
	{
		if (in >= i*9 + 6 && in <= i*9 + 8)
		{
			faceCols[5] = faceColors[5];
			facesShown[5] = true;
			yPos++;
		}
	}

	//Initialize the 8 points
	for (int i = 0; i < 8; i++)
	{
		points.push_back(Graphics::point(0,0,0));
		points[i].x = -0.5f * sideLen;// + xPos * sideLen;
		if ((i%4) == 1 || (i%4) == 2)
			points[i].x += 1.f * sideLen;
		points[i].y = (-0.5f + (1.f * ((i/2)%2))) * sideLen;// + yPos * sideLen; //y
		points[i].z = (-0.5f + (1.f * (i/4))) * sideLen;// + zPos * sideLen; //z
		
		basePoints.push_back(Graphics::point(points[i].x,points[i].y,points[i].z));
	}
	//Initialize the 4 face points for all 6 faces (24)
	for (int i = 0; i < 6; i++)
	{
		facePoints.push_back(std::vector<Graphics::point>());
		baseFacePoints.push_back(std::vector<Graphics::point>());
		for (int j = 0; j < 4; j++)
		{
			float x, y, z;
			if (i == 2 || i == 4) x = 1.0f;
				else x = faceSize;
			if (i == 0 || i == 5) y = 1.0f;
				else y = faceSize;
			if (i == 1 || i == 3) z = 1.0f;
				else z = faceSize;

			Graphics::point p = points[pointsOnFace[i][j]];
			x *= p.x;
			y *= p.y;
			z *= p.z;
			facePoints[i].push_back(Graphics::point(x,y,z));
			baseFacePoints[i].push_back(Graphics::point(x,y,z));
		}
	}
	//Move all points (faces and cube) by coord
	for (int i = 0; i < 8; i++)
	{
		points[i].x += xPos * sideLen;
		points[i].y += yPos * sideLen;
		points[i].z += zPos * sideLen;
		basePoints[i].x += xPos * sideLen;
		basePoints[i].y += yPos * sideLen;
		basePoints[i].z += zPos * sideLen;
	}
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			facePoints[i][j].x += xPos * sideLen;
			facePoints[i][j].y += yPos * sideLen;
			facePoints[i][j].z += zPos * sideLen;
			baseFacePoints[i][j].x += xPos * sideLen;
			baseFacePoints[i][j].y += yPos * sideLen;
			baseFacePoints[i][j].z += zPos * sideLen;
		}
	}
}

/*
 * 
 */
void Cubie::Draw(Graphics::Display &display)
{
	//	Identify which faces will be hidden from sight (Find which points are the furthest back)
	float highest;
	furthestZ.clear();
	furthestZ.push_back(0);
	highest = points[0].z;
	for (int i = 1; i < 8; i++)
	{
		if (points[i].z > highest) {
			furthestZ.clear();
			furthestZ.push_back(i);
			highest = points[i].z;
			continue;
		}
		if (points[i].z == highest) {
			furthestZ.push_back(i);
		}
	}
	int facesDrawn = 0;
	for (int i = 0; i < 6; i++)
	{
		if (facesShown[i])
		{
			bool drawThis = true;

			for (int pInd : pointsOnFace[i])
			{
				for (int check : furthestZ)
				{
					if (pInd == check)
					{
						//This face shouldn't be shown
						drawThis = false;
						break;
					}
				}
			}

			//Draw face if not on the back
			if (drawThis)
			{
				facesDrawn++;
				DrawFace(display, i);
			}
		}
	}
}

/* Given points (in a correct order)
 * Draw a polygon (face of a cubie)
 */
void Cubie::DrawFace(Graphics::Display &display, int index) //face Index (0-5)
{
	display.FillTriangle( points[pointsOnFace[index][0]], points[pointsOnFace[index][1]],
			points[pointsOnFace[index][2]], outCol);
	display.FillTriangle( points[pointsOnFace[index][0]], points[pointsOnFace[index][2]],
			points[pointsOnFace[index][3]], outCol);

	display.FillTriangle( facePoints[index][0], facePoints[index][1],
			facePoints[index][2], faceCols[index]);
	display.FillTriangle( facePoints[index][0], facePoints[index][2],
			facePoints[index][3], faceCols[index]);
}

/*
 * 
 */
void Cubie::RotateRelative(float angle [3])
{
	for (Graphics::point & point : points) //!!!!
	{
		MakeMatrixRotX(rotation, angle[0]);
		EditPointMult(rotation, point);
		MakeMatrixRotY(rotation, angle[1]);
		EditPointMult(rotation, point);
		MakeMatrixRotZ(rotation, angle[2]);
		EditPointMult(rotation, point);
	}
	for (int i = 0; i < 6; i++) //!!!!
	{
		MakeMatrixRotX(rotation, angle[0]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, facePoints[i][j]);
		MakeMatrixRotY(rotation, angle[1]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, facePoints[i][j]);
		MakeMatrixRotZ(rotation, angle[2]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, facePoints[i][j]);
	}
}

/*
 * 
 */
void Cubie::RotateBase(float angle [3])
{
	for (Graphics::point & point : basePoints)
	{
		MakeMatrixRotX(rotation, angle[0]);
		EditPointMult(rotation, point);
		MakeMatrixRotY(rotation, angle[1]);
		EditPointMult(rotation, point);
		MakeMatrixRotZ(rotation, angle[2]);
		EditPointMult(rotation, point);
	}
	for (int i = 0; i < 6; i++)
	{
		MakeMatrixRotX(rotation, angle[0]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, baseFacePoints[i][j]);
		MakeMatrixRotY(rotation, angle[1]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, baseFacePoints[i][j]);
		MakeMatrixRotZ(rotation, angle[2]);
		for (int j = 0; j < 4; j++)
			EditPointMult(rotation, baseFacePoints[i][j]);
	}
}

/*
 * 
 */
void Cubie::RotateFacePos(bool clockwise, int axis)
{
	int temp;
	if (clockwise)
	{
		temp = faceInPos[faceOrderByAxis[axis][3]];
		for (int i = 3; i > 0; i--)
		{
			faceInPos[faceOrderByAxis[axis][i]] = faceInPos[faceOrderByAxis[axis][i-1]];
		}
		faceInPos[faceOrderByAxis[axis][0]] = temp;
	} else
	{
		temp = faceInPos[faceOrderByAxis[axis][0]];
		for (int i = 0; i < 3; i++)
		{
			faceInPos[faceOrderByAxis[axis][i]] = faceInPos[faceOrderByAxis[axis][i+1]];
		}
		faceInPos[faceOrderByAxis[axis][3]] = temp;
	}
}

/*
 * 
 */
void Cubie::ResetToBase()
{
	for (int i = 0; i < 8; i++)
	{
		points[i].x = basePoints[i].x;
		points[i].y = basePoints[i].y;
		points[i].z = basePoints[i].z;	
	}
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 4; j++)
		{
			facePoints[i][j].x = baseFacePoints[i][j].x;
			facePoints[i][j].y = baseFacePoints[i][j].y;
			facePoints[i][j].z = baseFacePoints[i][j].z;
		}
	}
}

/*
 * 
 */
void Cubie::ResetVisibleFace()
{
	if (blackFaceToReset != -1)
	{
		facesShown[faceInPos[blackFaceToReset]] = false;
		blackFaceToReset = -1;
	}
}

/* Called when a rotation is made. cubies must make an inner black face
 * visible while rotation is in motion (which must be invisible once it's complete)
 */
void Cubie::SetFacePositionVisible(bool toggle, int position)
{
	facesShown[faceInPos[position]] = toggle;
	blackFaceToReset = position;
}

/*===============================================================================================================================================================================
 * 			RCSTATE CLASS
 ===============================================================================================================================================================================*/

/*
 * 
 */
void RCState::RotateFace(int face, int move)
{
	// Move the cubes
	if (move != 2)
	{
		bool forward;
		if (move == 0) forward = true;
		else forward = false;
		
		ShiftPositions(edgesOnFace[face], forward);
		ShiftPositions(cornersOnFace[face], forward);
	} else //180* turn
	{
		SwapPositions(edgesOnFace[face][0], edgesOnFace[face][2]);
		SwapPositions(edgesOnFace[face][1], edgesOnFace[face][3]);
		SwapPositions(cornersOnFace[face][0], cornersOnFace[face][2]);
		SwapPositions(cornersOnFace[face][1], cornersOnFace[face][3]);
	}
	
	//Rotate the Cubes
	RotateEdges(face, move);
	RotateCorners(face, move);
}

/*
 * 
 */
void RCState::RotateEdges(int face, int move)
{
	if (move != 2 && (face == 1 || face == 3))
	{
		for (int i = 0; i < 4; i++)
		{
			rotation[edgesOnFace[face][i]] = 1 - rotation[edgesOnFace[face][i]];
		}
	}
}

/*
 * 
 */
void RCState::RotateCorners(int face, int move)
{
	if (move != 2)
	{
		int index;
		if (face % 5 == 0) index = 0;
		else if (face % 2 == 1) index = 1;
		else index = 2;
		
		if (index != 0)
		{
			for (int i = 0; i < 4; i++)
			{
				int rot = rotation[cornersOnFace[face][i]] + index;
				if (rot > 2) rot -= 3;
				rotation[cornersOnFace[face][i]] = rot;
			}
		}
	}
}

/*
 * 
 */
void RCState::SwapPositions(int p1, int p2)
{
	int temp = indices[p1];
	indices[p1] = indices[p2];
	indices[p2] = temp;
}

/*
 * 
 */
void RCState::ShiftPositions(int (&arr)[4], bool forward)
{
	int temp;
	if (forward) // clockwise
	{
		temp = indices[arr[3]];
		for (int i = 3; i > 0; i--)
		{
			indices[arr[i]] = indices[arr[i-1]];
		}
		indices[0] = temp;
	} else //CCW
	{
		temp = indices[arr[0]];
		for (int i = 0; i < 3; i++)
		{
			indices[arr[i]] = indices[arr[i+1]];
		}
		indices[3] = temp;
	}
}

/*===============================================================================================================================================================================
 * 			RC CLASS
 ===============================================================================================================================================================================*/

/*
 * 
 */
void RC::DrawCubies(Graphics::Display &display)
{
	if (rotating)
	{
		DrawCubiesRotating(display);
	} else
	{
		for (int i = 0; i < 26; i++)
		{
			cubies[i].Draw(display);
		}
	}
}

/*
 * 
 */
void RC::DrawCubiesRotating(Graphics::Display &display)
{
	bool jumped = false;
	if (rotatingFaceBehind)
	{
		 goto drawFace;
	} else goto drawCubie;
	
	drawFace:
		for (int i = 0; i < 9; i++)
			cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].Draw(display);	
		if (!jumped)
		{
			jumped = true;
			goto drawCubie;
		}
		return;
	drawCubie:
		for (int i = 0; i < 17; i++)
			cubies[cubieInPos[notInFaceTurning[faceTurning][i]]].Draw(display);
		if (!jumped)
		{
			jumped = true;
			goto drawFace;
		}
		return;
}

/*
 * 
 */
void RC::RotateCubies(float add[3])
{
	for (int i = 0; i < 3; i++)
	{
		rotationTotal[i] += add[i];
	}
	for (int i = 0; i < 26; i++)
	{
		cubies[i].ResetToBase();
		cubies[i].RotateRelative(rotationTotal);
	}
}

/*
 * 
 */
void RC::RotateFace(int face, int move)
{
	if (!rotating)
	{
		bool clockwise = true;
		int axis;
		turnArr[0] = 0;
		turnArr[1] = 0;
		turnArr[2] = 0;
		rotating = true;
		faceTurning = face;
		rotProgress = 0;
		
		float add = piOver2;
		if (face == 0 || face == 3 || face == 4)
		{
			add *= -1;
		}
		if (face == 5 || face == 3 || face == 4 )
		{
			clockwise = !clockwise;
		}
		
		// Move cubies on face
		int temp, temp2;
		
		if (move == 0) //CW
		{
			temp = cubieInPos[cubiesOnFace[face][edgeOrder[3]]];
			temp2 = cubieInPos[cubiesOnFace[face][cornerOrder[3]]];
			for (int i = 3; i > 0; i--)
			{
				cubieInPos[cubiesOnFace[face][edgeOrder[i]]] = cubieInPos[cubiesOnFace[face][edgeOrder[i-1]]];
				cubieInPos[cubiesOnFace[face][cornerOrder[i]]] = cubieInPos[cubiesOnFace[face][cornerOrder[i-1]]];
			}
			cubieInPos[cubiesOnFace[face][edgeOrder[0]]] = temp;
			cubieInPos[cubiesOnFace[face][cornerOrder[0]]] = temp2;
		} else if (move == 1) //CCW
		{
			add *= -1;
			
			temp = cubieInPos[cubiesOnFace[face][edgeOrder[0]]];
			temp2 = cubieInPos[cubiesOnFace[face][cornerOrder[0]]];
			for (int i = 0; i < 3; i++)
			{
				cubieInPos[cubiesOnFace[face][edgeOrder[i]]] = cubieInPos[cubiesOnFace[face][edgeOrder[i+1]]];
				cubieInPos[cubiesOnFace[face][cornerOrder[i]]] = cubieInPos[cubiesOnFace[face][cornerOrder[i+1]]];
			}
			cubieInPos[cubiesOnFace[face][edgeOrder[3]]] = temp;
			cubieInPos[cubiesOnFace[face][cornerOrder[3]]] = temp2;
			
			clockwise = !clockwise;
		} else //180
		{
			add *= 2;
			
			for (int i = 0; i < 2; i++)
			{
				temp = cubieInPos[cubiesOnFace[face][edgeOrder[i]]];
				temp2 = cubieInPos[cubiesOnFace[face][cornerOrder[i]]];
				cubieInPos[cubiesOnFace[face][edgeOrder[i]]] = cubieInPos[cubiesOnFace[face][edgeOrder[i+2]]];
				cubieInPos[cubiesOnFace[face][cornerOrder[i]]] = cubieInPos[cubiesOnFace[face][cornerOrder[i+2]]];
				cubieInPos[cubiesOnFace[face][edgeOrder[i+2]]] = temp;
				cubieInPos[cubiesOnFace[face][cornerOrder[i+2]]] = temp2;
			}
		}
		//Set direction and distance of turn
		if (face%5 == 0) {
			turnArr[1] = add;
			axis = 1;
		} else if (face == 1 || face == 3) {
			turnArr[2] = add;
			axis = 2;
		} else {
			turnArr[0] = add;
			axis = 0;
		}
		for (int i = 0; i < 9; i++)
		{
			//Edit the base to be FULLY rotated
			cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateBase(turnArr);
			
			//Rotate Cubie face positions
			cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateFacePos(clockwise, axis);
			if (move == 2) cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateFacePos(clockwise, axis);
			
			//Set black face to be visible on each rotated cubie
			cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].SetFacePositionVisible(true, faceBlackUnderside[faceTurning]);
			
			//Set black face to be visible on each cubie now exposed by the turn face rotating
			int toEdit = cubiesOnFace[faceTurning][i];
			if (toEdit >= 13) toEdit++;
			toEdit += fromFaceToCenter[faceTurning];
			if (toEdit != 13)
			{
				if (toEdit >= 14) toEdit--;
				cubies[cubieInPos[toEdit]].SetFacePositionVisible(true, faceTurning);
			}
		}
	}
}

/*
 * 
 */
void RC::InterpFaceRot(float progress)
{
	for (int i = 0; i < 3; i++)
	{
		//Inbetween val
		interpArr[i] = turnArr[i] * (1-progress) * -1;
	}
	// Only turns the 9 face cubies
	for (int i = 0; i < 9; i++) 
	{
		//Reset Rotation
		cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].ResetToBase();
		//Apply interpolated face rotation to straight-on cubie
		cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateRelative(interpArr);
		//Apply total rotation back
		cubies[cubieInPos[cubiesOnFace[faceTurning][i]]].RotateRelative(rotationTotal);
	}
	//Turn the rest of the cubie to match
	for (int i = 0; i < 17; i++)
	{
		cubies[cubieInPos[notInFaceTurning[faceTurning][i]]].ResetToBase();
		cubies[cubieInPos[notInFaceTurning[faceTurning][i]]].RotateRelative(rotationTotal);
	}
	
	//Determing whether the center cubie of the face is behind a z of 0
	float centerCubieZ = (cubies[cubieInPos[cubiesOnFace[faceTurning][4]]].points[0].z - 
			cubies[cubieInPos[cubiesOnFace[faceTurning][4]]].points[6].z)/2 + 
			cubies[cubieInPos[cubiesOnFace[faceTurning][4]]].points[6].z;
	rotatingFaceBehind = (centerCubieZ >= 0); //if it is behind (larger than) 0, it is drawn second	
}

/*
 * 
 */
void RC::TestUpdate()
{
	float testRot [3];
	if (passiveRot)
		for (int i = 0; i < 3; i++)
		{
			//if (i == 1)
			testRot[i] = 0.03f;
		}
	
	if (rotating)
	{
		for (int i = 0; i < 3; i++)
		{
			rotationTotal[i] += testRot[i];
		}
		
		rotProgress += turnSpd;
		if (rotProgress > 1)
		{
			rotating = false;
			rotProgress = 1;
			for (int i = 0; i < 26; i++)
			{
				cubies[i].ResetVisibleFace();
			}
		}
		InterpFaceRot(rotProgress);
	} else
	{
		RotateCubies(testRot);
	}
}

void RC::GetSuccessors(const RCState &nodeID, std::vector<RCState> &neighbors) const
{
	neighbors.resize(18);
	for (int x = 0; x < 18; x++)
	{
		GetNextState(nodeID, x, neighbors[x]);
	}
}

void RC::GetPrunedActions(const RCState &nodeID, RCAction lastAction, std::vector<RCAction> &actions) const
{
	actions.resize(0);
	for (int x = 0; x < 18; x++)
	{
		// 1. after any face you can't turn the same face again
		if (x/3 == lastAction/3)
			continue;
		
		// 2. after faces 5, 4, 3 you can't turn 0, 2, 1 respectively
		if ((1 == (lastAction/3)%2) &&
			(x/3+1 == lastAction/3))
			continue;
		
		actions.push_back(x);
	}
}


void RC::GetActions(const RCState &nodeID, std::vector<RCAction> &actions) const
{
	actions.resize(0);
	if (!pruneSuccessors || history.size() == 0)
	{
		for (int x = 0; x < 18; x++)
			actions.push_back(x);
	}
	else {
		// 0, 5, 2, 4, 1, 3

		for (int x = 0; x < 18; x++)
		{
			// 1. after any face you can't turn the same face again
			if (x/3 == history.back()/3)
				continue;

			// 2. after faces 5, 4, 3 you can't turn 0, 2, 1 respectively
			if ((1 == (history.back()/3)%2) &&
				(x/3+1 == history.back()/3))
				continue;
			
			actions.push_back(x);
		}
	}
//	std::random_shuffle(actions.begin(), actions.end());
}

RCAction RC::GetAction(const RCState &s1, const RCState &s2) const
{
	//std::vector<RCAction> succ;
	//GetActions(s1, succ);
	RCState tmp;
	for (int x = 0; x < 18; x++)
	{
		GetNextState(s1, x, tmp);
		if (tmp == s2)
			return x;
	}
	assert(false);
	return 0;
}

void RC::ApplyAction(RCState &s, RCAction a) const
{
	// TODO: Write
}

void RC::UndoAction(RCState &s, RCAction a) const
{
	// TODO: Write
}

void RC::GetNextState(const RCState &s1, RCAction a, RCState &s2) const
{
	s2 = s1;
	ApplyAction(s2, a);
}

bool RC::InvertAction(RCAction &a) const
{
	// TODO: Test
	if (2 == a%3)
		return true;
	if (1 == a%3)
	{
		a -= 1;
		return true;
	}
	a += 1;
	return true;
	return false;
}

double RC::HCost(const RCState &node1, const RCState &node2, double parentHCost) const
{
	return HCost(node1, node2);
}

/** Heuristic value between two arbitrary nodes. **/
double RC::HCost(const RCState &node1, const RCState &node2) const
{
	return 0;
}

/** Heuristic value between node and the stored goal. Asserts that the
 goal is stored **/
double RC::HCost(const RCState &node) const
{
	return HCost(node, node);
}

bool RC::GoalTest(const RCState &node, const RCState &goal) const
{
	return node == goal;
}

/** Goal Test if the goal is stored **/
bool RC::GoalTest(const RCState &node) const
{
	assert(false);
	return false;
}

uint64_t RC::GetStateHash(const RCState &node) const
{
	// TODO: write
	return 0;
}

void RC::GetStateFromHash(uint64_t hash, RCState &node) const
{
// TODO: write
}

void RC::OpenGLDraw() const
{
}

void RC::OpenGLDraw(const RCState&s) const
{
//	OpenGLDrawCubeBackground();
//	e.OpenGLDraw(s.edge);
//	c.OpenGLDraw(s.corner);
//	OpenGLDrawCenters();
}

void RC::OpenGLDrawCorners(const RCState&s) const
{
//	OpenGLDrawCubeBackground();
//	c.OpenGLDraw(s.corner);
//	OpenGLDrawCenters();
}

void RC::OpenGLDrawEdges(const RCState&s) const
{
//	OpenGLDrawCubeBackground();
////	Rubik7EdgeState e7tmp;
////	s.edge7.GetDual(e7tmp);
//	e.OpenGLDraw(s.edge);
//	OpenGLDrawCenters();
}

void RC::OpenGLDrawEdgeDual(const RCState&s) const
{
//	OpenGLDrawCubeBackground();
//	s.edge.GetDual(dual);
//	e.OpenGLDraw(dual);
//	OpenGLDrawCenters();
}

void RC::OpenGLDrawCubeBackground() const
{
//{
////	return;
//
//	float scale = 0.3;
//	float offset = 0.95*scale/3.0;
//	glBegin(GL_QUADS);
//
//	glColor3f(0,0,0);
//	offset = scale/3.0;
//	scale*=0.99;
//	glVertex3f(-3.0*offset, -scale, -3.0*offset);
//	glVertex3f(3.0*offset, -scale, -3.0*offset);
//	glVertex3f(3.0*offset, -scale, 3.0*offset);
//	glVertex3f(-3.0*offset, -scale, 3.0*offset);
//
//	glVertex3f(-3.0*offset, scale, -3.0*offset);
//	glVertex3f(3.0*offset, scale, -3.0*offset);
//	glVertex3f(3.0*offset, scale, 3.0*offset);
//	glVertex3f(-3.0*offset, scale, 3.0*offset);
//
//	glVertex3f(-scale, -3.0*offset, -3.0*offset);
//	glVertex3f(-scale, 3.0*offset, -3.0*offset);
//	glVertex3f(-scale, 3.0*offset, 3.0*offset);
//	glVertex3f(-scale, -3.0*offset, 3.0*offset);
//
//	//	SetFaceColor(3);
//	glVertex3f(scale, -3.0*offset, -3.0*offset);
//	glVertex3f(scale, 3.0*offset, -3.0*offset);
//	glVertex3f(scale, 3.0*offset, 3.0*offset);
//	glVertex3f(scale, -3.0*offset, 3.0*offset);
//
//	//	SetFaceColor(2);
//	glVertex3f(-3.0*offset, -3.0*offset, -scale);
//	glVertex3f(3.0*offset, -3.0*offset, -scale);
//	glVertex3f(3.0*offset, 3.0*offset, -scale);
//	glVertex3f(-3.0*offset, 3.0*offset, -scale);
//
//	//	SetFaceColor(4);
//	glVertex3f(-3.0*offset, -3.0*offset, scale);
//	glVertex3f(3.0*offset, -3.0*offset, scale);
//	glVertex3f(3.0*offset, 3.0*offset, scale);
//	glVertex3f(-3.0*offset, 3.0*offset, scale);
//	glEnd();
//}
}

void RC::OpenGLDrawCenters() const
{
//	glBegin(GL_QUADS);
//	OpenGLDrawCube(4);
//	OpenGLDrawCube(10);
//	OpenGLDrawCube(12);
//	OpenGLDrawCube(14);
//	OpenGLDrawCube(16);
//	OpenGLDrawCube(22);
//	glEnd();
}

void RC::OpenGLDrawCube(int cube) const
{
//{
//	float scale = 0.3;
//	float offset = 0.95*scale/3.0;
//	const float offset2 = scale/3.0;
//	const float epsilon = 0.002;
//
//	switch (cube)
//	{
//		case 4:
//		{
//			SetFaceColor(0);
//			glVertex3f(-offset, -scale, -offset);
//			glVertex3f(offset, -scale, -offset);
//			glVertex3f(offset, -scale, offset);
//			glVertex3f(-offset, -scale, offset);
//
//			SetFaceColor(-1);
//			glVertex3f(-offset2, -scale+epsilon, -offset2);
//			glVertex3f(offset2, -scale+epsilon, -offset2);
//			glVertex3f(offset2, -scale+epsilon, offset2);
//			glVertex3f(-offset2, -scale+epsilon, offset2);
//
//		} break;
//		case 22:
//		{
//			SetFaceColor(5);
//			glVertex3f(-offset, scale, -offset);
//			glVertex3f(offset, scale, -offset);
//			glVertex3f(offset, scale, offset);
//			glVertex3f(-offset, scale, offset);
//
//			SetFaceColor(-1);
//			glVertex3f(-offset2, scale-epsilon, -offset2);
//			glVertex3f(offset2, scale-epsilon, -offset2);
//			glVertex3f(offset2, scale-epsilon, offset2);
//			glVertex3f(-offset2, scale-epsilon, offset2);
//		} break;
//		case 12:
//		{
//			SetFaceColor(1);
//			glVertex3f(-scale, -offset, -offset);
//			glVertex3f(-scale, offset, -offset);
//			glVertex3f(-scale, offset, offset);
//			glVertex3f(-scale, -offset, offset);
//
//			SetFaceColor(-1);
//			glVertex3f(-scale+epsilon, -offset2, -offset2);
//			glVertex3f(-scale+epsilon, offset2, -offset2);
//			glVertex3f(-scale+epsilon, offset2, offset2);
//			glVertex3f(-scale+epsilon, -offset2, offset2);
//
//		} break;
//		case 16:
//		{
//			SetFaceColor(4);
//			glVertex3f(-offset, -offset, scale);
//			glVertex3f(offset, -offset, scale);
//			glVertex3f(offset, offset, scale);
//			glVertex3f(-offset, offset, scale);
//
//			SetFaceColor(-1);
//			glVertex3f(-offset2, -offset2, scale-epsilon);
//			glVertex3f(offset2, -offset2, scale-epsilon);
//			glVertex3f(offset2, offset2, scale-epsilon);
//			glVertex3f(-offset2, offset2, scale-epsilon);
//
//		} break;
//		case 10:
//		{
//			SetFaceColor(2);
//			glVertex3f(-offset, -offset, -scale);
//			glVertex3f(offset, -offset, -scale);
//			glVertex3f(offset, offset, -scale);
//			glVertex3f(-offset, offset, -scale);
//
//			SetFaceColor(-1);
//			glVertex3f(-offset2, -offset2, -scale+epsilon);
//			glVertex3f(offset2, -offset2, -scale+epsilon);
//			glVertex3f(offset2, offset2, -scale+epsilon);
//			glVertex3f(-offset2, offset2, -scale+epsilon);
//		} break;
//		case 14:
//		{
//			SetFaceColor(3);
//			glVertex3f(scale, -offset, -offset);
//			glVertex3f(scale, offset, -offset);
//			glVertex3f(scale, offset, offset);
//			glVertex3f(scale, -offset, offset);
//
//			SetFaceColor(-1);
//			glVertex3f(scale-epsilon, -offset2, -offset2);
//			glVertex3f(scale-epsilon, offset2, -offset2);
//			glVertex3f(scale-epsilon, offset2, offset2);
//			glVertex3f(scale-epsilon, -offset2, offset2);
//		} break;
//	}
//}
}

void RC::SetFaceColor(int theColor) const
{
//	switch (theColor)
//	{
//		case -1: glColor3f(0.0, 0.0, 0.0); break;
//		case 0: glColor3f(1.0, 0.0, 0.0); break;
//		case 1: glColor3f(0.0, 1.0, 0.0); break;
//		case 2: glColor3f(0.0, 0.0, 1.0); break;
//		case 3: glColor3f(1.0, 1.0, 0.0); break;
//		case 4: glColor3f(1.0, 0.75, 0.0); break;
//		case 5: glColor3f(1.0, 1.0, 1.0); break;
//		default: assert(false);
//	}
}


/** Draw the transition at some percentage 0...1 between two states */
void RC::OpenGLDraw(const RCState &s1, const RCState &s2, float t) const
{
////	glEnable( GL_POLYGON_SMOOTH );
////	glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );
//	int vals[3] = {-90, 90, 180};
//	RCAction a = GetAction(s1, s2);
//	switch (a/3)
//	{
//		case 0: // 0
//		{
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 0, 1, 0); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, x);
//				c.OpenGLDrawCube(s1.corner, x);
//				OpenGLDrawCube(x);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 9; x < 27; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, x);
//				c.OpenGLDrawCube(s1.corner, x);
//				OpenGLDrawCube(x);
//			}
//			glEnd();
//		} break;
//		case 1: // 5
//		{
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 0, 1, 0); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 18; x < 27; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, x);
//				c.OpenGLDrawCube(s1.corner, x);
//				OpenGLDrawCube(x);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, x);
//				c.OpenGLDrawCube(s1.corner, x);
//				OpenGLDrawCube(x);
//			}
//			glEnd();
//		} break;
//		case 2: // 2
//		{
//			int which[9] = {0, 1, 2, 9, 10, 11, 18, 19, 20};
//			int others[18] = {3, 4, 5, 6, 7, 8, 12, 13, 14, 15, 16, 17, 21, 22, 23, 24, 25, 26};
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 0, 0, -1); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, which[x]);
//				c.OpenGLDrawCube(s1.corner, which[x]);
//				OpenGLDrawCube(which[x]);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, others[x]);
//				c.OpenGLDrawCube(s1.corner, others[x]);
//				OpenGLDrawCube(others[x]);
//			}
//			glEnd();
//		} break;
//		case 3: // 4
//		{
//			int which[9] = {6, 7, 8, 15, 16, 17, 24, 25, 26};
//			int others[18] = {3, 4, 5, 0, 1, 2, 12, 13, 14, 9, 10, 11, 21, 22, 23, 18, 19, 20};
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 0, 0, 1); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, which[x]);
//				c.OpenGLDrawCube(s1.corner, which[x]);
//				OpenGLDrawCube(which[x]);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, others[x]);
//				c.OpenGLDrawCube(s1.corner, others[x]);
//				OpenGLDrawCube(others[x]);
//			}
//			glEnd();
//		} break;
//		case 4: // 1
//		{
//			int which[9] = {0, 3, 6, 9, 12, 15, 18, 21, 24};
//			int others[18] = {1, 2, 4, 5, 7, 8, 10, 11, 13, 14, 16, 17, 19, 20, 22, 23, 25, 26};
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, -1, 0, 0); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, which[x]);
//				c.OpenGLDrawCube(s1.corner, which[x]);
//				OpenGLDrawCube(which[x]);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, others[x]);
//				c.OpenGLDrawCube(s1.corner, others[x]);
//				OpenGLDrawCube(others[x]);
//			}
//			glEnd();
//			
//		} break;
//		case 5: // 3
//		{
//			int which[9] = {2, 5, 8, 11, 14, 17, 20, 23, 26};
//			int others[18] = {0, 1, 3, 4, 6, 7, 9, 10, 12, 13, 15, 16, 18, 19, 21, 22, 24, 25};
//			glPushMatrix();
//			glRotatef(vals[a%3]*t, 1, 0, 0); // parameterize
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 9; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, which[x]);
//				c.OpenGLDrawCube(s1.corner, which[x]);
//				OpenGLDrawCube(which[x]);
//			}
//			glEnd();
//			glPopMatrix();
//			
//			glBegin(GL_QUADS);
//			for (int x = 0; x < 18; x++)
//			{
//				e.OpenGLDrawCube(s1.edge, others[x]);
//				c.OpenGLDrawCube(s1.corner, others[x]);
//				OpenGLDrawCube(others[x]);
//			}
//			glEnd();
//		} break;
//	}
}

void RC::OpenGLDraw(const RCState&, const RCAction&) const
{
	
}

// Draw a RCState. Internally do the 3d transform for the drawing.
void RC::Draw(Graphics::Display &display, const RCState&) const
{
}



/*
 * 
 */
void EditPointMult(float (&vec)[3][3], Graphics::point & p)
{
	int vecCols = 3;
	int vecRows = 3;

	array3[0] = p.x;
	array3[1] = p.y;
	array3[2] = p.z;

	for (int i = 0; i < vecRows; i++)
	{
		float sum = 0.f;

		for (int j = 0; j < vecCols; j++)
		{
			sum += vec[i][j] * array3[j];
		}

		switch (i)
		{
			case 0: p.x = sum;
				break;
			case 1: p.y = sum;
				break;
			case 2: p.z = sum;
				break;
		}
	}
}

/* Edit an existing 3x3 vector to become the
 * X rotation multiplication matrix for a given angle
 */
//void MakeMatrixRotX(std::vector<std::vector<float>> & a, float angle) {
void MakeMatrixRotX(float (&a)[3][3], float angle) {
	a[0][0] = 1;
	a[0][1] = 0;
	a[0][2] = 0;

	a[1][0] = 0;
	a[1][1] = cos(angle);
	a[1][2] = -sin(angle);

	a[2][0] = 0;
	a[2][1] = sin(angle);
	a[2][2] = cos(angle);
}

/* Edit an existing 3x3 vector to become the
 * Y rotation multiplication matrix for a given angle
 */
//void MakeMatrixRotY(std::vector<std::vector<float>> & a, float angle) {
void MakeMatrixRotY(float (&a)[3][3], float angle) {	
	a[0][0] = cosf(angle);
	a[0][1] = 0;
	a[0][2] = -sin(angle);

	a[1][0] = 0;
	a[1][1] = 1;
	a[1][2] = 0;

	a[2][0] = sin(angle);
	a[2][1] = 0;
	a[2][2] = cos(angle);
}

/* Edit an existing 3x3 vector to become the
 * Z rotation multiplication matrix for a given angle
 */
//void MakeMatrixRotZ(std::vector<std::vector<float>> & a, float angle) {
void MakeMatrixRotZ(float (&a)[3][3], float angle) {
	a[0][0] = cosf(angle);
	a[0][1] = -sin(angle);
	a[0][2] = 0;

	a[1][0] = sin(angle);
	a[1][1] = cos(angle);
	a[1][2] = 0;

	a[2][0] = 0;
	a[2][1] = 0;
	a[2][2] = 1;
}
