//
//  VoxelGrid.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/7/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#include <stdio.h>
#include "VoxelGrid.h"

bool operator==(const voxelGridState &v1, const voxelGridState &v2)
{
	return v1.x == v2.x && v1.y == v2.y && v1.z == v2.z;
}


VoxelGrid::VoxelGrid(const char *filename)
{
	FILE *f = fopen(filename, "r");
	if (f == 0)
	{
		printf("Error opening file\n");
		exit(0);
	}
	int cnt = fscanf(f, "voxel %d %d %d\n", &xWidth, &yWidth, &zWidth);
	if (cnt != 3)
	{
		printf("Error reading dimensions\n");
		return;
	}
	voxels.resize(xWidth*yWidth*zWidth);
	int a, b, c;
	while ((cnt = fscanf(f, "%d %d %d\n", &a, &b, &c)) == 3)
	{
		voxels[GetIndex(a, b, c)] = true;
	}
	
	fclose(f);
}

VoxelGrid::~VoxelGrid()
{
	
}

int VoxelGrid::GetIndex(int x, int y, int z) const
{
	return xWidth*(y*zWidth+z)+x;
}

void VoxelGrid::GetCoordinates(int index, int &x, int &y, int &z) const
{
	x = index%xWidth;
	index /= xWidth;
	z = index%zWidth;
	y = index/zWidth;
}


void VoxelGrid::GetSuccessors(const voxelGridState &nodeID, std::vector<voxelGridState> &neighbors) const
{
	
}

void VoxelGrid::GetActions(const voxelGridState &nodeID, std::vector<voxelGridAction> &actions) const
{
	
}

void VoxelGrid::ApplyAction(voxelGridState &s, voxelGridAction a) const
{
	
}

bool VoxelGrid::InvertAction(voxelGridAction &a) const
{
	return false;
}


/** Heuristic value between two arbitrary nodes. **/
double VoxelGrid::HCost(const voxelGridState &node1, const voxelGridState &node2) const
{
	return 0;
}

double VoxelGrid::GCost(const voxelGridState &node1, const voxelGridState &node2) const
{
	return 1;
}

double VoxelGrid::GCost(const voxelGridState &node, const voxelGridAction &act) const
{
	return 1;
}

bool VoxelGrid::GoalTest(const voxelGridState &node, const voxelGridState &goal) const
{
	return node == goal;
}


uint64_t VoxelGrid::GetStateHash(const voxelGridState &node) const
{
	return (node.x<<32)|(node.y<<16)|(node.z);
}

void VoxelGrid::GetStateFromHash(uint64_t parent, voxelGridState &s)
{
	s.z = parent&0xFFFF;
	s.y = (parent>>16)&0xFFFF;
	s.x = (parent>>32)&0xFFFF;
}


uint64_t VoxelGrid::GetActionHash(voxelGridAction act) const
{
	return (int)act;
}

void VoxelGrid::OpenGLDraw() const
{
	glEnable(GL_LIGHTING);
	glColor3f(0.0, 1.0, 0.0);
	double range = std::max(xWidth, std::max(yWidth, zWidth));
	for (int x = 0; x < xWidth; x++)
	{
		for (int y = 0; y < yWidth; y++)
		{
			for (int z = 0; z < zWidth; z++)
			{
				if (voxels[GetIndex(x, y, z)])
				{
					DrawBox(2.0*x/range-1.0+(-xWidth+range)/range,
							2.0*y/range-1.0+(-yWidth+range)/range,
							2.0*z/range-1.0+(-zWidth+range)/range, 1/range);
				}
			}
		}
	}
	glDisable(GL_LIGHTING);
}

void VoxelGrid::OpenGLDraw(const voxelGridState&) const
{
	
}

void VoxelGrid::OpenGLDraw(const voxelGridState&, const voxelGridState&, float) const
{
	
}

void VoxelGrid::OpenGLDraw(const voxelGridState&, const voxelGridAction&) const
{
	
}

void VoxelGrid::GLLabelState(const voxelGridState&, const char *) const
{
	
}

void VoxelGrid::GLDrawLine(const voxelGridState &x, const voxelGridState &y) const
{
	
}
