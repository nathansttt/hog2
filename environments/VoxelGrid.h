//
//  VoxelGrid.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/7/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef VoxelGrid_h
#define VoxelGrid_h

#include "SearchEnvironment.h"
#include <vector>


struct voxelGridState {
	uint16_t x, y, z;
};

bool operator==(const voxelGridState &v1, const voxelGridState &v2);

enum voxelGridAction {
//	kXpYpZp,
//	kXpYpZm,
//	kXpYpZs,
//	kXpYmZp,
//	kXpYmZm,
//	kXpYmZs,
//	kXpYsZp,
//	kXpYsZm,
//	kXpYsZs,
//	kXmYpZp,
//	kXmYpZm,
//	kXmYpZs,
//	kXmYmZp,
//	kXmYmZm,
//	kXmYmZs,
//	kXmYsZp,
//	kXmYsZm,
//	kXmYsZs,
//	kXsYpZp,
//	kXsYpZm,
//	kXsYpZs,
//	kXsYmZp,
//	kXsYmZm,
//	kXsYmZs,
//	kXsYsZp,
//	kXsYsZm,
//	kXsYsZs
};

class VoxelGrid : public SearchEnvironment<voxelGridState, voxelGridAction> {
public:
	VoxelGrid(const char *filename);
	~VoxelGrid();
	void GetSuccessors(const voxelGridState &nodeID, std::vector<voxelGridState> &neighbors) const;
	void GetActions(const voxelGridState &nodeID, std::vector<voxelGridAction> &actions) const;
	void ApplyAction(voxelGridState &s, voxelGridAction a) const;
	bool InvertAction(voxelGridAction &a) const;

	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const voxelGridState &node1, const voxelGridState &node2) const;
	double GCost(const voxelGridState &node1, const voxelGridState &node2) const;
	double GCost(const voxelGridState &node, const voxelGridAction &act) const;
	bool GoalTest(const voxelGridState &node, const voxelGridState &goal) const;

	uint64_t GetStateHash(const voxelGridState &node) const;
	void GetStateFromHash(uint64_t parent, voxelGridState &s);

	uint64_t GetActionHash(voxelGridAction act) const;

	void OpenGLDraw() const;
	void OpenGLDraw(const voxelGridState&) const;
	void OpenGLDraw(const voxelGridState&, const voxelGridState&, float) const;
	void OpenGLDraw(const voxelGridState&, const voxelGridAction&) const;
	void GLLabelState(const voxelGridState&, const char *) const;
	void GLDrawLine(const voxelGridState &x, const voxelGridState &y) const;
private:
	int GetIndex(int x, int y, int z) const;
	void GetCoordinates(int index, int &x, int &y, int &z) const;
	std::vector<bool> voxels;
	int xWidth, yWidth, zWidth;
};


#endif /* VoxelGrid_h */
