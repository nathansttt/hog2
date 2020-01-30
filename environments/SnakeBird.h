//
//  SnakeBird.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/26/20.
//  Copyright © 2020 University of Denver. All rights reserved.
//

#ifndef SnakeBird_h
#define SnakeBird_h

#include <stdio.h>
#include "SearchEnvironment.h"
#include <array>

namespace SnakeBird {

// 64 bits can hold
//
//
// world is 20x16 (WxH) = 320
// 1 snake = [5 bits, 4 bits] 9 bits head, 2 bits (4 dir) per segment
// 3 snakes = 27 bits + length

// uint64_t blockFruit
// block: 9 bits each, max 4(?) = 36 bits
// fruit: up to 28 - one bit each
const uint64_t locationMask = 0x1FF;
const uint64_t snakeLenMask = 0x1F;
const uint64_t fruitMask = 0x1F;
const uint64_t snakeBodyMask = 0x3;
const int kInGoal = 320;

enum snakeDir : uint8_t {
	kLeft=0x0, kRight=0x1, kUp=0x2, kDown=0x3
};
struct SnakeBirdState {
	uint64_t snakeBodies; // up to 32 in length
	uint64_t snakeHeads; // up to 4. 2bits (num snakes) + 9 bits head + 5 bits start location of next. (max coordinate) head means snake left level
	uint64_t locBlockFruit; // 4*9bit blocks; 28 fruit

	SnakeBirdState() :snakeBodies(0), snakeHeads(0), locBlockFruit(0) {}
	bool operator==(const SnakeBirdState &s) const {
		return (s.snakeBodies==snakeBodies)&&(s.snakeHeads==snakeHeads)&&(s.locBlockFruit==locBlockFruit);
	}
	int GetNumSnakes() const { return snakeHeads&0x3; }
	void SetNumSnakes(int count) { snakeHeads = (snakeHeads&(~snakeBodyMask))|(count&0x3); }
	int GetSnakeHeadLoc(int whichSnake) const
	{ return (snakeHeads>>(2+whichSnake*14))&0x1FF;}
	void SetSnakeHeadLoc(int whichSnake, int loc)
	{ snakeHeads &= ~(locationMask<<(2+whichSnake*14)); snakeHeads |= ((loc&locationMask)<<(2+whichSnake*14)); }
	int GetSnakeBodyEnd(int whichSnake) const
	{ return (whichSnake<0)?0:(snakeHeads>>(2+9*(whichSnake+1)+5*whichSnake))&snakeLenMask; }
	void SetSnakeBodyEnd(int whichSnake, int endOffset)
	{ snakeHeads &= ~(snakeLenMask<<(2+9*(whichSnake+1)+5*whichSnake)); snakeHeads |= ((endOffset&snakeLenMask)<<(2+9*(whichSnake+1)+5*whichSnake)); }
	
	snakeDir GetSnakeDir(int whichSnake, int segment) const
	{ return static_cast<snakeDir>((snakeBodies>>(2*segment+2*GetSnakeBodyEnd(whichSnake-1)))&snakeBodyMask); }
	void SetSnakeDir(int whichSnake, int segment, snakeDir dir)
	{
		snakeBodies &= ~(snakeBodyMask<<(2*segment+2*GetSnakeBodyEnd(whichSnake-1)));
		snakeBodies |= ((static_cast<uint64_t>(dir))<<(2*segment+2*GetSnakeBodyEnd(whichSnake-1)));
	}
	// add dir to beginning of snake
	void InsertSnakeDir(int whichSnake, snakeDir dir)
	{
		uint64_t mask = 1;
		mask <<= (GetSnakeBodyEnd(whichSnake)-GetSnakeBodyEnd(whichSnake-1))*2; // length in bits
		mask -= 1;
		mask <<= GetSnakeBodyEnd(whichSnake-1)*2;
		uint64_t oldSnake = snakeBodies&(mask);
		oldSnake<<=2;
		oldSnake |= dir<<(GetSnakeBodyEnd(whichSnake-1)*2);
		snakeBodies &= (~mask);
		snakeBodies |= (oldSnake&(mask));
	}
	
	int GetObjectLocation(int whichObstacle) const { return (locBlockFruit>>(9*whichObstacle))&locationMask; }
	void SetObjectLocation(int whichObstacle, int loc) //{ return (locBlockFruit>>(9*whichObstacle))&locationMask; }
	{ locBlockFruit &= ~(locationMask<<(whichObstacle*9)); locBlockFruit |= ((loc&locationMask)<<(whichObstacle*9)); }
	
	bool GetFruitPresent(int which) const { return (locBlockFruit>>(36+which))&0x1; }
	void ToggleFruitPresent(int which) { locBlockFruit ^= (fruitMask<<(36+which));}
};

struct SnakeBirdAction {
	unsigned int bird : 4; // which bird
	unsigned int direction : 4; // which direction
	bool operator==(const SnakeBirdAction &a) const
	{
		return a.bird == bird && a.direction == direction;
	}
};


/*
 Level files are text
 * Terrain types in text:
 * . Empty
 * G Ground
 * X Spikes
 * O Portal
 * E Exit
 * F Fruit
 * 1 [1-4] Parts of object
 * < Snake heads. Body is on the open side, eg <===
 * >
 * ^
 * V
 * a-z [snake body in order, all letters must be unique in level]
 */

enum SnakeBirdWorldObject : uint8_t {
	kEmpty = 0,
	kGround,
	kSpikes,
	kPortal,
	kExit,
	kFruit
};

class SnakeBird : public SearchEnvironment<SnakeBirdState, SnakeBirdAction> {
public:
	//SnakeBird()
	bool Load(const char *filename);
	bool Save(const char *filename);
	SnakeBirdState GetStart();
	void SetGroundType(int x, int y, SnakeBirdWorldObject o);
	SnakeBirdWorldObject GetGroundType(int x, int y);

	void GetSuccessors(const SnakeBirdState &nodeID, std::vector<SnakeBirdState> &neighbors) const;
	void GetActions(const SnakeBirdState &nodeID, std::vector<SnakeBirdAction> &actions) const;
	
	//SnakeBirdAction GetAction(const SnakeBirdState &s1, const SnakeBirdState &s2) const;
	void ApplyAction(SnakeBirdState &s, SnakeBirdAction a) const;
	bool Legal(SnakeBirdState &s, SnakeBirdAction a);
	// Cannot undo actions
	void UndoAction(SnakeBirdState &s, SnakeBirdAction a) const
	{ assert(false); }
	
	void GetNextState(const SnakeBirdState &s1, SnakeBirdAction a, SnakeBirdState &s2) const
	{
		s2 = s1;
		ApplyAction(s2, a);
	};
	
	bool InvertAction(SnakeBirdAction &a) const { return false; }
	
	/** Stores the goal for use by single-state HCost. **/
	void StoreGoal(SnakeBirdState &s)
	{ bValidSearchGoal = true; searchGoal = s; }
	
	/** Clears the goal from memory. **/
	void ClearGoal()
	{ bValidSearchGoal = false; }
	
	/** Returns true if the goal is stored and false otherwise. **/
	bool IsGoalStored() const
	{ return bValidSearchGoal; }
	
	
	/** Heuristic value between two arbitrary nodes. **/
	double HCost(const SnakeBirdState &node1, const SnakeBirdState &node2) const { return 0; }
	
	double GCost(const SnakeBirdState &node1, const SnakeBirdState &node2) const { return 1;}
	double GCost(const SnakeBirdState &node, const SnakeBirdAction &act) const { return 1;}
	bool GoalTest(const SnakeBirdState &node, const SnakeBirdState &goal) const { return GoalTest(node); }
	
	/** Goal Test if the goal is stored **/
	virtual bool GoalTest(const SnakeBirdState &node) const
	{ for (int x = 0; x < node.GetNumSnakes(); x++) if (node.GetSnakeHeadLoc(x) != kInGoal) return false; return true; }
		
	uint64_t GetActionHash(SnakeBirdAction act) const {return (act.bird<<2)|act.direction;}
	uint64_t GetStateHash(const SnakeBirdState &node) const
	{
		return node.snakeHeads^node.snakeBodies^node.locBlockFruit;
	}

	void OpenGLDraw() const {}
	void OpenGLDraw(const SnakeBirdState&) const {}
	/** Draw the transition at some percentage 0...1 between two states */
	void OpenGLDraw(const SnakeBirdState&, const SnakeBirdState&, float) const {}
	void OpenGLDraw(const SnakeBirdState&, const SnakeBirdAction&) const {};
	void GLLabelState(const SnakeBirdState&, const char *) const {} // draw label over state
	void GLDrawLine(const SnakeBirdState &x, const SnakeBirdState &y) const {}
	void GLDrawPath(const std::vector<SnakeBirdState> &x) const {}
	
	void Draw(Graphics::Display &display);
	void Draw(Graphics::Display &display, const SnakeBirdState&) const;
	void Draw(Graphics::Display &display, const SnakeBirdState&, int active) const;
	void DrawLine(Graphics::Display &display, const SnakeBirdState &x, const SnakeBirdState &y, float width = 1.0) const;
private:
	int GetIndex(int x, int y) const;
	int GetX(int index) const;
	int GetY(int index) const;
	Graphics::point GetCenter(int x, int y) const;
	float GetRadius() const;
	void DrawSnakeSegment(Graphics::Display &display, int x, int y, const rgbColor &color, bool head, bool tail, bool awake, snakeDir dirFrom, snakeDir dirTo) const;

	std::array<SnakeBirdWorldObject, 320> world; // static world
};

}

#endif /* SnakeBird_h */
