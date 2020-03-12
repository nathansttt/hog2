//
//  SnakeBird.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/26/20.
//  Copyright © 2020 University of Denver. All rights reserved.
//

#include "SnakeBird.h"
#include <fstream>
using namespace std;

namespace SnakeBird {

SnakeBird::SnakeBird(int width, int height)
:width(width), height(height)
{
	assert(width*height < 512);
	portal1Loc = portal2Loc = -1;
	std::fill_n(&world[0], 512, kEmpty);
	for (int x = 0; x < width; x++)
		world[GetIndex(x, height-1)] = kSpikes;
}

void SnakeBird::Reset()
{
	width = 20;
	height = 16;
	portal1Loc = portal2Loc = -1;
	std::fill_n(&world[0], 512, kEmpty);
	for (int x = 0; x < width; x++)
		world[GetIndex(x, height-1)] = kSpikes;
	startState.Reset();
	fruit.clear();
	for (int x = 0; x < objects.size(); x++)
		objects[x].clear();
}

SnakeBirdState SnakeBird::GetStart()
{
	return startState;
}

void SnakeBird::AddSnake(int x, int y, const std::vector<snakeDir> &body)
{
	int count = startState.GetNumSnakes();
	startState.SetNumSnakes(count+1);
	startState.SetSnakeHeadLoc(count, GetIndex(x, y));
	startState.SetSnakeLength(count, body.size()+1);
	for (int x = 0; x < body.size(); x++)
		startState.SetSnakeDir(count, x, body[x]);
}

std::vector<snakeDir>  LoadSnake(std:: vector<snakeDir> snakeBod, int width, int pos,std::vector<char> lvl) 
{
	bool snakeBuilt = false;
	while(!snakeBuilt)
	{
			//build_snake:
		if (lvl[pos+1] == 1+lvl[pos]){
			snakeBod.push_back(kRight);
			pos++;
			//goto build_snake;
		}
		else if (lvl[pos-1] == 1+lvl[pos])
		{
			snakeBod.push_back(kLeft);
			pos--;
			//goto build_snake;
		}
		else if (lvl[pos+width+1] == 1+lvl[pos])
		{
			snakeBod.push_back(kDown);
			pos+= width +1;
			//goto build_snake;
		}
		else if (lvl[pos-width-1] == 1+lvl[pos])
		{
			snakeBod.push_back(kUp);
			pos-= width -1;
			//goto build_snake;
		}
		else
		{
			snakeBuilt = true;
			return snakeBod;
		}
	}
}





bool SnakeBird::Load(const char *filename)
{
	ifstream infile;
	infile.open(filename);
	if (infile.fail()) {
		printf("File could not be opened.");
		return false;
	}
	if(infile.is_open()){
		char ch, ach;
		int x = 0;
		int y = 0;
		int ct = 0;
	
		std::string hLine;
		getline(infile, hLine);
	 	
		std::string sHeight = hLine.substr(8,2);
		std::string sWidth = hLine.substr(5, 2);

		int wWidth = std::stoi(sWidth, nullptr, 10);
		int wHeight = std::stoi(sHeight, nullptr,10);

		portal1Loc = portal2Loc = -1;
		std::fill_n(&world[0], 512, kEmpty);
		startState.Reset();
		fruit.clear();
		for (int x = 0; x < objects.size(); x++)
			objects[x].clear();

		//Note that you can't call the constructor again at this point
		//SnakeBird(wWidth, wHeight);
		this->width = wWidth;
		this->height = wHeight;
		for (int x = 0; x < width; x++)
			world[GetIndex(x, height-1)] = kSpikes;

		std::vector<char> lvlary;
		while (infile.get(ch)) {
			
			lvlary.push_back(ch);
		}
		
		std:: vector<snakeDir> bod;
		bool snakeBuilt = false;
		bool madePortal = false;
		for(int a = 0; a <= lvlary.size(); a++)
		{
			int b = 0;
			switch (lvlary[a])
			{
				case '.': SetGroundType(x,y,SnakeBirdWorldObject::kEmpty); x++; break;
				case 'G': SetGroundType(x,y,SnakeBirdWorldObject::kGround); x++; break;
				case 'S': SetGroundType(x,y,SnakeBirdWorldObject::kSpikes); x++; break;
				case 'O':
					if (!madePortal)
					{
						SetGroundType(x,y,SnakeBirdWorldObject::kPortal1);
						madePortal = true;
					}
					else {
						SetGroundType(x,y,SnakeBirdWorldObject::kPortal2);
					}
						x++;
						break;
				case 'E': SetGroundType(x,y,SnakeBirdWorldObject::kExit); x++; break;
				case 'F': SetGroundType(x,y,SnakeBirdWorldObject::kFruit); x++; break;
				case '1': SetGroundType(x, y, kBlock1); x++; break;
				case '2': SetGroundType(x, y, kBlock2); x++; break;
				case '3': SetGroundType(x, y, kBlock3); x++; break;
				case '4': SetGroundType(x, y, kBlock4); x++; break;
				case '\n': x = 0; y++; break;
				case '<': 
					bod.push_back(kRight);
					b = a;
					b++;
					bod = LoadSnake(bod, wWidth, b, lvlary);
					AddSnake(x, y, bod);
					bod.clear();
					SetGroundType(x,y,SnakeBirdWorldObject::kEmpty);
					x++;
					break;
				case '>':
					bod.push_back(kLeft);
					b = a;
					b--;
					bod = LoadSnake(bod, wWidth, b, lvlary);
					AddSnake(x, y, bod);
					bod.clear();
					x++;
					break;
				case '^':
					bod.push_back(kDown);
					b = a;
					b += wWidth+1;
					bod = LoadSnake(bod, wWidth, b, lvlary);
					AddSnake(x, y, bod);
					bod.clear();
					x++;
					break;
				case 'v':
					bod.push_back(kUp);
					b = a;
					b -= wWidth-1;
					bod = LoadSnake(bod, wWidth, b, lvlary);
					AddSnake(x, y, bod);
					bod.clear();
					x++;
					break;
				default: SetGroundType(x,y,SnakeBirdWorldObject::kEmpty); x++; break;
			}		
		}
		infile.close();
		return  true;
	}
	else{
		cout << "Error loading level.";
		return false;
	}	
}

bool SnakeBird::Save(const char *filename)
{
	return false;
}


void SnakeBird::GetSuccessors(const SnakeBirdState &nodeID, std::vector<SnakeBirdState> &neighbors) const
{
	static std::vector<SnakeBirdAction> acts;
	GetActions(nodeID, acts);
	neighbors.clear();
	for (auto &a : acts)
	{
		SnakeBirdState s = nodeID;
		ApplyAction(s, a);
		bool valid = true;
		for (int x = s.GetNumSnakes()-1; x >= 0; x--)
			if (s.GetSnakeHeadLoc(x) == kDead)
			{
				valid = false;
				break;
			}
		if (valid)
			neighbors.push_back(s);
	}
}

bool SnakeBird::Legal(SnakeBirdState &s, SnakeBirdAction a)
{
	static std::vector<SnakeBirdAction> acts;
	GetActions(s, acts);
	for (auto &act : acts)
		if (act == a)
			return true;
	return false;
}

bool SnakeBird::Render(const SnakeBirdState &s) const
{
	SnakeBirdWorldObject obj[4] = {kSnake1, kSnake2, kSnake3, kSnake4};
	std::fill_n(&render[0], 512, kEmpty);
	
	// render snakes into world
	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		int loc = s.GetSnakeHeadLoc(snake);
		if (loc == kInGoal)
			continue;
		if (loc == kDead) // no legal actions for any snake if one is dead
			return false;
		render[loc] = obj[snake];
		if (world[loc] == kSpikes)
			return false;
		int len = s.GetSnakeBodyEnd(snake)-s.GetSnakeBodyEnd(snake-1);
		for (int x = 0; x < len; x++)
		{
			switch (s.GetSnakeDir(snake, x))
			{
				case kLeft: loc-=height; break;
				case kRight: loc+=height; break;
				case kUp: loc-=1; break;
				case kDown: loc+=1; break;
			}
			assert(loc >= 0 && loc < width*height);
			render[loc] = obj[snake];
			if (world[loc] == kSpikes)
				return false;
		}
	}
	// render fruit into world
	for (int f = 0; f < fruit.size(); f++)
	{
		if (s.GetFruitPresent(f))
			render[fruit[f]] = kFruit;
	}

	// render objects into world
	for (int x = 0; x < 4; x++)
	{
		SnakeBirdWorldObject item[4] = {kBlock1, kBlock2, kBlock3, kBlock4};
		if (s.GetObjectLocation(x) == kDead)
			// continue;  //    allow pushing objects off the board
			return false; // disallow pushing objects off the board

		for (int i = 0; i < objects[x].size(); i++)
		{
			render[GetIndex(GetX(objects[x][i])+GetX(s.GetObjectLocation(x)),
							GetY(objects[x][i])+GetY(s.GetObjectLocation(x)))] = item[x];
		}
	}
	return true;
}

void SnakeBird::GetActions(const SnakeBirdState &s, std::vector<SnakeBirdAction> &actions) const
{
	actions.clear();
	SnakeBirdAction a;
	
	bool success = Render(s);
	if (!success) // something illegal about state - usually has dead snakes
		return;
	SnakeBirdWorldObject obj[4] = {kSnake1, kSnake2, kSnake3, kSnake4};

	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		// first pass - don't allow to move back onto yourself or onto other obstacles
		int loc = s.GetSnakeHeadLoc(snake);
		if (loc == kInGoal)
			continue;

		a.bird = snake;
		a.pushed = 0;
		// kDown - can never push any object down due to gravity
		if (!(kGroundMask == (world[loc+1]&kGroundMask)) && // not ground or spikes
			GetY(loc)+1 < height && // not off screen
			!(kSnakeMask == (render[loc+1]&kSnakeMask)) && // not snake
			!(kBlockMask == (render[loc+1]&kBlockMask))) // not block
		{
			if (s.GetSnakeDir(snake, 0) != kDown) // not blocked by own snake
			{
				a.direction = kDown;
				actions.push_back(a);
			}
		}

		a.pushed = 0;
		// kUp
		if (!(kGroundMask == (world[loc-1]&kGroundMask)) && // not ground or spikes
			GetY(loc) > 0 && // not off screen
			render[loc-1] != obj[snake]) // not self
		{
			// check if snake or object which can be pushed
			if ((kSnakeMask == (render[loc-1]&kSnakeMask)) || (kBlockMask == (render[loc-1]&kBlockMask)))
			{
				if (CanPush(s, snake, render[loc-1], kUp, a))
				{
					a.direction = kUp;
					actions.push_back(a);
				}
			}
			else {
				assert(kCanEnterMask == (world[loc-1]&kCanEnterMask));
				a.direction = kUp;
				actions.push_back(a);
			}
		}

		a.pushed = 0;
		// right
		if (!(kGroundMask == (world[loc+height]&kGroundMask)) && // not ground or spikes
			GetX(loc)+1 < width && // not off screen
			render[loc+height] != obj[snake]) // not self
		{
			// check if snake or object which can be pushed
			if ((kSnakeMask == (render[loc+height]&kSnakeMask)) || (kBlockMask == (render[loc+height]&kBlockMask)))
			{
				if (CanPush(s, snake, render[loc+height], kRight, a))
				{
					a.direction = kRight;
					actions.push_back(a);
				}
			}
			else {
				assert(kCanEnterMask == (world[loc+height]&kCanEnterMask));
				a.direction = kRight;
				actions.push_back(a);
			}
		}
		
		a.pushed = 0;
		// left
		if (!(kGroundMask == (world[loc-height]&kGroundMask)) && // not ground or spikes
			GetX(loc)-1 >= 0 && // not off screen
			render[loc-height] != obj[snake]) // not self
		{
			// check if snake or object which can be pushed
			if ((kSnakeMask == (render[loc-height]&kSnakeMask)) || (kBlockMask == (render[loc-height]&kBlockMask)))
			{
				if (CanPush(s, snake, render[loc-height], kLeft, a))
				{
					a.direction = kLeft;
					actions.push_back(a);
				}
			}
			else {
				assert(kCanEnterMask == (world[loc-height]&kCanEnterMask));
				a.direction = kLeft;
				actions.push_back(a);
			}
		}
	}
}

bool SnakeBird::CanPush(const SnakeBirdState &s, int snake, SnakeBirdWorldObject obj, snakeDir dir,
						SnakeBirdAction &a) const
{
//	if (pushObject >= kMaxPushedObjects)
//		return false;
	int pushed = -1;
	// can't push yourself
	//if (pushObject+1 < kMaxPushedObjects)
	//a.pushed[pushObject+1] = kNothingPushed;
	switch (obj) {
		case kSnake1: if (snake == 0) return false; pushed = 0; break;
		case kSnake2: if (snake == 1) return false; pushed = 1; break;
		case kSnake3: if (snake == 2) return false; pushed = 2; break;
		case kSnake4: if (snake == 3) return false; pushed = 3; break;
		case kBlock1: pushed = 4; break;
		case kBlock2: pushed = 5; break;
		case kBlock3: pushed = 6; break;
		case kBlock4: pushed = 7; break;
		default: return false; break;
	}
	// already pushing this
	if ((a.pushed>>pushed)&0x1)
		return true;
	
	// mark object as being pushed
	a.pushed |= (1<<pushed);
	
	if (pushed <= 3) // snake is pushed
	{
		// Render pushed obj one step in (dir)
		int loc = s.GetSnakeHeadLoc(pushed);
		switch (dir)
		{
			case kLeft: if (loc < height) return false; loc-=height; break;
			case kRight: if (loc+height > width*height) return false; loc+=height; break;
			case kUp: if (loc == 0) return false; loc-=1; break;
			case kDown: if (loc+1 == width*height) return false; loc+=1; break;
		}
		// cannot push into solid object - except when going down
		if ((((world[loc]&kGroundMask) == kGroundMask) && dir != kDown) ||
			(dir == kDown && world[loc] == kGround))
			return false;
		if (render[loc]==kFruit) // cannot push into fruit
			return false;
		if (((render[loc]&kSnakeMask) == kSnakeMask) || (render[loc]&kBlockMask) == kBlockMask) // pushable object
		{
			if (!CanPush(s, snake, render[loc], dir, a))
				return false;
		}
		
		int len = s.GetSnakeBodyEnd(pushed)-s.GetSnakeBodyEnd(pushed-1);
		for (int x = 0; x < len; x++)
		{
			switch (s.GetSnakeDir(pushed, x))
			{
				case kLeft: if (loc < height) return false; loc-=height; break;
				case kRight: if (loc+height > width*height) return false; loc+=height; break;
				case kUp: if (loc == 0) return false; loc-=1; break;
				case kDown: if (loc+1 == width*height) return false; loc+=1; break;
			}
			// TODO: check if snake renders into spikes
			// cannot push into solid object - except when going down
			if ((((world[loc]&kGroundMask) == kGroundMask) && dir != kDown) ||
				(dir == kDown && world[loc] == kGround))
				return false;
			if (render[loc]==kFruit) // cannot push into fruit
				return false;
			if (((render[loc]&kSnakeMask) == kSnakeMask) || (render[loc]&kBlockMask) == kBlockMask) // pushable object
			{
				if (!CanPush(s, snake, render[loc], dir, a))
					return false;
			}
		}
	}
	else if (pushed >= 4) // object is pushed
	{
		pushed-=4;
		// Render pushed obj one step in (dir)
		int loc = s.GetObjectLocation(pushed); // s.GetSnakeHeadLoc(pushed);
		if (loc == kDead)
			return false;
		for (int x = 0; x < objects[pushed].size(); x++)
		{
			int pieceLoc = objects[pushed][x];
			pieceLoc = GetIndex(GetX(objects[pushed][x])+GetX(loc),
								GetY(objects[pushed][x])+GetY(loc));
			switch (dir)
			{
				case kLeft: if (pieceLoc < height) return false; pieceLoc-=height; break;
				case kRight: if (pieceLoc+height > width*height) return false; pieceLoc+=height; break;
				case kUp: if (pieceLoc == 0) return false; pieceLoc-=1; break;
				case kDown: if (pieceLoc+1 == width*height) return false; pieceLoc+=1; break;
			}
			if ((world[pieceLoc]&kGroundMask) == kGroundMask) // cannot push into solid object
				return false;
			if (render[pieceLoc]==kFruit) // cannot push into fruit
				return false;
			if (((render[pieceLoc]&kSnakeMask) == kSnakeMask) || (render[pieceLoc]&kBlockMask) == kBlockMask) // pushable object
			{
				if (!CanPush(s, snake, render[pieceLoc], dir, a))
					return false;
			}
		}
	}
	// temporarily return false to not push other snakes
	//return ;
	return true;
}


void SnakeBird::DoFirstMovement(const SnakeBirdAction &a, int offset,
								snakeDir opposite, SnakeBirdState &s) const {
	if (world[s.GetSnakeHeadLoc(a.bird)+offset] == kExit && s.KFruitEaten(fruit.size()))
	{
		s.SetSnakeHeadLoc(a.bird, kInGoal);
		// previously returned, but still need to push objects and let them fall
		//return;
	}
	// eating fruit (can't push and eat fruit in the same action)
	else if (world[s.GetSnakeHeadLoc(a.bird)+offset] == kFruit &&
			 s.GetFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)+offset)))
	{
		s.ToggleFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)+offset));
		for (int x = a.bird; x < s.GetNumSnakes(); x++)
			s.SetSnakeBodyEnd(x, s.GetSnakeBodyEnd(x)+1);
		s.InsertSnakeHeadDir(a.bird, opposite);
		s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+offset);
	}
	else { // normal movement or pushing
		s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+offset);
		s.InsertSnakeDir(a.bird, opposite);
		
		// check for pushed snakes
		for (int i = 0; i < 4 && (a.pushed>>i); i++)
		{
			if ((a.pushed>>i)&0x1)
			{
				s.SetSnakeHeadLoc(i, s.GetSnakeHeadLoc(i)+offset);
				if (world[s.GetSnakeHeadLoc(i)] == kExit && s.KFruitEaten(fruit.size()))
				{
					s.SetSnakeHeadLoc(i, kInGoal);
				}
			}
		}
		// check for pushed objects
		for (int i = 4; i < 8 && (a.pushed>>i); i++)
		{
			if ((a.pushed>>i)&0x1)
			{
				if (s.GetObjectLocation(i-4) == kDead)
					continue;
				int newloc = s.GetObjectLocation(i-4)+offset;
				s.SetObjectLocation(i-4, newloc);
			}
		}
	}
}

bool SnakeBird::DoFall(SnakeBirdAction &a, SnakeBirdState &s) const
{
	const SnakeBirdWorldObject objs[8] = {kSnake1, kSnake2, kSnake3, kSnake4, kBlock1, kBlock2, kBlock3, kBlock4};
	uint8_t falling = 0;
	SnakeBirdAction move;
	// Find which objects can fall
	for (int i = 0; i < 8; i++)
	{
		move.pushed = 0;
		// snake isn't in the world
		if ((i < 4 && i >= s.GetNumSnakes()) || s.GetSnakeHeadLoc(i) == kInGoal)
			continue;
		// block isn't in the world (fell out or not there to begin with)
		if (i >= 4 && (objects[i-4].size() == 0 || s.GetObjectLocation(i-4) == kDead))
			continue;
		
		if (CanPush(s, -1, objs[i], kDown, move))
		{
			falling |= move.pushed;
		}
	}
	// If nothing can fall, we are done
	if (falling == 0)
		return false;
	
	// Actually move the objects down
	for (int i = 0; i < 8; i++)
	{
		if (i < 4 && ((falling>>i)&0x1))
		{
			s.SetSnakeHeadLoc(i, s.GetSnakeHeadLoc(i)+1);
			if (world[s.GetSnakeHeadLoc(i)] == kExit && s.KFruitEaten(fruit.size()))
			{
				s.SetSnakeHeadLoc(i, kInGoal);
			}
		}
		if (i >= 4 && ((falling>>i)&0x1))
			s.SetObjectLocation(i-4, s.GetObjectLocation(i-4)+1);
	}
	a.pushed = falling;
	
	// check if an object hit the bottom of the world - if so it is dead
	for (int i = 0; i < objects.size(); i++)
	{
		int objLoc = s.GetObjectLocation(i);
		if (objLoc == kDead)
			continue;
		int objectY = GetY(objLoc);
		for (int j = 0; j < objects[i].size(); j++)
		{
			if (objectY+GetY(objects[i][j]) >= height-2)
			{
				s.SetObjectLocation(i, kDead);
				break;
			}
		}
	}
	return true;
}

bool SnakeBird::ApplyPartialAction(SnakeBirdState &s, SnakeBirdAction act, SnakeBirdAnimationStep &step) const
{
//	ApplyAction(s, act);
//	return true;
	
	if (step.anim == kNeedsInitialization)
	{
		step.anim = kMovement;
		step.a = act;
	}
	int offset;
	snakeDir opposite;
	snakeDir lastAction = (snakeDir)act.direction;
	switch (step.a.direction)
	{
		case kLeft: offset = -height; opposite = kRight; break;
		case kRight: offset = +height; opposite = kLeft; break;
		case kUp: offset = -1; opposite = kDown; break;
		case kDown: offset = 1; opposite = kUp; break;
	}
	
	if (step.anim == kMovement)
	{
		// exited level
		DoFirstMovement(step.a, offset, opposite, s);
		step.anim = kInitialTeleport;
		return false;
	}
	
	// Small hack to make the following loop more generic:
	// Add current bird to list of pushed birds
	// This will cause us to check it for portals
	if (step.anim == kInitialTeleport)
	{
		step.a.pushed |= (1<<step.a.bird);
	}
	else {
		lastAction = kDown;
		opposite = kUp;
	}
	
	// Now for gravity and portals.
	// 1. Check if any portals are activated.
	// 2. Then try all moved objects to see if they can be pushed down.
	// 2a. Merge the pushable objects into a single bit vector
	// 3. If no objects moved, stop; else repeat to 1.
	bool success = Render(s);
	if (!success) // someone died as a result of the action
		return true;

	if (step.anim == kTeleport || step.anim == kInitialTeleport)
	{
		bool didTeleport = HandleTeleports(s, step.a, lastAction, opposite, step);
		if (didTeleport)
		{
			step.anim = kFall;
			return false;
		}
	}
		
	bool didFall = DoFall(step.a, s);
	if (!didFall)
		return true;
	
	step.anim = kTeleport;
	return false;
}

//SnakeBirdAction GetAction(const SnakeBirdState &s1, const SnakeBirdState &s2) const;
void SnakeBird::ApplyAction(SnakeBirdState &s, SnakeBirdAction a) const
{
	SnakeBirdAnimationStep step;
	step.anim = kMovement;
	int offset;
	snakeDir opposite;
	snakeDir lastAction = (snakeDir)a.direction;
	switch (a.direction)
	{
		case kLeft: offset = -height; opposite = kRight; break;
		case kRight: offset = +height; opposite = kLeft; break;
		case kUp: offset = -1; opposite = kDown; break;
		case kDown: offset = 1; opposite = kUp; break;
	}
	
	if (step.anim == kMovement)
	{
		// exited level
		DoFirstMovement(a, offset, opposite, s);
		step.anim = kInitialTeleport;
	}
	
	// Small hack to make the following loop more generic:
	// Add current bird to list of pushed birds
	// This will cause us to check it for portals
	a.pushed |= (1<<a.bird);
	
	// Now for gravity and portals.
	// 1. Check if any portals are activated.
	// 2. Then try all moved objects to see if they can be pushed down.
	// 2a. Merge the pushable objects into a single bit vector
	// 3. If no objects moved, stop; else repeat to 1.
	while (true) {
		bool success = Render(s);
		if (!success) // someone died as a result of the action
			return;

		if (step.anim == kTeleport)
			HandleTeleports(s, a, kDown, kUp, step);
		else
			HandleTeleports(s, a, lastAction, opposite, step);
		step.anim = kFall;
		
		bool didFall = DoFall(a, s);
		if (!didFall)
			return;
		
		step.anim = kTeleport;
	}
}

bool SnakeBird::HandleTeleports(SnakeBirdState &s, SnakeBirdAction &a,
								snakeDir lastAction, snakeDir opposite, SnakeBirdAnimationStep step) const
{
	const SnakeBirdWorldObject objs[8] = {kSnake1, kSnake2, kSnake3, kSnake4, kBlock1, kBlock2, kBlock3, kBlock4};
	bool didTeleport = false;
	// Check for snakes which are now on portals
	// *and* didn't have an adjacent segment on a portal in the last time step
	for (int i = 0; i < 4 && (a.pushed>>i); i++)
	{
		int snake = i;
		if ((a.pushed>>i)&0x1) // ignore if it wasn't moved
		{
			int newloc = s.GetSnakeHeadLoc(snake);
			int piece = newloc;
			for (int x = 0; x < s.GetSnakeLength(snake); x++)
			{
				//int lastPiece = piece;
				// in the first round of gravity/portals
				// only check the head of the bird that was moved
				if (i == a.bird && step.anim == kInitialTeleport && x > 0)
				{
					break;
				}
				if (x > 0)
				{
					switch (s.GetSnakeDir(snake, x-1))
					{
						case kLeft: piece-=height; break;
						case kRight: piece+=height; break;
						case kUp: piece-=1; break;
						case kDown: piece+=1; break;
					}
				}
				int p1, p2 = -1;
				if (piece == portal1Loc)
				{
					p1 = portal1Loc;
					p2 = portal2Loc;
				}
				else if (piece == portal2Loc)
				{
					p1 = portal2Loc;
					p2 = portal1Loc;
				}

				// Did the snake just move to adjacent locations?
				if (p2 != -1)
				{
					// We are looking at the snake location after it was moved.
					// Need to check in forward and backwards directions
					if (x > 0 && s.GetSnakeDir(snake, x-1) == opposite)
					{ p2 = -1; }
					else if (x < s.GetSnakeLength(snake)-1 && s.GetSnakeDir(snake, x) == lastAction)
					{ p2 = -1; }
				}

				// We have a portal to teleport to
				if (p2 != -1)
				{
					int xChange = -GetX(p1)+GetX(p2);
					int yChange = -GetY(p1)+GetY(p2);

					// Validate if the object can be placed in the new location
					bool valid = true;
					int newPiece = GetIndex(GetX(newloc)+xChange, GetY(newloc)+yChange);
					//for (int v = 0; v < objects[snake].size(); v++)
					for (int v = 0; v < s.GetSnakeLength(snake); v++)
					{
						if (v > 0)
						{
							switch (s.GetSnakeDir(snake, v-1))
							{
								case kLeft: newPiece-=height; break;
								case kRight: newPiece+=height; break;
								case kUp: newPiece-=1; break;
								case kDown: newPiece+=1; break;
							}
						}

						if (GetX(newPiece) < 0 ||
							GetX(newPiece) >= width ||
							GetY(newPiece) < 0 ||
							GetY(newPiece) >= height)
						{
							valid = false;
							break;
						}
						
						if (((render[newPiece]&kSnakeMask) == kSnakeMask &&
							 render[newPiece] != objs[snake]) || // hit another snake
							((world[newPiece]&kGroundMask) == kGroundMask) || // hit ground
							((render[newPiece]&kBlockMask) == kBlockMask)) // hit piece
						{
							valid = false;
							break;
						}
					}
					if (valid)
					{
						int offsetFromPortalX = GetX(newloc)+xChange;
						int offsetFromPortalY = GetY(newloc)+yChange;
						s.SetSnakeHeadLoc(i, GetIndex(offsetFromPortalX, offsetFromPortalY));
						didTeleport = true;
						bool success = Render(s);
						assert(success == true);
//						if (!success) // someone died as a result of the action
//							return;
					}
				}
			}
		}
	}
	
	// Check for objects which are now on portals
	// and didn't have an adjacent segment on a portal in the last step
	// If an object teleports we have to re-render
	// Note - only one object can teleport, so don't check if a snake teleported
	for (int i = 4; i < 8 && (a.pushed>>i) && !didTeleport; i++)
	{
		int object = i-4;
		if ((a.pushed>>i)&0x1) // ignore if it wasn't moved
		{
			int newloc = s.GetObjectLocation(object);
			if (newloc == kDead)
				continue;
			for (int x = 0; x < objects[object].size(); x++)
			{
				int piece = GetIndex(GetX(objects[object][x])+GetX(newloc),
									 GetY(objects[object][x])+GetY(newloc));
				int p1=-1, p2 = -1;
				if (piece == portal1Loc)
				{
					p1 = portal1Loc;
					p2 = portal2Loc;
				}
				else if (piece == portal2Loc)
				{
					p1 = portal2Loc;
					p2 = portal1Loc;
				}
				// Check if our last action suggests that the object was in the portal previously
				if (p2 != -1)
				{
					switch (lastAction)
					{
						case kLeft:
							if (p1-height >= 0 && render[p1-height] == objs[i]) // can't teleport
								p2 = -1;
							break;
						case kRight:
							if (p1+height < width*height && render[p1+height] == objs[i]) // can't teleport
								p2 = -1;
							break;
						case kUp:
							if (p1-1 >= 0 && render[p1-1] == objs[i]) // can't teleport
								p2 = -1;
							break;
						case kDown:
							if (p1+1 < width*height && render[p1+1] == objs[i]) // can't teleport
								p2 = -1;
							break;
					}
				}
				
				if (p2 != -1)
				{
					// TODO: Check if it can be placed in new loc relative to self
					// TODO: Check if it is offscreen
					int xChange = -GetX(p1)+GetX(p2);
					int yChange = -GetY(p1)+GetY(p2);

					// Validate if the object can be placed in the new location
					bool valid = true;
					for (int v = 0; v < objects[object].size(); v++)
					{
						if (GetX(newloc)+GetX(objects[object][v])+xChange < 0 ||
							GetX(newloc)+GetX(objects[object][v])+xChange >= width ||
							GetY(newloc)+GetY(objects[object][v])+yChange < 0 ||
							GetY(newloc)+GetY(objects[object][v])+yChange >= height)
						{
							valid = false;
							break;
						}
						int newPiece = GetIndex(GetX(newloc)+GetX(objects[object][v])+xChange,
												GetY(newloc)+GetY(objects[object][v])+yChange);
						if (((render[newPiece]&kSnakeMask) == kSnakeMask) || // hit snake
							((world[newPiece]&kGroundMask) == kGroundMask) || // hit ground
							((render[newPiece]&kBlockMask) == kBlockMask && render[newPiece] != objs[i])) // hit another piece
						{
							valid = false;
							break;
						}
					}
					if (valid)
					{
						int offsetFromPortalX = GetX(newloc)+xChange;
						int offsetFromPortalY = GetY(newloc)+yChange;
						s.SetObjectLocation(object, GetIndex(offsetFromPortalX, offsetFromPortalY));
						didTeleport = true;

						bool success = Render(s);
						assert(success == true);
//						if (!success) // someone died as a result of the action
//							return;
					}
				}
			}
		}
	}
	return didTeleport;
}

void SnakeBird::SetGroundType(int x, int y, SnakeBirdWorldObject o)
{
	// TODO: check duplicates and removals for fruit and blocks
	if (y == height-1)
	{
		printf("Error - cannot add terrain at bottom of map\n");
		return;
	}
	
	if ((o&kBlockMask) == kBlockMask) // block
	{
		int which = -1;
		switch (o)
		{
			case kBlock1:
				which = 0;
				break;
			case kBlock2:
				which = 1;
				break;
			case kBlock3:
				which = 2;
				break;
			case kBlock4:
				which = 3;
				break;
			default:
				assert(!"Can't get here - object is a block");
				break;
		}
		assert(which != -1);
		// objects are stored as offsets from 0.
		// The offset in the original world is in startState
		int newX = x, newY=y;
		int xOffset = 0, yOffset = 0;
		if (objects[which].size() > 0) // other objects - get their offset base
		{
			xOffset = GetX(startState.GetObjectLocation(which));
			yOffset = GetY(startState.GetObjectLocation(which));
		}
		
		// Get new base location (minx/y)
		for (int i = 0; i < objects[which].size(); i++)
		{
			newX = std::min(newX, GetX(objects[which][i])+xOffset);
			newY = std::min(newY, GetY(objects[which][i])+yOffset);
		}
		startState.SetObjectLocation(which, GetIndex(newX, newY));
		for (int i = 0; i < objects[which].size(); i++)
		{
			// reset locations based on new base location
			objects[which][i] = GetIndex(GetX(objects[which][i])+xOffset - newX,
										 GetY(objects[which][i])+yOffset - newY);
		}
		// add new piece of new object
		objects[which].push_back(GetIndex(x-newX, y-newY));
		
		return;
	}
	
	world[GetIndex(x, y)] = o;

	if (o == kFruit)
	{
		fruit.push_back(GetIndex(x, y));
	}
	if (o == kPortal1)
		portal1Loc = GetIndex(x, y);
	if (o == kPortal2)
		portal2Loc = GetIndex(x, y);
}

int SnakeBird::GetFruitOffset(int index) const
{
	for (int x = 0; x < fruit.size(); x++)
		if (fruit[x] == index)
			return x;
	assert(false);
	return -1;
}


SnakeBirdWorldObject SnakeBird::GetGroundType(int x, int y)
{
	return world[GetIndex(x, y)];
}


int SnakeBird::GetIndex(int x, int y) const
{
	return x*height+y;
}

int SnakeBird::GetX(int index) const
{
	return index/height;
}

int SnakeBird::GetY(int index) const
{
	return index%height;
}

void SnakeBird::Draw(Graphics::Display &display) const
{
	display.FillRect({-1, -1, 1, 0.5}, rgbColor::mix(Colors::cyan, Colors::lightblue, 0.5));
	display.FillRect({-1, 0.5, 1, 1}, Colors::darkblue+Colors::darkred);
	for (int x = 0; x < width*height; x++)
	{
		Graphics::point p = GetCenter(GetX(x), GetY(x));
		double radius = GetRadius()*0.95;
		switch (world[x])
		{
			case kEmpty:
				break;//display.FillSquare(p, GetRadius(), Colors::lightblue);  break;
			case kGround:
				switch ((GetX(x)*13*+11*GetY(x))%6)
				{
					case 0: display.FillSquare(p, GetRadius(), Colors::brown); break;
					case 1: display.FillSquare(p, GetRadius(), Colors::brown); break;
					case 2: display.FillSquare(p, GetRadius(), Colors::brown+Colors::gray); break;
					case 3: display.FillSquare(p, GetRadius(), Colors::brown+Colors::gray); break;
					case 4: display.FillSquare(p, GetRadius(), Colors::brown*0.8+Colors::darkgreen); break;
					case 5: display.FillSquare(p, GetRadius(), Colors::brown*0.8+Colors::darkgreen); break;
				}
				break;
			case kSpikes:
				display.FillNGon(p, radius, 3, 0, Colors::darkgray);
				display.FillNGon(p, radius, 3, 60, Colors::gray*0.75f);
				break;
				break;
			case kPortal1:
			case kPortal2:
				display.FillCircle(p, radius, Colors::red);
				display.FillCircle(p, radius*0.75, Colors::blue);
				display.FillCircle(p, radius*0.5, Colors::green);
				display.FillCircle(p, radius*0.25, Colors::purple);
				break;
			case kExit:
				display.FillNGon(p, radius, 5, 0, Colors::yellow);
				display.FillNGon(p, radius*0.66, 5, 36, Colors::orange);
				display.FillNGon(p, radius*0.25, 5, 54, Colors::red);
				break;
			case kFruit:
//				p.x-=radius/4;
//				display.FillCircle(p, radius/2.0, Colors::green);
//				p.x+=radius/2;
//				display.FillCircle(p, radius/2.0, Colors::green);
				break;
		}
	}
	
//	// draw grid
//	for (int x = 0; x < width; x++)
//	{
//		Graphics::point p1, p2;
//		p1.x = GetX(GetIndex(x, 0))-GetRadius();
//		p1.y = GetY(GetIndex(x, 0))-GetRadius();
//		p2.x = p1.x;
//		p2.y = GetY(GetIndex(x, 16))-GetRadius();
//		display.DrawLine(p1, p2, 0.5, Colors::darkgray);
//	}
//	for (int y = 0; y < 16; y++)
//	{
//		Graphics::point p1, p2;
//		p1.x = GetX(GetIndex(0, y))-GetRadius();
//		p1.y = GetY(GetIndex(width, y))-GetRadius();
//		p2.y = p1.y;
//		display.DrawLine(p1, p2, 0.5, Colors::darkgray);
//	}
}

Graphics::point SnakeBird::GetCenter(int x, int y) const
{
	Graphics::point p;
	p.x = -1+2*x*GetRadius()+GetRadius();
	p.y = -1+2*y*GetRadius()+GetRadius();
	return p;
}

float SnakeBird::GetRadius() const
{
	return 2.0/(std::max(width, height)*2.0);
}

void SnakeBird::Draw(Graphics::Display &display, const SnakeBirdState&s) const
{
	Draw(display, s, -1);
}

void SnakeBird::Draw(Graphics::Display &display, const SnakeBirdState&s, int active) const
{
	rgbColor c[4] = {Colors::red, Colors::blue, Colors::green, Colors::yellow};
	rgbColor objColors[4] = {Colors::red*0.5, Colors::blue*0.5, Colors::green*0.5, Colors::yellow*0.5};
	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		// get head loc
		int index = s.GetSnakeHeadLoc(snake);
		if (index == kInGoal)
			continue;
		
		int x = GetX(index);
		int y = GetY(index);
//		display.FillSquare(p, GetRadius(), Colors::red);
		int len = s.GetSnakeBodyEnd(snake)-s.GetSnakeBodyEnd(snake-1);

		if (len%2)
			DrawSnakeSegment(display, x, y, c[snake]*0.8, true, false, (active==-1)||(active==snake), kUp, s.GetSnakeDir(snake, 0));
		else
			DrawSnakeSegment(display, x, y, c[snake], true, false, (active==-1)||(active==snake), kUp, s.GetSnakeDir(snake, 0));

		int cnt = 0;
		for (int t = 0; t < len; t++)
		{
			switch (s.GetSnakeDir(snake, t))
			{
				case kUp:
					y-=1;
					break;
				case kDown:
					y+=1;
					break;
				case kRight:
					x+=1;
					break;
				case kLeft:
					x-=1;
					break;
				default:
					break;
			}
			bool tail = t+1==(s.GetSnakeBodyEnd(snake)-s.GetSnakeBodyEnd(snake-1));
			if ((++cnt+len)&1)
				DrawSnakeSegment(display, x, y, c[snake]*0.8, false, tail, false, s.GetSnakeDir(snake, t), tail?kUp:s.GetSnakeDir(snake, t+1));
			else
				DrawSnakeSegment(display, x, y, c[snake], false, tail, false, s.GetSnakeDir(snake, t), tail?kUp:s.GetSnakeDir(snake, t+1));
		}
	}
	
	// draw objects
	static std::vector<Graphics::point> points;
	for (int x = 0; x < 4; x++)
	{
		points.clear();
		if (s.GetObjectLocation(x) == kDead)
			continue;
		for (int i = 0; i < objects[x].size(); i++)
		{
			Graphics::point p = GetCenter(GetX(objects[x][i])+GetX(s.GetObjectLocation(x)),
										  GetY(objects[x][i])+GetY(s.GetObjectLocation(x)));
			double radius = GetRadius();//*0.95;
			display.FrameSquare(p, radius-radius*0.3, objColors[x], radius*0.6);
			points.push_back(p);
		}
		if (points.size() > 0)
			display.DrawLineSegments(points, GetRadius()*0.3, objColors[x]*0.8);
	}
	for (int x = 0; x < fruit.size(); x++)
	{
		if (s.GetFruitPresent(x))
		{
			Graphics::point p = GetCenter(GetX(fruit[x]), GetY(fruit[x]));

			display.FillCircle(p, GetRadius()*0.8, Colors::orange);
		}
	}
}

void SnakeBird::DrawSnakeSegment(Graphics::Display &display, int x, int y, const rgbColor &color, bool head, bool tail, bool awake, snakeDir dirFrom, snakeDir dirTo) const
{
	Graphics::point p = GetCenter(x, y);
	const float cornerWidth = 0.75;
	float offset = cornerWidth*GetRadius();
	
	Graphics::rect r(p, GetRadius());
	r.top+=offset;
	r.bottom-=offset;
	display.FillRect(r, color);
	r.top-=offset;
	r.bottom+=offset;
	r.left+=offset;
	r.right-=offset;
	display.FillRect(r, color);

	if ((!head && (dirFrom == kDown || dirFrom == kRight)) ||
		(!tail && (dirTo == kUp || dirTo == kLeft)))
	{
		display.FillSquare(p+Graphics::point(offset-GetRadius(), offset-GetRadius()), offset, color);
	}
	else {
		display.FillCircle(p+Graphics::point(offset-GetRadius(), offset-GetRadius()), offset, color);
	}

	if ((!head && (dirFrom == kUp || dirFrom == kRight)) ||
		(!tail && (dirTo == kDown || dirTo == kLeft)))
	{
		display.FillSquare(p+Graphics::point(offset-GetRadius(), -offset+GetRadius()), offset, color);
	}
	else {
		display.FillCircle(p+Graphics::point(offset-GetRadius(), -offset+GetRadius()), offset, color);
	}
	
	if ((!head && (dirFrom == kDown || dirFrom == kLeft)) ||
		(!tail && (dirTo == kUp || dirTo == kRight)))
	{
		display.FillSquare(p+Graphics::point(-offset+GetRadius(), offset-GetRadius()), offset, color);
	}
	else {
		display.FillCircle(p+Graphics::point(-offset+GetRadius(), offset-GetRadius()), offset, color);
	}

	if ((!head && (dirFrom == kUp || dirFrom == kLeft)) ||
		(!tail && (dirTo == kDown || dirTo == kRight)))
	{
		display.FillSquare(p+Graphics::point(-offset+GetRadius(), -offset+GetRadius()), offset, color);
	}
	else {
		display.FillCircle(p+Graphics::point(-offset+GetRadius(), -offset+GetRadius()), offset, color);
	}

	if (head)
	{
		// draw eyes
		p.x+=GetRadius()*0.2;
		display.FillCircle(p, GetRadius()*0.2, awake?Colors::white:(color*0.5));
		if (awake)
			display.FillCircle(p, GetRadius()*0.1, Colors::black);
		p.x-=2*GetRadius()*0.2;
		display.FillCircle(p, GetRadius()*0.2, awake?Colors::white:(color*0.5));
		if (awake)
			display.FillCircle(p, GetRadius()*0.1, Colors::black);
		p.x+=GetRadius()*0.2;
		p.y+=2*GetRadius()*0.25;
		display.FillNGon(p, GetRadius()*0.3, 3, 240, Colors::orange);
	}
	
	
	// in spikes!
	if (world[GetIndex(x, y)] == kSpikes)
	{
		display.FillNGon(p, GetRadius()*0.95, 3, 0, Colors::darkgray);
		display.FillNGon(p, GetRadius()*0.95, 3, 60, Colors::gray*0.75f);
		return;
	}
}


void SnakeBird::DrawLine(Graphics::Display &display, const SnakeBirdState &x, const SnakeBirdState &y, float width) const
{
	
}


}
