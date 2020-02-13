//
//  SnakeBird.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/26/20.
//  Copyright © 2020 University of Denver. All rights reserved.
//

#include "SnakeBird.h"

namespace SnakeBird {

SnakeBird::SnakeBird(int width, int height)
:width(width), height(height)
{
	assert(width*height < 512);
}

void SnakeBird::GetSuccessors(const SnakeBirdState &nodeID, std::vector<SnakeBirdState> &neighbors) const
{
	
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

void SnakeBird::GetActions(const SnakeBirdState &s, std::vector<SnakeBirdAction> &actions) const
{
	actions.clear();
	SnakeBirdAction a;
	
	SnakeBirdWorldObject obj[4] = {kSnake1, kSnake2, kSnake3, kSnake4};
	std::fill_n(&render[0], 512, kEmpty);
	
	// render snakes into world
	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		int loc = s.GetSnakeHeadLoc(snake);
		render[loc] = obj[snake];
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
			render[loc] = obj[snake];
		}
	}
	// TODO: render objects as well
	{
		
	}
	
//	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
//	{
//		int loc = s.GetSnakeHeadLoc(snake);
//		render[loc] = obj[snake];
//		int len = s.GetSnakeBodyEnd(snake)-s.GetSnakeBodyEnd(snake-1);
//		for (int x = 0; x < len; x++)
//		{
//			switch (s.GetSnakeDir(snake, x))
//			{
//				case kLeft: loc-=height; break;
//				case kRight: loc+=height; break;
//				case kUp: loc-=1; break;
//				case kDown: loc+=1; break;
//			}
//			if (loc-1 < 0 || render[loc-1] == 3)
//				canPushDir[snake][kLeft] = false;
//		}
//	}

	//	for (int y = 0; y < height; y++)
//	{
//		for (int x = 0; x < width; x++)
//		{
//			printf("%c", 'a'+render[GetIndex(x, y)]);
//		}
//		printf("\n");
//	}
	
	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		// first pass - don't allow to move back onto yourself or onto other obstacles
		int loc = s.GetSnakeHeadLoc(snake);
		a.bird = snake;
		a.pushed[0] = kNothingPushed;

		// kDown - can never push any object down due to gravity
		if (world[loc+1] != kGround && world[loc+1] != kSpikes && GetY(loc)+1 < height && render[loc+1] == kEmpty)
		{
			if (s.GetSnakeDir(snake, 0) != kDown) // not blocked by snake
			{
				a.direction = kDown;
				actions.push_back(a);
			}
		}

		// kUp
		if (world[loc-1] != kGround && world[loc-1] != kSpikes && GetY(loc) > 0 && render[loc-1] != obj[snake])
		{
			if (s.GetSnakeDir(snake, 0) != kUp && (render[loc-1] == kEmpty || CanPush(s, snake, render[loc-1], kUp, a, 0))) // snake not blocked by self
			{
				a.direction = kUp;
				actions.push_back(a);
			}
		}
		
		a.pushed[0] = kNothingPushed;
		// kRight
		if (world[loc+height] != kGround && world[loc+height] != kSpikes && GetX(loc)+1 < width && render[loc+height] != obj[snake])
		{
			if (s.GetSnakeDir(snake, 0) != kRight && (render[loc+height] == kEmpty || CanPush(s, snake, render[loc+height], kRight, a, 0))) // not blocked by snake
			{
				a.direction = kRight;
				actions.push_back(a);
			}
		}
		
		a.pushed[0] = kNothingPushed;
		// kLeft
		if (world[loc-height] != kGround && world[loc-height] != kSpikes && GetX(loc) > 0 && render[loc-height] != obj[snake])
		{
			if (s.GetSnakeDir(snake, 0) != kLeft && (render[loc-height] == kEmpty || CanPush(s, snake, render[loc-height], kLeft, a, 0))) // not blocked by snake
			{
				a.direction = kLeft;
				actions.push_back(a);
			}
		}
	}
}

bool SnakeBird::CanPush(const SnakeBirdState &s, int snake, SnakeBirdWorldObject obj, snakeDir dir,
						SnakeBirdAction &a, int pushObject) const
{
	if (pushObject >= kMaxPushedObjects)
		return false;
	int pushed;
	// can't push yourself
	if (pushObject+1 < kMaxPushedObjects)
	a.pushed[pushObject+1] = kNothingPushed;
	switch (obj) {
		case kSnake1: if (snake == 0) return false; pushed = 0; break;
		case kSnake2: if (snake == 1) return false; pushed = 1; break;
		case kSnake3: if (snake == 2) return false; pushed = 2; break;
		case kSnake4: if (snake == 3) return false; pushed = 3; break;
		default: return false; break;
	}
	// already pushing this
	for (int x = 0; x < pushObject; x++)
		if (a.pushed[x] == pushed)
			return true;
	// not pushing; put in queue
	a.pushed[pushObject] = pushed;
	// Render pushed obj one step in (dir)
	int loc = s.GetSnakeHeadLoc(pushed);
	switch (dir)
	{
		case kLeft: if (loc < height) return false; loc-=height; break;
		case kRight: if (loc+height > width*height) return false; loc+=height; break;
		case kUp: if (loc == 0) return false; loc-=1; break;
		case kDown: if (loc+1 == width*height) return false; loc+=1; break;
	}
	if (world[loc]!=kEmpty)
		return false;
	// If moved into location is occupied by another object (not us), see if it can be pushed
	if ((render[loc] != kEmpty && render[loc] != obj))
	{
		if (!CanPush(s, snake, render[loc], dir, a, pushObject+1))
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
		if (world[loc]!=kEmpty)
			return false;
		// If moved into location is occupied by another object (not us), see if it can be pushed
		if ((render[loc] != kEmpty && render[loc] != obj))
		{
			if (!CanPush(s, snake, render[loc], dir, a, pushObject+1))
				return false;
		}
	}
	// temporarily return false to not push other snakes
	//return ;
	return true;
}

void SnakeBird::Gravity(SnakeBirdState &s) const
{
	SnakeBirdWorldObject obj[4] = {kSnake1, kSnake2, kSnake3, kSnake4};
	std::fill_n(&render[0], 512, kEmpty);
	// render snakes into world
	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		int loc = s.GetSnakeHeadLoc(snake);
		render[loc] = obj[snake];
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
			render[loc] = obj[snake];
		}
	}
}


//SnakeBirdAction GetAction(const SnakeBirdState &s1, const SnakeBirdState &s2) const;
void SnakeBird::ApplyAction(SnakeBirdState &s, SnakeBirdAction a) const
{
	switch (a.direction)
	{
		case kLeft:
			// exited level
			if (world[s.GetSnakeHeadLoc(a.bird)-height] == kExit && s.KFruitEaten(fruit.size()))
			{
				s.SetSnakeHeadLoc(a.bird, kInGoal);
				break;
			}

			// eating fruit
			if (world[s.GetSnakeHeadLoc(a.bird)-height] == kFruit &&
				s.GetFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)-height)))
			{
				s.ToggleFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)-height));
				for (int x = a.bird; x < s.GetNumSnakes(); x++)
					s.SetSnakeBodyEnd(x, s.GetSnakeBodyEnd(x)+1);
				s.InsertSnakeHeadDir(a.bird, kRight);
				s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)-height);
			}
			else {
				s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)-height);
				s.InsertSnakeDir(a.bird, kRight);
				for (int i = 0; i < 4&&a.pushed[i] != kNothingPushed; i++)
					s.SetSnakeHeadLoc(a.pushed[i], s.GetSnakeHeadLoc(a.pushed[i])-height);
			}
			break;
		case kRight:
			// exited level
			if (world[s.GetSnakeHeadLoc(a.bird)+height] == kExit && s.KFruitEaten(fruit.size()))
			{
				s.SetSnakeHeadLoc(a.bird, kInGoal);
				break;
			}

			// eating fruit
			if (world[s.GetSnakeHeadLoc(a.bird)+height] == kFruit &&
				s.GetFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)+height)))
			{
				s.ToggleFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)+height));
				for (int x = a.bird; x < s.GetNumSnakes(); x++)
					s.SetSnakeBodyEnd(x, s.GetSnakeBodyEnd(x)+1);
				s.InsertSnakeHeadDir(a.bird, kLeft);
				s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+1);
			}
			else {
				s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+height);
				s.InsertSnakeDir(a.bird, kLeft);
				for (int i = 0; i < 4&&a.pushed[i] != kNothingPushed; i++)
					s.SetSnakeHeadLoc(a.pushed[i], s.GetSnakeHeadLoc(a.pushed[i])+height);
			}
			break;
		case kUp:
			// exited level
			if (world[s.GetSnakeHeadLoc(a.bird)-1] == kExit && s.KFruitEaten(fruit.size()))
			{
				s.SetSnakeHeadLoc(a.bird, kInGoal);
				break;
			}

			// eating fruit
			if (world[s.GetSnakeHeadLoc(a.bird)-1] == kFruit &&
				s.GetFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)-1)))
			{
				s.ToggleFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)-1));
				for (int x = a.bird; x < s.GetNumSnakes(); x++)
					s.SetSnakeBodyEnd(x, s.GetSnakeBodyEnd(x)+1);
				s.InsertSnakeHeadDir(a.bird, kDown);
				s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)-1);
			}
			else {
				s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)-1);
				s.InsertSnakeDir(a.bird, kDown);
				for (int i = 0; i < 4&&a.pushed[i] != kNothingPushed; i++)
					s.SetSnakeHeadLoc(a.pushed[i], s.GetSnakeHeadLoc(a.pushed[i])-1);
			}
			break;
		case kDown:
			// exited level
			if (world[s.GetSnakeHeadLoc(a.bird)+1] == kExit && s.KFruitEaten(fruit.size()))
			{
				s.SetSnakeHeadLoc(a.bird, kInGoal);
				break;
			}

			// eating fruit
			if (world[s.GetSnakeHeadLoc(a.bird)+1] == kFruit &&
				s.GetFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)+1)))
			{
				s.ToggleFruitPresent(GetFruitOffset(s.GetSnakeHeadLoc(a.bird)+1));
				for (int x = a.bird; x < s.GetNumSnakes(); x++)
					s.SetSnakeBodyEnd(x, s.GetSnakeBodyEnd(x)+1);
				s.InsertSnakeHeadDir(a.bird, kUp);
				s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+1);
			}
			else {
				s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+1);
				s.InsertSnakeDir(a.bird, kUp);
				// removed code -- can't push down
//				if (a.pushed[0] != kNothingPushed)
//					s.SetSnakeHeadLoc(a.pushed[0], s.GetSnakeHeadLoc(a.pushed[0])+1);
			}
			break;
	}
}

void SnakeBird::SetGroundType(int x, int y, SnakeBirdWorldObject o)
{
	world[GetIndex(x, y)] = o;
	if (o == kFruit)
	{
		fruit.push_back(GetIndex(x, y)); // TODO: check duplicates
	}
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

void SnakeBird::Draw(Graphics::Display &display)
{
	display.FillRect({-1, -1, 1, 0}, rgbColor::mix(Colors::cyan, Colors::lightblue, 0.5));
	display.FillRect({-1, 0, 1, 1}, Colors::darkblue);
	for (int x = 0; x < width*height; x++)
	{
		Graphics::point p = GetCenter(GetX(x), GetY(x));
		double radius = GetRadius()*0.95;
		switch (world[x])
		{
			case kEmpty:
				break;//display.FillSquare(p, GetRadius(), Colors::lightblue);  break;
			case kGround:
				display.FillSquare(p, GetRadius(), Colors::brown);
				break;
			case kSpikes:
				display.FillNGon(p, radius, 3, 0, Colors::darkgray);
				display.FillNGon(p, radius, 3, 60, Colors::gray*0.75f);
				break;
				break;
			case kPortal:
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
	return 2.0/40.0;
}

void SnakeBird::Draw(Graphics::Display &display, const SnakeBirdState&s) const
{
	Draw(display, s, -1);
}

void SnakeBird::Draw(Graphics::Display &display, const SnakeBirdState&s, int active) const
{
	rgbColor c[4] = {Colors::red, Colors::blue, Colors::green, Colors::yellow};
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
}


void SnakeBird::DrawLine(Graphics::Display &display, const SnakeBirdState &x, const SnakeBirdState &y, float width) const
{
	
}


}
