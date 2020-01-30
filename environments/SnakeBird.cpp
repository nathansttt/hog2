//
//  SnakeBird.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 1/26/20.
//  Copyright © 2020 University of Denver. All rights reserved.
//

#include "SnakeBird.h"

namespace SnakeBird {

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
	for (int snake = 0; snake < s.GetNumSnakes(); snake++)
	{
		// first pass - don't allow to move back onto yourself or onto other obstacles
		int loc = s.GetSnakeHeadLoc(snake);
		a.bird = snake;
		
		// kDown
		if (world[loc+1] != kGround && world[loc+1] != kSpikes && GetY(loc)+1 < 16) // not blocked directly
		{
			if (s.GetSnakeDir(snake, 0) != kDown) // not blocked by snake
			{
				a.direction = kDown;
				actions.push_back(a);
			}
		}

		// kUp
		if (world[loc-1] != kGround && world[loc-1] != kSpikes && GetY(loc) > 0) // not blocked directly
		{
			if (s.GetSnakeDir(snake, 0) != kUp) // not blocked by snake
			{
				a.direction = kUp;
				actions.push_back(a);
			}
		}

		// kRight
		if (world[loc+16] != kGround && world[loc+16] != kSpikes && GetX(loc)+1 < 20) // not blocked directly
		{
			if (s.GetSnakeDir(snake, 0) != kRight) // not blocked by snake
			{
				a.direction = kRight;
				actions.push_back(a);
			}
		}

		// kLeft
		if (world[loc-16] != kGround && world[loc-16] != kSpikes && GetX(loc) > 0) // not blocked directly
		{
			if (s.GetSnakeDir(snake, 0) != kLeft) // not blocked by snake
			{
				a.direction = kLeft;
				actions.push_back(a);
			}
		}
	}
}

//SnakeBirdAction GetAction(const SnakeBirdState &s1, const SnakeBirdState &s2) const;
void SnakeBird::ApplyAction(SnakeBirdState &s, SnakeBirdAction a) const
{
	switch (a.direction)
	{
		case kLeft:
			s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)-16);
			s.InsertSnakeDir(a.bird, kRight);
			break;
		case kRight:
			s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+16);
			s.InsertSnakeDir(a.bird, kLeft);
			break;
		case kUp:
			s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)-1);
			s.InsertSnakeDir(a.bird, kDown);
			break;
		case kDown:
			s.SetSnakeHeadLoc(a.bird, s.GetSnakeHeadLoc(a.bird)+1);
			s.InsertSnakeDir(a.bird, kUp);
			break;
	}
}

void SnakeBird::SetGroundType(int x, int y, SnakeBirdWorldObject o)
{
	world[GetIndex(x, y)] = o;
}

SnakeBirdWorldObject SnakeBird::GetGroundType(int x, int y)
{
	return world[GetIndex(x, y)];
}


int SnakeBird::GetIndex(int x, int y) const
{
	return x*16+y;
}

int SnakeBird::GetX(int index) const
{
	return index/16;
}

int SnakeBird::GetY(int index) const
{
	return index%16;
}

void SnakeBird::Draw(Graphics::Display &display)
{
	display.FillRect({-1, -1, 1, 0}, rgbColor::mix(Colors::cyan, Colors::lightblue, 0.5));
	display.FillRect({-1, 0, 1, 1}, Colors::darkblue);
	for (int x = 0; x < 320; x++)
	{
		Graphics::point p = GetCenter(GetX(x), GetY(x));
		double radius = GetRadius()*0.95;
		switch (world[x])
		{
			case kEmpty:
				break;//display.FillSquare(p, GetRadius(), Colors::lightblue);  break;
			case kGround:
				display.FillSquare(p, radius, Colors::brown);
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
				p.x-=radius/4;
				display.FillCircle(p, radius/2.0, Colors::green);
				p.x+=radius/2;
				display.FillCircle(p, radius/2.0, Colors::green);
				break;
		}
	}
	
//	// draw grid
//	for (int x = 0; x < 20; x++)
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
//		p1.y = GetY(GetIndex(20, y))-GetRadius();
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

		int x = GetX(index);
		int y = GetY(index);
//		display.FillSquare(p, GetRadius(), Colors::red);
		DrawSnakeSegment(display, x, y, c[snake], true, false, (active==-1)||(active==snake), kUp, s.GetSnakeDir(snake, 0));

		int cnt = 0;
		for (int t = 0; t < s.GetSnakeBodyEnd(snake)-s.GetSnakeBodyEnd(snake-1); t++)
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
			if ((++cnt)&1)
				DrawSnakeSegment(display, x, y, c[snake]*0.8, false, tail, false, s.GetSnakeDir(snake, t), tail?kUp:s.GetSnakeDir(snake, t+1));
			else
				DrawSnakeSegment(display, x, y, c[snake], false, tail, false, s.GetSnakeDir(snake, t), tail?kUp:s.GetSnakeDir(snake, t+1));
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
