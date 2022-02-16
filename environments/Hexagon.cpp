//
//  Hexagon.cpp
//  Hexagon
//
//  Created by Nathan Sturtevant on 11/13/21.
//  Copyright © 2021 MovingAI. All rights reserved.
//

#include "Hexagon.h"
#include <ctype.h>

Hexagon::Hexagon()
{
	
}
Hexagon::~Hexagon()
{
	
}

void Hexagon::LoadSolution(const char *file, HexagonState &s)
{
	Load(file, s, true);
}

void Hexagon::LoadPuzzle(const char *file, HexagonState &s)
{
	Load(file, s, false);
}

void Hexagon::Load(const char *file, HexagonState &s, bool solution)
{
	s.state.FillMax();
	this->solution.state.FillMax();
	FILE *f = fopen(file, "r");
	if (f == 0)
	{
		printf("Error opening file for read\n");
		return;
	}
	int h, w;
	fscanf(f, "type triangle\n");
	int cnt = fscanf(f, "height %d\nwidth %d\nmap\n", &h, &w);
	if (cnt != 2)
	{
		printf("Error reading height/width from file\n");
		return;
	}
	if (h != 6 || w != 11)
	{
		printf("Invalid height/width from file. Requires 6/11.\n");
		return;
	}
	// read constraints (if any)
	char buffer[255];
	fgets(buffer, 255, f);
	if (strncmp("Constraint:", buffer, 11) != 0)
	{
		printf("Error: expected 'Constraint:'\n");
		return;
	}
	char *tmp;
	
	// must touch diagonally
	tmp = strcasestr(buffer, "Diag");
	if (tmp != NULL)
	{
		diagPieces.resize(3);
		sscanf(&tmp[5], "%d %d %d", &diagPieces[0], &diagPieces[1], &diagPieces[2]);
	}
	
	tmp = strcasestr(buffer, "NoFlip");

	if (tmp != NULL)
	{
		noFlipPieces.resize(3);
		sscanf(&tmp[7], "%d %d %d", &noFlipPieces[0], &noFlipPieces[1], &noFlipPieces[2]);
	}

	tmp = strcasestr(buffer, "NotTouch");

	if (tmp != NULL)
	{
		notTouchPieces.resize(3);
		sscanf(&tmp[9], "%d %d %d", &notTouchPieces[0], &notTouchPieces[1], &notTouchPieces[2]);
	}

	tmp = strcasestr(buffer, " Touch"); // Add space -- otherwise matches NotTouch

	if (tmp != NULL)
	{
		touchPieces.resize(3);
		sscanf(&tmp[7], "%d %d %d", &touchPieces[0], &touchPieces[1], &touchPieces[2]);
	}
	
	for (int x = 0; x < 66; x++)
	{
		char c = fgetc(f);
		while (isspace(c))
			c = fgetc(f);
		if (c == 'x')
		{
			s.state.Set(x, -1);
			this->solution.state.Set(x, -1);
		}
		else if (isalpha(c))
		{
			this->solution.state.Set(x, tolower(c)-'a');
			if (solution)
			{
				s.state.Set(x, tolower(c)-'a');
			}
			else {
				s.state.Set(x, 11);
			}
		}
		else if (isdigit(c))
		{
			this->solution.state.Set(x, tolower(c)-'0');
			s.state.Set(x, tolower(c)-'0');
		}
	}
	fclose(f);
//	this->solution = s;
	
	// Get all of the colors
	for (int y = 0; y < 6; y++)
	{
		for (int x = 0; x < 11; x++)
		{
			if (!Valid(x, y))
				continue;
			int piece = this->solution.state.Get(y*11+x);
			if (piece > pieceColors.size())
				pieceColors.resize(piece+1);
			pieceColors[piece] = rgbColor::hsl((piece)/11.0, (piece%2)?1.0:0.5, 0.5);
		}
	}
	for (auto i : diagPieces)
		pieceColors[i] = rgbColor::hsl((diagPieces[0])/11.0, (diagPieces[0]%2)?1.0:0.5, 0.5);
	for (auto i : noFlipPieces)
		pieceColors[i] = rgbColor::hsl((noFlipPieces[0])/11.0, (noFlipPieces[0]%2)?1.0:0.5, 0.5);
	for (auto i : touchPieces)
		pieceColors[i] = rgbColor::hsl((touchPieces[0])/11.0, (touchPieces[0]%2)?1.0:0.5, 0.5);
	for (auto i : notTouchPieces)
		pieceColors[i] = rgbColor::hsl((notTouchPieces[0])/11.0, (notTouchPieces[0]%2)?1.0:0.5, 0.5);
}

void Hexagon::GetSuccessors(const HexagonState &nodeID, std::vector<HexagonState> &neighbors) const
{
	
}

void Hexagon::GetActions(const HexagonState &nodeID, std::vector<HexagonAction> &actions) const
{
	
}

HexagonAction Hexagon::GetAction(const HexagonState &s1, const HexagonState &s2) const
{
	HexagonAction a;
	return a;
}

void Hexagon::ApplyAction(HexagonState &s, HexagonAction a) const
{
	
}

void Hexagon::GetNextState(const HexagonState &, HexagonAction , HexagonState &) const
{
	
}

bool Hexagon::InvertAction(HexagonAction &a) const
{
	return true;
}


uint64_t Hexagon::GetStateHash(const HexagonState &node) const
{
	return 0;
}

uint64_t Hexagon::GetActionHash(HexagonAction act) const
{
	return 0;
}

/** Goal Test if the goal is stored **/
bool Hexagon::GoalTest(const HexagonState &node) const
{
	return true;
}

void Hexagon::RotateCW(HexagonState &s) const
{
	
}


bool Hexagon::GetBorder(int x, int y, int xoff, int yoff, Graphics::point &p1, Graphics::point &p2) const
{
	const float triangleWidth = 2.0/7;
	const float triangleHeight = triangleWidth/1.154700538379252; // 2/sqrt(3)
	const float xMargin = triangleWidth/2;
	const float yMargin = (2.0-triangleHeight*6)/2;
//	const float triangleHeight = 2.0/7;
//	const float triangleWidth = 1.154700538379252*triangleHeight; // 2/sqrt(3)
//	const float margin = triangleHeight/2;
	
	if (0==(x+y)%2) // points up
	{
		if (yoff < 0)
		{
			return false;
		}
		else if (xoff > 0)
		{
			p1 = {
				-1+xMargin+(x+2)*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
			p2 = {
				-1+xMargin+(x+1)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
		}
		else if (xoff < 0)
		{
			p1 = {
				-1+xMargin+x*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
			p2 = {
				-1+xMargin+(x+1)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
		}
		else if (yoff > 0)
		{
			p1 = {
				-1+xMargin+x*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
			p2 = {
				-1+xMargin+(x+2)*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
		}
		return true;
	}
	else { // points down
		if (yoff > 0)
		{
			return false;
		}
		else if (xoff > 0)
		{
			// upper right
			p1 = {
				-1+xMargin+(x+2)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
			// bottom tip
			p2 = {
				-1+xMargin+(x+1)*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
		}
		else if (xoff < 0)
		{
			// upper left
			p1 = {
				-1+xMargin+(x)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
			// bottom tip
			p2 = {
				-1+xMargin+(x+1)*triangleWidth/2,
				-1+yMargin+(y+1)*triangleHeight
			};
		}
		else if (yoff < 0)
		{
			// upper right
			p1 = {
				-1+xMargin+(x+2)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
			// upper left
			p2 = {
				-1+xMargin+(x)*triangleWidth/2,
				-1+yMargin+(y)*triangleHeight
			};
		}
		return true;
//		// upper right
//		p1 = {
//			-1+margin+(x+2)*triangleHeight/2,
//			-1+margin+(y)*triangleHeight
//		};
//		// upper left
//		p2 = {
//			-1+margin+(x)*triangleHeight/2,
//			-1+margin+(y)*triangleHeight
//		};
//		// bottom tip
//		p3 = {
//			-1+margin+(x+1)*triangleHeight/2,
//			-1+margin+(y+1)*triangleHeight
//		};
//		return false;
	}
}

void Hexagon::GetCorners(int x, int y, Graphics::point &p1, Graphics::point &p2, Graphics::point &p3) const
{
	const float triangleWidth = 2.0/7;
	const float triangleHeight = triangleWidth/1.154700538379252; // 2/sqrt(3)
	const float xMargin = triangleWidth/2;
	const float yMargin = (2.0-triangleHeight*6)/2;

	if (0==(x+y)%2) // points up
	{
		p1 = {
			-1+xMargin+x*triangleWidth/2,
			-1+yMargin+(y+1)*triangleHeight
		};
		p2 = {
			-1+xMargin+(x+2)*triangleWidth/2,
			-1+yMargin+(y+1)*triangleHeight
		};
		p3 = {
			-1+xMargin+(x+1)*triangleWidth/2,
			-1+yMargin+(y)*triangleHeight
		};
	}
	else { // points down
		p1 = {
			-1+xMargin+(x+2)*triangleWidth/2,
			-1+yMargin+(y)*triangleHeight
		};
		p2 = {
			-1+xMargin+(x)*triangleWidth/2,
			-1+yMargin+(y)*triangleHeight
		};
		p3 = {
			-1+xMargin+(x+1)*triangleWidth/2,
			-1+yMargin+(y+1)*triangleHeight
		};
	}
}

bool Hexagon::Valid(int x, int y) const
{
	static bool valid[6][11] =
	{
		{false, false, true, true, true, true, true, true, true, false, false},
		{false, true, true, true, true, true, true, true, true, true, false},
		{true, true, true, true, true, true, true, true, true, true, true},
		{true, true, true, true, true, true, true, true, true, true, true},
		{false, true, true, true, true, true, true, true, true, true, false},
		{false, false, true, true, true, true, true, true, true, false, false}
	};
	return valid[y][x];
}

void Hexagon::Draw(Graphics::Display &display) const
{
	// Draw board
	display.FillRect(Graphics::rect({0,0}, 1.0), Colors::white);
	for (int y = 0; y < 6; y++)
	{
		for (int x = 0; x < 11; x++)
		{
			if (!Valid(x, y))
				//if (!valid[y][x])
				continue;
			Graphics::point p1, p2, p3;
			GetCorners(x, y, p1, p2, p3);
			display.FillTriangle(p1, p2, p3, Colors::lightgray);
			display.FrameTriangle(p1, p2, p3, 0.01f, Colors::darkgray);
		}
	}
	float xLoc = -1, yLoc = -1;
	float dim = 0.1;
	
	if (noFlipPieces.size() != 0)
	{
		display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[noFlipPieces[0]]);
		display.DrawText("Cannot Flip", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
		xLoc = 0;
	}
	if (diagPieces.size() != 0)
	{
		display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[diagPieces[0]]);
		display.DrawText("Must Touch Diagonally", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
		xLoc = 0;
	}
	if (notTouchPieces.size() != 0)
	{
		display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[notTouchPieces[0]]);
		display.DrawText("Cannot Touch", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
		xLoc = 0;
	}
	if (touchPieces.size() != 0)
	{
		display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[touchPieces[0]]);
		display.DrawText("Must Touch Edges", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
		xLoc = 0;
	}
}

void Hexagon::DrawSetup(Graphics::Display &display) const
{
	// Draw board
	display.FillRect(Graphics::rect({0,0}, 1.0), Colors::white);

	// Horribly inefficient for now; just looking for correctness
	// Could all be pre-computed and cached
	for (int piece = 0; piece < 10; piece++)
	{
		bool found = false;
		Graphics::rect bounds;
		// get bounding box for piece
		for (int y = 0; y < 6; y++)
		{
			for (int x = 0; x < 11; x++)
			{
				if (!Valid(x, y))
					continue;
				int p = solution.state.Get(y*11+x);
				if (piece == p)
				{
					Graphics::point p1, p2, p3;
					GetCorners(x, y, p1, p2, p3);

					if (!found)
					{
						found = true;
						bounds = Graphics::rect(p1, p2);
					}
					else {
						bounds |= Graphics::rect(p1, p2);
					}
					bounds |= Graphics::rect(p2, p1);
					bounds |= Graphics::rect(p2, p3);
					bounds |= Graphics::rect(p3, p2);
					bounds |= Graphics::rect(p1, p3);
					bounds |= Graphics::rect(p3, p1);
				}
			}
		}

		float newWidth = 0.45f;
		float newHeight = 0.45f;
		float border = 0.01;
		float scale = 0.45f/std::max(bounds.right-bounds.left, bounds.bottom-bounds.top);
		float baseX = -0.9f+(piece%4)*(newWidth+border);
		float baseY = -0.7f+(piece>>2)*(newHeight+border);
//		std::cout << "Piece " << piece << " rect: " << bounds << "\n";
		// draw piece scaled
		for (int y = 0; y < 6; y++)
		{
			for (int x = 0; x < 11; x++)
			{
				if (!Valid(x, y))
					continue;
				int p = solution.state.Get(y*11+x);
				if (piece == p)
				{
					Graphics::point p1, p2, p3;
					GetCorners(x, y, p1, p2, p3);
					p1.x = baseX+(p1.x-bounds.left)*scale;
					p1.y = baseY+(p1.y-bounds.top)*scale;
					p2.x = baseX+(p2.x-bounds.left)*scale;
					p2.y = baseY+(p2.y-bounds.top)*scale;
					p3.x = baseX+(p3.x-bounds.left)*scale;
					p3.y = baseY+(p3.y-bounds.top)*scale;
					display.FillTriangle(p1, p2, p3, pieceColors[piece]);
				}
			}
		}
	}
	
	// Draw constraints at top
	{
		float xLoc = -1, yLoc = -1;
		float dim = 0.1;
		
		if (noFlipPieces.size() != 0)
		{
			display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[noFlipPieces[0]]);
			display.DrawText("Cannot Flip", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
			xLoc = 0;
		}
		if (diagPieces.size() != 0)
		{
			display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[diagPieces[0]]);
			display.DrawText("Must Touch Diagonally", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
			xLoc = 0;
		}
		if (notTouchPieces.size() != 0)
		{
			display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[notTouchPieces[0]]);
			display.DrawText("Cannot Touch", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
			xLoc = 0;
		}
		if (touchPieces.size() != 0)
		{
			display.FillRect({xLoc, yLoc, xLoc+dim, yLoc+dim}, pieceColors[touchPieces[0]]);
			display.DrawText("Must Touch Edges", {xLoc+1.2f*dim, yLoc+dim/2.0f}, Colors::black, dim*0.8f, Graphics::textAlignLeft, Graphics::textBaselineMiddle);
			xLoc = 0;
		}
	}
}

void Hexagon::Draw(Graphics::Display &display, const HexagonState &s) const
{
	// Draw objects
	for (int y = 0; y < 6; y++)
	{
		for (int x = 0; x < 11; x++)
		{
			if (!Valid(x, y))
				//if (!valid[y][x])
				continue;
			Graphics::point p1, p2, p3;
			GetCorners(x, y, p1, p2, p3);
			int piece = s.state.Get(y*11+x);
			if (piece < 10)
			{
				display.FillTriangle(p1, p2, p3, pieceColors[piece]);
//				// Note: 0 4 and 6 can't flip over; puzzles starting with "A" have the non-flip constraint
//				if (piece == 2 || piece == 3 || piece == 8) // 2 is ugly; 3 is green; 8 is purple
//					display.FillTriangle(p1, p2, p3, rgbColor::hsl((8)/11.0, (8%2)?1.0:0.5, 0.5));
//				else
//					display.FillTriangle(p1, p2, p3, rgbColor::hsl((piece)/11.0, (piece%2)?1.0:0.5, 0.5));
//				if (piece == 0 || piece == 4 || piece == 6) // 2 is ugly; 3 is green; 8 is purple
//					display.FrameTriangle(p1, p2, p3, 0.01, Colors::red);
			}
		}
	}
	// Draw Borders
	for (int y = 0; y < 6; y++)
	{
		for (int x = 0; x < 11; x++)
		{
			if (!Valid(x, y))
				//if (!valid[y][x])
				continue;
			Graphics::point p1, p2, p3;
			GetCorners(x, y, p1, p2, p3);
			int piece = s.state.Get(y*11+x);

			if ((x == 10) || (piece != s.state.Get(y*11+x+1)))
			{
				if (GetBorder(x, y, 1, 0, p1, p2))
				{
					display.DrawLine(p1, p2, 0.02, Colors::black);
				}
			}
			if ((x == 0) || (x > 0 && piece != s.state.Get(y*11+x-1)))
			{
				if (GetBorder(x, y, -1, 0, p1, p2))
				{
					display.DrawLine(p1, p2, 0.02, Colors::black);
				}
			}
			if ((y == 5) || (y < 5 && piece != s.state.Get((y+1)*11+x)))
			{
				if (GetBorder(x, y, 0, 1, p1, p2))
				{
					display.DrawLine(p1, p2, 0.02, Colors::black);
				}
			}
			if ((y == 0) || (y > 0 && piece != s.state.Get((y-1)*11+x)))
			{
				if (GetBorder(x, y, 0, -1, p1, p2))
				{
					display.DrawLine(p1, p2, 0.02, Colors::black);
				}
			}
		}
	}
}

