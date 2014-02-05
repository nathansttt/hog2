//
//  MapGenerator.cpp
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/5/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#include "MapGenerators.h"


/**
 * MakeMaze(map, pathsize)
 *
 * A cheap function I hacked together to make psuedo mazes.
 * The only good values for pathSize are 1 and 3.
 */
void MakePseudoMaze(Map *map, int pathSize)
{
	int width = map->GetMapWidth();
	int height = map->GetMapHeight();
	map->SetRectHeight(0, 0, width-1, height-1, 0, kGround);
	int pathWidth = 1;
	for (int t = 0; (pathSize>>t); t++)
		pathWidth = t+1;
	for (int i = 0; i < 6*width*height; i++)
	{
		long x = (random()%width)&(~pathSize); // only path on even tiles
		long y = (random()%height)&(~pathSize);
		
		if (map->GetHeight(x, y) <= 1)
		{
			switch(random()%4)
			{
				case 0: // NORTH
					if ((x >= 2*pathWidth) && (map->GetHeight(x-2*pathWidth, y)+map->GetHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->SetHeight(x-2, y+t, map->GetHeight(x-2, y)+1);
							map->SetHeight(x-1, y+t, map->GetHeight(x-1, y)+1);
							map->SetHeight(x, y+t, map->GetHeight(x, y)+1);
						}
					}
					break;
				case 1: // SOUTH
					if ((x < width-2*pathWidth) && (map->GetHeight(x+2*pathWidth, y)+map->GetHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->SetHeight(x+2, y+t, map->GetHeight(x+2, y)+1);
							map->SetHeight(x+1, y+t, map->GetHeight(x+1, y)+1);
							map->SetHeight(x, y+t, map->GetHeight(x, y)+1);
						}
					}
					break;
				case 2: // WEST
					if ((y >= 2*pathWidth) && (map->GetHeight(x, y-2*pathWidth)+map->GetHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->SetHeight(x+t, y-2, map->GetHeight(x, y-2)+1);
							map->SetHeight(x+t, y-1, map->GetHeight(x, y-1)+1);
							map->SetHeight(x+t, y, map->GetHeight(x, y)+1);
						}
					}
					break;
				case 3: // EAST
					if ((y < height-2*pathWidth) && (map->GetHeight(x, y+2*pathWidth)+map->GetHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->SetHeight(x+t, y+2, map->GetHeight(x, y+2)+1);
							map->SetHeight(x+t, y+1, map->GetHeight(x, y+1)+1);
							map->SetHeight(x+t, y, map->GetHeight(x, y)+1);
						}
					}
					break;
			}
		}
	}
	for (int x = 0; x < width; x++)
		for (int y = 0; y < height; y++)
			if (map->GetHeight(x, y) > 0)
			{
				map->SetHeight(x, y, 0);
				map->SetTerrainType(x, y, kOutOfBounds);
			}
}

/**
 * MakeMaze(map)
 */
void MakeMaze(Map *map, int corridorWidth)
{
	int width = map->GetMapWidth();
	int height = map->GetMapHeight();
	int boxSize = corridorWidth+1;
	// fill the whole map empty
	map->SetRectHeight(0, 0, width-1, height-1, 0, kGround);
	// block all the walls
	for (int x = 0; x < width; x+=boxSize)
		map->SetRectHeight(x, 0, x, height-1, 0, kOutOfBounds);
	for (int y = 0; y < height; y+=boxSize)
		map->SetRectHeight(0, y, width-1, y, 0, kOutOfBounds);
	// mark univisited states
	for (int x = 0; x < width; x+=boxSize)
		for (int y = 0; y < height; y += boxSize)
			map->SetTerrainType(x+1, y+1, kOutOfBounds);
	std::vector<int> x;
	std::vector<int> y;
	
	x.push_back(width/(2*boxSize));
	y.push_back(height/(2*boxSize));
	//	x.push_back(0);
	//	y.push_back(0);
	//	map->SetHeight(0, 0, 1);
	map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
	
	while (x.size() > 0)
	{
		bool lastRandom = false;
		//		map->Print();
		int val;
		
		if (lastRandom || (1 != random()%10))
		{
			val = x.size()-1;
			if (1 == random()%30)
				lastRandom = false;
		}
		else {
			// bias towards end of array
			val = ((random()%(3*x.size()/2))+x.size()/2)%x.size();
			lastRandom = true;
		}
		
		//printf("Trying to extend (%d, %d)\n", x[val], y[val]);
		// raise up a corridor and open paths
		switch (random()%4)
		{
			case 0: // NORTH
			{
				if ((y[val] > 0) && (map->GetTerrainType(x[val]*boxSize+1, (y[val]-1)*boxSize+1) == kOutOfBounds))
				{
					for (int t = 0; t < boxSize-1; t++)
					{
						map->SetTerrainType(x[val]*boxSize+1+t, y[val]*boxSize, kGround);
					}
					x.push_back(x[val]);
					y.push_back(y[val]-1);
					// mark cell as unblocked
					map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
					//printf("Extended NORTH to (%d, %d) [size %d]\n", x.back(), y.back(), x.size());
					break;
				}
			}
			case 1: // SOUTH
			{
				if ((y[val] < (height+boxSize-1)/boxSize) && (map->GetTerrainType(x[val]*boxSize+1, (y[val]+1)*boxSize+1) == kOutOfBounds))
				{
					for (int t = 0; t < boxSize-1; t++)
					{
						map->SetTerrainType(x[val]*boxSize+1+t, (y[val]+1)*boxSize, kGround);
					}
					x.push_back(x[val]);
					y.push_back(y[val]+1);
					// mark cell as unblocked
					map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
					//printf("Extended SOUTH to (%d, %d) [size %d]\n", x.back(), y.back(), x.size());
					break;
				}
			}
			case 2: // EAST
			{
				if (((x[val]+1)*boxSize+1 < width) && (map->GetTerrainType((x[val]+1)*boxSize+1, y[val]*boxSize+1) == kOutOfBounds))
				{
					for (int t = 0; t < boxSize-1; t++)
					{
						map->SetTerrainType((x[val]+1)*boxSize, y[val]*boxSize+1+t, kGround);
					}
					x.push_back(x[val]+1);
					y.push_back(y[val]);
					// mark cell as unblocked
					map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
					//printf("Extended EAST to (%d, %d) [size %d]\n", x.back(), y.back(), x.size());
					break;
				}
			}
			case 3: // WEST
			{
				if ((x[val] > 0) && (map->GetTerrainType((x[val]-1)*boxSize+1, y[val]*boxSize+1) == kOutOfBounds))
				{
					for (int t = 0; t < boxSize-1; t++)
					{
						map->SetTerrainType(x[val]*boxSize, y[val]*boxSize+1+t, kGround);
					}
					x.push_back(x[val]-1);
					y.push_back(y[val]);
					// mark cell as unblocked
					map->SetTerrainType(x.back()*boxSize+1, y.back()*boxSize+1, kGround);
					//printf("Extended WEST to (%d, %d) [size %d]\n", x.back(), y.back(), x.size());
				}
				break;
			}
		}
		
		// check to see if node is blocked on all sides
		if (((x[val] == 0) || map->GetTerrainType((x[val]-1)*boxSize+1, y[val]*boxSize+1) == kGround) && // blocked left
			(((x[val]+1)*boxSize+1 >= width) || map->GetTerrainType((x[val]+1)*boxSize+1, y[val]*boxSize+1) == kGround) && // blocked right
			((y[val] == 0) || map->GetTerrainType(x[val]*boxSize+1, (y[val]-1)*boxSize+1) == kGround) && // blocked up
			(((y[val]+1)*boxSize+1 >= height) || map->GetTerrainType(x[val]*boxSize+1, (y[val]+1)*boxSize+1) == kGround)) // blocked down
		{
			//	printf("(%d, %d) now blocked: %d left\n", x[val], y[val], x.size()-1);
			x[val] = x.back();
			y[val] = y.back();
			x.pop_back();
			y.pop_back();
		}
	}
}

void BuildRandomRoomMap(Map *map, int roomSize, int openingProbability)
{
    int width = map->GetMapWidth();
    int height = map->GetMapHeight();
	
	map->SetTerrainType(0, 0, width-1, height-1, kGround);
    for (int x = 0; x < height; x += roomSize)
    {
        // draw a horizontal line
        map->SetTerrainType(0, x, width-1, x, kOutOfBounds);
        // then punch a bunch of holes in it
        for (int y = 0; y < width; y += roomSize)
		{
			if ((random()%100) < openingProbability) // chance of creating hole
			{
				for (int z = 0; z < roomSize/8; z++)
				{
					map->SetTerrainType(y+1+random()%(roomSize-1), x, kGround);
				}
			}
		}
    }
    for (int x = 0; x < width; x += roomSize)
    {
        // draw a vertical line
        map->SetTerrainType(x, 0, x, height-1, kOutOfBounds);
        // then punch a bunch of holes in it
        for (int y = 0; y < height; y += roomSize)
		{
			if ((random()%100) < openingProbability) // chance of creating hole
			{
				for (int z = 0; z < roomSize/8; z++)
				{
					map->SetTerrainType(x, y+1+random()%(roomSize-1), kGround);
				}
			}
		}
    }
}

void MakeRandomMap(Map *map, int obstacles)
{
	int total = map->GetMapWidth()*map->GetMapHeight()*obstacles/100;
	for (int x = 0; x < total; x++)
	{
		int xloc, yloc;
		while (1)
		{
			xloc = random()%map->GetMapWidth();
			yloc = random()%map->GetMapHeight();
			if (map->GetTerrainType(xloc, yloc) == kGround)
				break;
		}
		map->SetTerrainType(xloc, yloc, kOutOfBounds);
	}
}
