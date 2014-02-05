//
//  Maze.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 2/5/14.
//  Copyright (c) 2014 University of Denver. All rights reserved.
//

#ifndef MAP_GENERATOR_H
#define MAP_GENERATOR_H

#include "Map.h"

void MakeMaze(Map *map, int width = 1);
void MakeMaze(Map *map, int pathSize);
void MakeRandomMap(Map *map, int obstacles);
void MakePseudoMaze(Map *map, int pathSize);
void BuildRandomRoomMap(Map *map, int roomSize, int openingProbability=80);

#endif /* defined(MAP_GENERATOR_H) */
