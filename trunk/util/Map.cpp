/*
 * $Id: map.cpp,v 1.28 2007/03/07 22:01:36 nathanst Exp $
 *
 * This file is part of HOG.
 *
 * HOG is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * HOG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with HOG; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/ 

#include <stack>
#include "Map.h"
#include "GLUtil.h"

using namespace std;

static const bool verbose = false; 

/** 
* Construct a half tile, initializing to flat values.
*
* A halfTile is the lowest level in the map - it contains the height of the
* three relevant corners. The heights are initialized to 0, the type of
* territory is initialized to ground.
*/		
halfTile::halfTile()
{
	corners[0] = corners[1] = corners[2] = 0;
	type = kGround;
	node = -1;
}

/** 
* Construct a tile with no split.
*
* A Tile contains two half tiles and information about how the tile is
* split. If a tile isn't split, the first three parts of the tile are in
* the first halfTile and the fourth is in th second.
*/
Tile::Tile()
:tile1(), tile2(), split(kNoSplit)
{ }

/** 
* Create a new map of a particular size.
*
* A map is an array of tiles according to the height and width of the map.
*/
Map::Map(long _width, long _height)
:width(_width), height(_height)
{
	mapType = kRaw;
	tileSet = kFall;
	map_name[0] = 0;
	sizeMultiplier = 1;
	land = new Tile *[width];
	//	for (int x = 0; x < 8; x++)
	//		g[x] = 0;
	for (int x = 0; x < width; x++) land[x] = new Tile [height];
	drawLand = true;
	dList = 0;
	updated = true;
	revision = 0;
	//	numAbstractions = 1;
	//	pathgraph = 0;
}

/** 
* Create a new map by copying it from another map.
*
* Creates a new map and initializes it with the map passed to it.
*/
Map::Map(Map *m)
{
	mapType = m->mapType;
	tileSet = m->tileSet;
	strncpy(map_name, m->map_name, 128);
	sizeMultiplier = m->sizeMultiplier;
	width = m->width;
	height = m->height;
	
	land = new Tile *[width];
	for (int x = 0; x < width; x++) land[x] = new Tile [height];
	
	drawLand = m->drawLand;
	dList = 0;
	updated = true;
	revision = m->revision;
	
	for (int x = 0; x < width; x++)
		for (int y = 0; y < height; y++)
			land[x][y] = m->land[x][y];
}

/** 
* Create a new map by loading it from a file.
*
* Creates a new map and initializes it with the file passed to it.
*/
Map::Map(const char *filename)
{
	sizeMultiplier = 1;
	land = 0;
	load(filename);
	tileSet = kFall;
}

/** 
* Create a new map by loading it from a file pointer.
*
* Creates a new map and initializes it with the file pointer passed to it.
*/
Map::Map(FILE *f)
{
	sizeMultiplier = 1;
	map_name[0] = 0;
	land = 0;
	load(f);
	tileSet = kFall;
}

/** 
* Not implemented.
*
* This function is not implemented.
*/
Map::Map(std::istringstream &/*data*/)
{
	sizeMultiplier = 1;
	dList = 0;
	tileSet = kFall;
}

Map::~Map()
{
	for (int x = 0; x < width; x++)
		delete [] land[x];
	delete [] land;
}

void Map::scale(long newWidth, long newHeight)
{
	Tile **newLand;
	newLand = new Tile *[newWidth];
	for (int x = 0; x < newWidth; x++)
	{
		newLand[x] = new Tile [newHeight];
		for (int y = 0; y < newHeight; y++)
		{
			newLand[x][y] = land[(x*width)/newWidth][(y*height)/newHeight];
			
			if (((y*height)/newHeight > 0) && (adjacentEdges((x*width)/newWidth, (y*height)/newHeight, kTopEdge)))
			{ // make sure top edge is still adjacent
				newLand[x][y].tile1.corners[0] = newLand[x][y-1].tile1.corners[1];
				// this won't work with splits...
				newLand[x][y].tile2.corners[0] = newLand[x][y-1].tile1.corners[2];
			}
			if (((x*width)/newWidth > 0) && (adjacentEdges((x*width)/newWidth, (y*height)/newHeight, kLeftEdge)))
			{ // make sure left edge is still adjacent
				newLand[x][y].tile1.corners[0] = newLand[x-1][y].tile2.corners[0];
				// this won't work with splits...
				newLand[x][y].tile1.corners[1] = newLand[x-1][y].tile1.corners[2];
			}
		}
	}
	for (int x = 0; x < width; x++) delete [] land[x];
	delete [] land;
	land = newLand;
	width = newWidth;
	height = newHeight;
	revision++;
	updated = true;
	map_name[0] = 0;
}


/** 
* Resets the current map by loading the file passed in.
*
* Resets the current map by loading the file passed in.
*/
void Map::load(const char *filename)
{
	if (land)
	{
		for (int x = 0; x < width; x++)
			delete [] land[x];
		delete [] land;
		land = 0;
	}
	revision++;
	FILE *f = fopen(filename, "r");
	if (f)
	{
		load(f);
		fclose(f);
		strncpy(map_name, filename, 128);
	}
	else {
		printf("Error! Can't open file %s\n", filename);
		width = height = 64;
		land = new Tile *[width];
		//	for (int x = 0; x < 8; x++)
		//		g[x] = 0;
		for (int x = 0; x < width; x++)
			land[x] = new Tile [height];
		drawLand = true;
		dList = 0;
		updated = true;
		map_name[0] = 0;
	}
}

/** 
* Resets the current map by loading the file from the pointer passed in.
*/
void Map::load(FILE *f)
{
	if (land)
	{
		for (int x = 0; x < width; x++)
			delete [] land[x];
		delete [] land;
		land = 0;
	}
	
	char format[32];
	// ADD ERROR HANDLING HERE
	int num = fscanf(f, "type %s\nheight %d\nwidth %d\nmap\n", format, &height, &width);
	// printf("got height %d, width %d\n", height, width);
	if (num == 3)
	{
		if (strcmp(format, "octile") == 0)
			loadOctile(f, height, width);
		else if (strcmp(format, "octile-corner") == 0)
			loadOctileCorner(f, height, width);
		else if (strcmp(format, "raw") == 0)
			loadRaw(f, height, width);
	}
	else if (!tryLoadRollingStone(f))
	{
		printf("Unknown map type; aborting load!\n");
		width = height = 64;
		land = new Tile *[width];
		//	for (int x = 0; x < 8; x++)
		//		g[x] = 0;
		for (int x = 0; x < width; x++) land[x] = new Tile [height];
		drawLand = true;
		dList = 0;
		updated = true;
		map_name[0] = 0;
	}
}

void Map::loadRaw(FILE *f, int high, int wide)
{
	mapType = kRaw;
	height = high;
	width = wide;
	land = new Tile *[wide];
	for (int x = 0; x < wide; x++)
		land[x] = new Tile [high];
	drawLand = true;
	dList = 0;
	updated = true;
	for (int x = 0; x < wide; x++)
	{
		fread(&land[x][0], sizeof(Tile), wide, f);
		for (int y = 0; y < high; y++)
		{
			land[x][y].tile1.node = kNoGraphNode;
			land[x][y].tile2.node = kNoGraphNode;
		}
	}
}

void Map::loadOctile(FILE *f, int high, int wide)
{
	mapType = kOctile;
	height = high*sizeMultiplier;
	width = wide*sizeMultiplier;
	land = new Tile *[wide*sizeMultiplier];
	for (int x = 0; x < wide*sizeMultiplier; x++) land[x] = new Tile [high*sizeMultiplier];
	drawLand = true;
	dList = 0;
	updated = true;
	for (int y = 0; y < high; y++)
	{
		for (int x = 0; x < wide; x++)
		{
			char what;
			fscanf(f, "%c", &what);
			switch (toupper(what))
			{
				case '@':
				case 'O':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							setTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kOutOfBounds); break;
				case 'S':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							setTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kSwamp); break;
				case 'W':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							setTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kWater); break;
				case 'T':
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							setTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kTrees); break;
				default:
					for (int r = 0; r < sizeMultiplier; r++)
						for (int s = 0; s < sizeMultiplier; s++)
							setTerrainType(x*sizeMultiplier+r, y*sizeMultiplier+s,
														 kGround); break;
			}
			for (int r = 0; r < sizeMultiplier; r++)
				for (int s = 0; s < sizeMultiplier; s++)
				{
					land[x*sizeMultiplier+r][y*sizeMultiplier+s].tile1.node = kNoGraphNode;
					land[x*sizeMultiplier+r][y*sizeMultiplier+s].tile2.node = kNoGraphNode;
				}
		}
			fscanf(f, "\n");
	}
}

void Map::loadOctileCorner(FILE *f, int high, int wide)
{
	mapType = kOctileCorner;
	width--;
	height--;
	land = new Tile *[width];
	for (int x = 0; x < width; x++) land[x] = new Tile [height];
	drawLand = true;
	dList = 0;
	updated = true;
	for (int y = 0; y < high; y++)
	{
		for (int x = 0; x < wide; x++)
		{
			int h;
			char hc;
			fscanf(f, "%c", &hc);
			h = hc-'0'-2;
			if ((x > 0) && (y < high-1))
				setCornerHeight(x-1, y, kTopRight, h);
			if ((x < wide-1) && (y > 0))
				setCornerHeight(x, y-1, kBottomLeft, h);
			if ((x > 0) && (y > 0))
				setCornerHeight(x-1, y-1, kBottomRight, h);
			if ((x < wide-1) && (y < high-1))
				setCornerHeight(x, y, kTopLeft, h);
		}
		fscanf(f, "\n");
	}
	for (int y = 0; y < high; y++)
	{
		for (int x = 0; x < wide; x++)
		{
			char what;
			fscanf(f, "%c", &what);
			if ((y == high-1) || (x == wide-1))
				continue;
			if (!islower(what))
				setHeight(x, y, getCornerHeight(x, y, kTopLeft));
			switch (toupper(what))
			{
				case 'O':
					setTerrainType(x, y, kOutOfBounds); break;
				case 'S':
					setTerrainType(x, y, kSwamp); break;
				case 'W':
					setTerrainType(x, y, kWater); break;
				case 'T':
					setTerrainType(x, y, kTrees); break;
				default:
					setTerrainType(x, y, kGround); break;
			}
			land[x][y].tile1.node = kNoGraphNode;
			land[x][y].tile2.node = kNoGraphNode;
		}
		fscanf(f, "\n");
	}
}

// check to see if we can load it as a rolling stone puzzle
bool Map::tryLoadRollingStone(FILE *f)
{
	// preprocess - all chars must be ' ', '\n', '*', '$', '#', '@' or '.'
	// also calculate map dimensions
	rewind(f);
	int nrows = 0;
	int ncols = 0;
	while (1) // get columns
	{
		int currCol = 1;
		char nextc = fgetc(f);
		if (feof(f))
			break;
		if (!isLegalStone(nextc))
		{
			printf("illegal stone on row %d, (column pos %d)\n", nrows, currCol);
			return false;
		}
		nrows++;
		while ((nextc = fgetc(f)) != '\n') // get this row
		{
			if (feof(f))
			{
				break;
			}
			if (!isLegalStone(nextc))
			{
				printf("illegal stone on row %d, column pos %d\n", nrows, currCol);
				return false;
			}
			currCol++;
		}
		ncols = max(currCol, ncols);
	}
	mapType = kSokoban;
	printf("Dimensions (%d, %d)\n", ncols, nrows);
	width = ncols;
	height = nrows;
	
	rewind(f);
	int manx=0, many=0;
	land = new Tile *[width];
	for (int x = 0; x < width; x++) land[x] = new Tile [width];
	drawLand = true;
	dList = 0;
	updated = true;
	for (int y = 0; y < height; y++)
	{
		bool foundEOL = false;
		for (int x = 0; x < width; x++)
		{
			char what;
			what = fgetc(f);
			//fscanf(f, "%c", &what);
			switch (toupper(what))
			{
				case ' ':
					setTerrainType(x, y, kOutOfBounds); printf(" "); break;
				case '#':
					setTerrainType(x, y, kWater); printf("#"); break;
				case '$':
					setTerrainType(x, y, kSwamp); printf("$"); break;
				case '*':
					setTerrainType(x, y, kBlight); printf("*"); break;
				case '@':
					manx = x; many = y;
					setTerrainType(x, y, kGrass); printf("@"); break;
				case '.':
					setTerrainType(x, y, kBlight); printf("."); break;
				case '\n':
					foundEOL = true;
					for (; x < width; x++)
					{
						setTerrainType(x, y, kOutOfBounds);
						printf("_"); 
						land[x][y].tile1.node = kNoGraphNode;
						land[x][y].tile2.node = kNoGraphNode;
					}
						x--;
					break;
			}
			land[x][y].tile1.node = kNoGraphNode;
			land[x][y].tile2.node = kNoGraphNode;
		}
		if (!foundEOL)
			fgetc(f);
		printf("\n");
	}
	// from man location, paint inside of room
	paintRoomInside(manx, many);
	for (int x = 0; x < width; x++)
	{
		for (int y = 0; y < height; y++)
		{
			land[x][y].tile1.node = kNoGraphNode;
			if (getTerrainType(x, y) == kWater)
				setTerrainType(x, y, kOutOfBounds);
			setHeight(x,y,0);
			setSplit(x,y,kNoSplit);
		}
	}
	return true;
}

void Map::paintRoomInside(int x, int y)
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
		return;
	if (land[x][y].tile1.node != kNoGraphNode)
		return;
	if (getTerrainType(x, y) == kWater)
		return;
	if (getTerrainType(x, y) == kOutOfBounds)
		setTerrainType(x, y, kGround);
	land[x][y].tile1.node = kNoGraphNode+1;
	paintRoomInside(x+1, y);
	paintRoomInside(x-1, y);
	paintRoomInside(x, y+1);
	paintRoomInside(x, y-1);
}

bool Map::isLegalStone(char c)
{
	const int numLegals = 7;
	char legals[numLegals] = {' ', '\n', '$', '#', '@', '.', '*'};
	for (int x = 0; x < numLegals; x++)
		if (c == legals[x])
			return true;
	printf("Found illegal stone: \'%c\'\n", c);
	return false;
}

/** 
* unimplemented.
*/
void Map::save(std::stringstream &/*data*/) {}

/** 
* Saves the current map out to the designated file.
*
* Saves the current map out to the designated file.
*/
void Map::save(const char *filename)
{
	FILE *f = fopen(filename, "w+");
	if (f)
	{
		save(f);
		fclose(f);
	}
	else {
		printf("Error! Couldn't open file to save\n");
	}
}

/** 
* Saves the current map out to the designated file.
*
* Saves the current map out to the designated file.
*/
void Map::save(FILE *f)
{
	switch(mapType)
	{
		case kOctile:
			saveOctile(f);
			break;
		case kSokoban:
		case kOctileCorner:
			printf("Unable to save to identical map type\n");
		case kRaw:
		default:
			saveRaw(f);
			break;
	}
}

void Map::saveOctile(FILE *f)
{
	if (f)
	{
		fprintf(f, "type octile\nheight %d\nwidth %d\nmap\n", height, width);
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				switch (getTerrainType(x, y))
				{
					case kGround: fprintf(f, "."); break;
					case kSwamp: fprintf(f, "S"); break;
					case kWater: fprintf(f, "W"); break;
					case kTrees: fprintf(f, "T"); break;
					default: fprintf(f, "@"); break; // out of bounds
				}
			}
			fprintf(f, "\n");
		}
	}
}

void Map::saveRaw(FILE *f)
{
	if (f)
	{
		fprintf(f, "type raw\nheight %d\nwidth %d\nmap\n", height, width);
		for (int x = 0; x < width; x++)
		{
			fwrite(&land[x][0], sizeof(Tile), width, f);
		}
	}
}


/** 
* Do an ASCII/ANSI print out of the map.
*
* Moves the cursor to the top of the screen and draws an ASCII version of
* the map. The map is shrunk by a factor of 2 in the vertical scale, but
* left normal in the horizontal scale. For the moment we leave ground blank
* and draw the walls as x/X/^. No other ground type is drawn.
*/
void Map::print(int _scale)
{
	//  printf("%c[%d;%dmHeight\n", 27, 4, 30);
	//  printf("%c[%d;%dm", 27, 0, 0);
	//  for (int x = 0; x < 5; x++)
	//    printf("%c[%d;%dm%d\n", 27, 1, 31+x, x);
	//  printf("\n");
	// clear the screen
	printf("%c[H", 27);
	for (int y = 0; y < height; y+=2*_scale)
	{
		for (int x = 0; x < width; x+=_scale)
		{
			//      if (land[x][y].split == kForwardSplit) {
			//				printf("%c[%d;%dm", 27, 0, 0);
			//				printf("/");
			//      } else if (land[x][y].split == kBackwardSplit) {
			//				printf("%c[%d;%dm", 27, 0, 0);
			//				printf("\\");
			//      } else {
			//				printf("%c[%d;%ldm", 27, 1, 31+land[x][y].tile1.corners[0]); }
			switch (land[x][y].tile1.type) {
				case kOutOfBounds:
					if (y+1 >= height)
					{ printf("^"); break; }
					switch (land[x][y+1].tile1.type) {
						case kOutOfBounds: printf("X"); break;
						case kGround: case kSwamp: printf("^"); break;
						default: printf("#"); break;
					}
					break;
					//					case kUndefined: printf("?"); break;
					//					case kWater: printf("%%"); break;
					//					case kSwamp: printf("@"); break;
				case kGround:
					if (y+1 >= height)
					{ printf(" "); break; }
					switch (land[x][y+1].tile1.type) {
						case kOutOfBounds: printf("x"); break;
						case kGround: printf(" "); break;
						default: printf("#"); break;
					}
					break;
				case kSwamp:
					if (y+1 >= height)
					{ printf(","); break; }
					switch (land[x][y+1].tile1.type) {
						case kOutOfBounds: printf("x"); break;
						case kGround: printf("."); break;
						case kSwamp: printf(":"); break;
						default: printf("#"); break;
					}
					break;
				default:
					printf("?");
					break;
			}
		}
		printf("\n");
	}
	printf("%c[%d;%dm", 27, 0, 0);
	printf("\n");
}

const char *Map::getMapName()
{
	if (map_name[0] == 0)
		return "";
	return map_name;
}

/** 
* Return the tile at location x, y.
*
* returns a reference to the type at a particular x/y location. (starting from 0)
*/

Tile &Map::getTile(long x, long y)
{
	return land[x][y];
}

/** 
* Return the split of the tile at x, y.
*
* Returns the type of split; either kNoSplit, kForwardSplit, kBackwardSplit
*/
tSplit Map::getSplit(long x, long y) const
{
	return land[x][y].split;
}

/** 
* Set the split of the tile at x, y.
*
* Sets how a map is split; either kNoSplit, kForwardSplit, kBackwardSplit
*/
void Map::setSplit(long x, long y, tSplit split)
{
	revision++;
	land[x][y].split = split;
}


/** 
* Get the terrain type of the (split) tile at x, y.
*
* Gets the terrain type for this tile. By default it looks for the value
* of the whole tile. Possible split values are kWholeTile, kLeftSide, and
* kRightSide. 
*/
long Map::getTerrainType(long x, long y, tSplitSide split) const
{
	if ((x < 0) || (x >= width) || (y < 0) || (y >= height)) return kUndefined;
	if (split == kRightSide) return land[x][y].tile2.type;
	return land[x][y].tile1.type;
}

/**
* Get the terrain type for one side of the tile at x, y.
 *
 * Gets the terrain type for a particular edge of the type. 
 * (kLeftEdge, kRightEdge, kTopEdge, kBottom Edge)
 * This function avoids making you figure out all the ways a tile could
 * be split to get the correct value out.
 */
long Map::getTerrainType(long x, long y, tEdge side) const
{
	if ((x < 0) || (x >= width) || (y < 0) || (y >= height)) return kUndefined;
	if (land[x][y].split == kNoSplit) return land[x][y].tile1.type;
	switch (side) {
		case kLeftEdge:
			return land[x][y].tile1.type;
		case kRightEdge:
			return land[x][y].tile2.type;
		case kTopEdge:
			if (land[x][y].split == kForwardSplit)
				return land[x][y].tile1.type;
			else
				return land[x][y].tile2.type;
		case kBottomEdge:
			if (land[x][y].split == kBackwardSplit)
				return land[x][y].tile1.type;
			else
				return land[x][y].tile2.type;
		default:
			return kUndefined;
	}
}

/** Set the terrain type of the side of the tile at x, y.
*
* side is one of kWholeTile, kLeftSide or kRightSide
* If tile is not split and you specify a split side, nothing happens
* If tile is split and you specify kWholeTile, the split remains,
* and the terrain is applied to both sides.
*/
void Map::setTerrainType(long x, long y, tTerrain type, tSplitSide split)
{
	revision++;
	updated = true;
	map_name[0] = 0;
	if ((land[x][y].split == kNoSplit) && (split != kWholeTile)) return;
	
	switch (split) {
		
		case kWholeTile:
			land[x][y].tile1.type = type;
			land[x][y].tile2.type = type;
			break;
			
		case kLeftSide:
			land[x][y].tile1.type = type;
			break;
			
		case kRightSide:
			land[x][y].tile2.type = type;
			break;
	}
}

/** 
* Get the (flat) height of the tile at x, y.
*
* returns the height of a particular tile -- actually just one corner of the
* tile. If the tile is sloping you'll get back kUndefined and need to get
* the specific corner heights.
* returns kUndefinedHeight if the tile is split and you specify
* the whole tile
*/
long Map::getHeight(long x, long y, tSplitSide split)
{
	if ((land[x][y].split != kNoSplit) && (split == kWholeTile)) return kUndefinedHeight;
	
	switch (split) {
		
		case kWholeTile:
			
		case kLeftSide:
			if ((land[x][y].tile1.corners[0] == land[x][y].tile1.corners[1]) &&
					(land[x][y].tile1.corners[1] == land[x][y].tile1.corners[2]))
				return land[x][y].tile1.corners[0];
			return kUndefinedHeight;
			break;
			
		case kRightSide:
			if ((land[x][y].tile2.corners[0] == land[x][y].tile2.corners[1]) &&
					(land[x][y].tile2.corners[1] == land[x][y].tile2.corners[2]))
				return land[x][y].tile2.corners[0];
			break;
	}
	return kUndefinedHeight;
}

/** 
* Set the (flat) height of the tile at x, y.
* 
* Split is kWholeTile, kLeftSide or kRightSide.
*/
void Map::setHeight(long x, long y, long tHeight, tSplitSide split)
{
	revision++;
	switch (split) {
		
		case kWholeTile:
			land[x][y].tile1.corners[0] = land[x][y].tile1.corners[1] =
			land[x][y].tile1.corners[2] = tHeight;
			land[x][y].tile2.corners[0] = tHeight;
			land[x][y].tile2.corners[1] = tHeight;
			land[x][y].tile2.corners[2] = tHeight;
			break;
		case kLeftSide:
			land[x][y].tile1.corners[0] = land[x][y].tile1.corners[1] =
			land[x][y].tile1.corners[2] = tHeight;
			break;
		case kRightSide:
			land[x][y].tile2.corners[0] = land[x][y].tile2.corners[1] =
			land[x][y].tile2.corners[2] = tHeight;
			break;
	}
}

/**
* Set the height of any one corner of a tile.
 * 
 * corner is kTopLeft, kBottomLeft, kTopRight or kBottomRight
 * edge is kBottomEdge, kLeftEdge, kRightEdge, kTopEdge
 * returns kUndefinedHeight if the split is inconsistant with the tile type
 * The combination of a corner and an edge uniquely define a single height
 */
long Map::getCornerHeight(long x, long y, tCorner which, tEdge edge) const
{
	if (getSplit(x, y) == kNoSplit)
	{
		switch (which) {
			case kTopLeft: return land[x][y].tile1.corners[0];
			case kBottomLeft: return land[x][y].tile1.corners[1];
			case kTopRight: return land[x][y].tile2.corners[0];
			case kBottomRight: return land[x][y].tile1.corners[2];
			default: break;
		}
	}
	else {
		if (edge == kLeftEdge)
		{
			switch (which) {
				case kTopLeft: return land[x][y].tile1.corners[0];
				case kBottomLeft: return land[x][y].tile1.corners[1];
				default: break;
			}
		}
		else if (edge == kRightEdge)
		{
			switch (which) {
				case kTopRight: return land[x][y].tile2.corners[0];
				case kBottomRight: return land[x][y].tile2.corners[1];
				default: break;
			}
		}
		else if ((edge == kTopEdge) && (getSplit(x, y) == kForwardSplit))
		{
			switch (which) {
				case kTopLeft: return land[x][y].tile1.corners[0];
				case kTopRight: return land[x][y].tile1.corners[2];
				default: break;
			}
		}
		else if ((edge == kTopEdge) && (getSplit(x, y) == kBackwardSplit))
		{
			switch (which) {
				case kTopLeft: return land[x][y].tile2.corners[2];
				case kTopRight: return land[x][y].tile2.corners[0];
				default: break;
			}
		}
		else if ((edge == kBottomEdge) && (getSplit(x, y) == kForwardSplit))
		{
			switch (which) {
				case kBottomLeft: return land[x][y].tile2.corners[2];
				case kBottomRight: return land[x][y].tile2.corners[1];
				default: break;
			}
		}
		else if ((edge == kBottomEdge) && (getSplit(x, y) == kBackwardSplit))
		{
			switch (which) {
				case kBottomLeft: return land[x][y].tile1.corners[1];
				case kBottomRight: return land[x][y].tile1.corners[2];
				default: break;
			}
		}
	}
	// should never get here...
	return kUndefinedHeight;
}

/**
* Get the height of any one corner of a tile.
 * 
 * corner is kTopLeft, kBottomLeft, kTopRight or kBottomRight
 * split is kLeftSide, kRightSide or kWholeTile
 * returns kUndefinedHeight if the split is inconsistant with the tile type
 * The combination of a corner and a split side uniquely define a single height
 */
long Map::getCornerHeight(long x, long y, tCorner which, tSplitSide split) const
{
	if ((land[x][y].split != kNoSplit) && (split == kWholeTile))
		return kUndefinedHeight;
	if (split == kWholeTile)
	{
		switch (which) {
			case kTopLeft: return land[x][y].tile1.corners[0];
			case kBottomLeft: return land[x][y].tile1.corners[1];
			case kTopRight: return land[x][y].tile2.corners[0];
			case kBottomRight: return land[x][y].tile1.corners[2];
			default: break;
		}
	}
	else if (split == kLeftSide)
	{
		switch (which) {
			case kTopLeft: return land[x][y].tile1.corners[0];
			case kBottomLeft: return land[x][y].tile1.corners[1];
			case kTopRight:
				if (land[x][y].split == kForwardSplit)
					return land[x][y].tile1.corners[2];
				return kUndefinedHeight;
			case kBottomRight:
				if (land[x][y].split == kBackwardSplit)
					return land[x][y].tile1.corners[2];
				return kUndefinedHeight;
			default: break;
		}
	}
	else if (split == kRightSide)
	{
		switch (which) {
			case kTopRight: return land[x][y].tile2.corners[0];
			case kBottomRight: return land[x][y].tile2.corners[1];
			case kTopLeft:
				if (land[x][y].split == kBackwardSplit)
					return land[x][y].tile2.corners[2];
				return kUndefinedHeight;
			case kBottomLeft:
				if (land[x][y].split == kForwardSplit)
					return land[x][y].tile2.corners[2];
				return kUndefinedHeight;
			default: break;
		}
	}
	return kUndefinedHeight;
}

/**
* Set the height of any one corner of a tile.
 * 
 * corner is kTopLeft, kBottomLeft, kTopRight or kBottomRight
 * split is kLeftSide, kRightSide or kWholeTile
 * The combination of a corner and a split side uniquely define a single height,
 * which is returned.
 */
void Map::setCornerHeight(long x, long y, tCorner which,
													long cHeight, tSplitSide split)
{
	if ((land[x][y].split != kNoSplit) && (split == kWholeTile))
		return;
	revision++;
	if (split == kWholeTile)
	{
		switch (which) {
			case kTopLeft: land[x][y].tile1.corners[0] = cHeight; break;
			case kBottomLeft: land[x][y].tile1.corners[1] = cHeight; break;
			case kTopRight: land[x][y].tile2.corners[0] = cHeight; break;
			case kBottomRight: land[x][y].tile1.corners[2] = cHeight; 
				land[x][y].tile2.corners[1] = cHeight;break;
			default: break;
		}
	}
	else if (split == kLeftSide)
	{
		switch (which) {
			case kTopLeft: land[x][y].tile1.corners[0] = cHeight; break;
			case kBottomLeft: land[x][y].tile1.corners[1] = cHeight; break;
			case kTopRight:
				if (land[x][y].split == kForwardSplit)
					land[x][y].tile1.corners[2] = cHeight;
				break;
			case kBottomRight:
				if (land[x][y].split == kBackwardSplit)
					land[x][y].tile1.corners[2] = cHeight;
				break;
			default: break;
		}
	}
	else if (split == kRightSide)
	{
		switch (which) {
			case kTopRight: land[x][y].tile2.corners[0] = cHeight; break;
			case kBottomRight: land[x][y].tile2.corners[1] = cHeight; break;
			case kTopLeft:
				if (land[x][y].split == kBackwardSplit)
					land[x][y].tile2.corners[2] = cHeight;
				break;
			case kBottomLeft:
				if (land[x][y].split == kForwardSplit)
					land[x][y].tile2.corners[2] = cHeight;
				break;
			default: break;
		}
	}
}


/**
* Places a rectangle into the map, but also modifies the edges to make the
 * transition smooth.
 *
 * sets a rectangle of with corner coordinates (x1, y1) (x2, y2)
 * but also takes the 1-radius tiles surrounding that rectangle and
 * smooths them so you get a nice fit of land together.
 */
void Map::smoothSetRectHeight(long x1, long y1, long x2, long y2, long h, tTerrain type)
{
	updated = true;
	map_name[0] = 0;
	if (x1 > x2)
	{
		smoothSetRectHeight(x2, y1, x1, y2, h, type);
		return;
	}
	else if (y1 > y2)
	{
		smoothSetRectHeight(x1, y2, x2, y1, h, type);
		return;
	}
	printf("Doing smooth rect between (%ld, %ld) and (%ld, %ld) height %ld\n", x1, y1, x2, y2, h);
	setRectHeight(x1, y1, x2, y2, h, type);
	
	// top side
	for (int x = x1; x <= x2; x++)
	{
		setTerrainType(x, y1-1, type);
		switch(getSplit(x, y1-1)) {
			case kNoSplit:
				if (getCornerHeight(x, y1-1, kTopLeft) != getCornerHeight(x, y1-1, kTopRight))
				{ // need to split slanted tile
					setSplit(x, y1-1, kForwardSplit); // split is arbitarary?
																						//setCornerHeight(x, y1-1, kBottomLeft, getCornerHeight(x, y1-1, kBottomLeft, kLeftSide), kRightSide);
					
					setCornerHeight(x, y1-1, kBottomLeft, h, kRightSide);
					setCornerHeight(x, y1-1, kBottomRight, h, kRightSide);
					setCornerHeight(x, y1-1, kBottomLeft, h, kLeftSide);
				}
				else {
					setCornerHeight(x, y1-1, kBottomLeft, h);
					setCornerHeight(x, y1-1, kBottomRight, h);
				}
				break;
			case kForwardSplit:
				setCornerHeight(x, y1-1, kBottomLeft, h, kRightSide);
				setCornerHeight(x, y1-1, kBottomRight, h, kRightSide);
				setCornerHeight(x, y1-1, kBottomLeft, h, kLeftSide);
				break;
			case kBackwardSplit:
				setCornerHeight(x, y1-1, kBottomLeft, h, kLeftSide);
				setCornerHeight(x, y1-1, kBottomRight, h, kLeftSide);
				setCornerHeight(x, y1-1, kBottomRight, h, kRightSide);
				break;
		}
	}
	
	// bottom side
	for (int x = x1; x <= x2; x++)
	{
		setTerrainType(x, y2+1, type);
		switch(getSplit(x, y2+1)) {
			case kNoSplit:
				if (getCornerHeight(x, y2+1, kBottomLeft) != getCornerHeight(x, y2+1, kBottomRight))
				{ // need to split slanted tile
					setSplit(x, y2+1, kBackwardSplit); // split is arbitarary?
																						 //setCornerHeight(x, y2+1, kTopLeft, getCornerHeight(x, y2+1, kTopLeft, kLeftSide), kRightSide);
					
					setCornerHeight(x, y2+1, kTopLeft, h, kRightSide);
					setCornerHeight(x, y2+1, kTopRight, h, kRightSide);
					setCornerHeight(x, y2+1, kTopLeft, h, kLeftSide);
				}
				else {
					setCornerHeight(x, y2+1, kTopLeft, h);
					setCornerHeight(x, y2+1, kTopRight, h);
				}
				break;
			case kBackwardSplit:
				setCornerHeight(x, y2+1, kTopLeft, h, kRightSide);
				setCornerHeight(x, y2+1, kTopRight, h, kRightSide);
				setCornerHeight(x, y2+1, kTopLeft, h, kLeftSide);
				break;
			case kForwardSplit:
				setCornerHeight(x, y2+1, kTopLeft, h, kLeftSide);
				setCornerHeight(x, y2+1, kTopRight, h, kLeftSide);
				setCornerHeight(x, y2+1, kTopRight, h, kRightSide);
				break;
		}
	}
	
	// left side
	for (int y = y1; y <= y2; y++)
	{
		setTerrainType(x1-1, y, type);
		switch(getSplit(x1-1, y)) {
			case kNoSplit:
				if (getCornerHeight(x1-1, y, kTopLeft) != getCornerHeight(x1-1, y, kBottomLeft))
				{ // need to split slanted tile
					setSplit(x1-1, y, kBackwardSplit); // split is arbitarary?
					setCornerHeight(x1-1, y, kBottomRight, getCornerHeight(x1-1, y, kTopRight, kLeftSide), kRightSide);
					setCornerHeight(x1-1, y, kTopLeft, getCornerHeight(x1-1, y, kTopLeft, kLeftSide), kRightSide);
					
					setCornerHeight(x1-1, y, kBottomRight, h, kLeftSide);
					setCornerHeight(x1-1, y, kBottomRight, h, kRightSide);
					setCornerHeight(x1-1, y, kTopRight, h, kRightSide);
				}
				else {
					setCornerHeight(x1-1, y, kTopRight, h);
					setCornerHeight(x1-1, y, kBottomRight, h);
				}
				break;
			case kBackwardSplit:
				setCornerHeight(x1-1, y, kBottomRight, h, kLeftSide);
				setCornerHeight(x1-1, y, kBottomRight, h, kRightSide);
				setCornerHeight(x1-1, y, kTopRight, h, kRightSide);
				//setCornerHeight(x1-1, y, kTopLeft, getCornerHeight(x1-1, y, kTopLeft, kLeftSide), kRightSide);
				break;
			case kForwardSplit:
				setCornerHeight(x1-1, y, kTopRight, h, kLeftSide);
				setCornerHeight(x1-1, y, kBottomRight, h, kRightSide);
				setCornerHeight(x1-1, y, kTopRight, h, kRightSide);
				break;
		}
	}
	
	// right side
	for (int y = y1; y <= y2; y++)
	{
		setTerrainType(x2+1, y, type);
		switch(getSplit(x2+1, y)) {
			case kNoSplit:
				if (getCornerHeight(x2+1, y, kTopLeft) != getCornerHeight(x2+1, y, kBottomLeft))
				{ // need to split slanted tile
					setSplit(x2+1, y, kForwardSplit); // split is arbitarary?
					setCornerHeight(x2+1, y, kTopRight, getCornerHeight(x2+1, y, kTopRight, kRightSide), kLeftSide);
					
					setCornerHeight(x2+1, y, kBottomLeft, h, kRightSide);
					setCornerHeight(x2+1, y, kBottomLeft, h, kLeftSide);
					setCornerHeight(x2+1, y, kTopLeft, h, kLeftSide);
				}
				else {
					setCornerHeight(x2+1, y, kTopLeft, h);
					setCornerHeight(x2+1, y, kBottomLeft, h);
				}
				break;
			case kBackwardSplit:
				setCornerHeight(x2+1, y, kTopLeft, h, kRightSide);
				setCornerHeight(x2+1, y, kBottomLeft, h, kLeftSide);
				setCornerHeight(x2+1, y, kTopLeft, h, kLeftSide);
				break;
			case kForwardSplit:
				setCornerHeight(x2+1, y, kBottomLeft, h, kRightSide);
				setCornerHeight(x2+1, y, kBottomLeft, h, kLeftSide);
				setCornerHeight(x2+1, y, kTopLeft, h, kLeftSide);
				break;
		}
	}
	
	
	setSplit(x1-1, y1-1, kForwardSplit);
	setTerrainType(x1-1, y1-1, type, kRightSide);
	setCornerHeight(x1-1, y1-1, kBottomRight, h, kRightSide);
	setCornerHeight(x1-1, y1-1, kBottomLeft, getCornerHeight(x1-1, y1-1, kBottomLeft, kLeftSide), kRightSide);
	setCornerHeight(x1-1, y1-1, kTopRight, getCornerHeight(x1-1, y1-1, kTopRight, kRightSide), kLeftSide);
	
	setSplit(x2+1, y2+1, kForwardSplit);
	setTerrainType(x2+1, y2+1, type, kLeftSide);
	setCornerHeight(x2+1, y2+1, kTopLeft, h, kLeftSide);
	setCornerHeight(x2+1, y2+1, kBottomLeft, getCornerHeight(x2+1, y2+1, kBottomLeft, kLeftSide), kRightSide);
	setCornerHeight(x2+1, y2+1, kTopRight, getCornerHeight(x2+1, y2+1, kTopRight, kRightSide), kLeftSide);
	
	setSplit(x1-1, y2+1, kBackwardSplit);
	setTerrainType(x1-1, y2+1, type, kRightSide);
	setCornerHeight(x1-1, y2+1, kTopRight, h, kRightSide);
	setCornerHeight(x1-1, y2+1, kTopLeft, getCornerHeight(x1-1, y2+1, kTopLeft, kLeftSide), kRightSide);
	setCornerHeight(x1-1, y2+1, kBottomRight, getCornerHeight(x1-1, y2+1, kBottomRight, kRightSide), kLeftSide);
	
	setSplit(x2+1, y1-1, kBackwardSplit);
	setTerrainType(x2+1, y1-1, type, kLeftSide);
	setCornerHeight(x2+1, y1-1, kBottomLeft, h, kLeftSide);
	setCornerHeight(x2+1, y1-1, kTopLeft, getCornerHeight(x2+1, y1-1, kTopLeft, kLeftSide), kRightSide);
	setCornerHeight(x2+1, y1-1, kBottomRight, getCornerHeight(x2+1, y1-1, kBottomRight, kRightSide), kLeftSide);
}

/**
* Set the height and terrain of a set of tiles.
 *
 * Sets all the tiles in the region between (x1, y1) (x2, y2) to be the same
 * height and terrain type, with no splits.
 */
void Map::setRectHeight(long x1, long y1, long x2, long y2, long h, tTerrain type)
{
	updated = true;
	map_name[0] = 0;
	revision++;
	printf("Doing rect between (%ld, %ld) and (%ld, %ld) height %ld\n", x1, y1, x2, y2, h);
	for (int x = x1; x <= x2; x++)
	{
		for (int y = y1; y <= y2; y++)
		{
			setSplit(x, y, kNoSplit);
			setTerrainType(x, y, type);
			setHeight(x, y, h);
		}
	}
}

/**
* Is the tile at x, y adjacent across the edge?
 *
 * given an edge (kInternalEdge, kLeftEdge, kRightEdge, kBottomEdge, kTopEdge)
 * returns whether the tiles on both sides of that edge have a smooth boundary
 * that a unit should be able to cross.
 */
bool Map::adjacentEdges(long x, long y, tEdge edge) const
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
		return false;
	switch (edge) {
		case kInternalEdge:
		{
			tSplit split;
			if ((split = getSplit(x, y)) == kNoSplit)
				return ((getTerrainType(x, y, kLeftSide)>>terrainBits) == (getTerrainType(x, y, kRightSide)>>terrainBits));
			else if (split == kForwardSplit)
			{
				return ((getCornerHeight(x, y, kTopRight, kLeftSide) == getCornerHeight(x, y, kTopRight, kRightSide)) &&
								(getCornerHeight(x, y, kBottomLeft, kLeftSide) == getCornerHeight(x, y, kBottomLeft, kRightSide)) &&
								((getTerrainType(x, y, kLeftSide)>>terrainBits) == (getTerrainType(x, y, kRightSide)>>terrainBits)));
			}
			else if (split == kBackwardSplit)
			{
				return ((getCornerHeight(x, y, kTopLeft, kLeftSide) == getCornerHeight(x, y, kTopLeft, kRightSide)) &&
								(getCornerHeight(x, y, kBottomRight, kLeftSide) == getCornerHeight(x, y, kBottomRight, kRightSide)) &&
								((getTerrainType(x, y, kLeftSide)>>terrainBits) == (getTerrainType(x, y, kRightSide)>>terrainBits)));
			}
			return false;
		} break;
		case kLeftEdge:
			if (x == 0)
				return false;
			return ((getCornerHeight(x, y, kTopLeft, kLeftEdge) == getCornerHeight(x-1, y, kTopRight, kRightEdge)) &&
							(getCornerHeight(x, y, kBottomLeft, kLeftEdge) == getCornerHeight(x-1, y, kBottomRight, kRightEdge)) &&
							((getTerrainType(x, y, kLeftSide)>>terrainBits) == (getTerrainType(x-1, y, kRightSide)>>terrainBits)));
			break;
		case kRightEdge:
			if (x+1 >= width)
				return false;
			return ((getCornerHeight(x, y, kTopRight, kRightEdge) == getCornerHeight(x+1, y, kTopLeft, kLeftEdge)) &&
							(getCornerHeight(x, y, kBottomRight, kRightEdge) == getCornerHeight(x+1, y, kBottomLeft, kLeftEdge)) &&
							((getTerrainType(x, y, kRightSide)>>terrainBits) == (getTerrainType(x+1, y, kLeftSide)>>terrainBits)));
			break;
		case kTopEdge:
			if (y == 0)
				return false;
			return ((getCornerHeight(x, y, kTopRight, kTopEdge) == getCornerHeight(x, y-1, kBottomRight, kBottomEdge)) &&
							(getCornerHeight(x, y, kTopLeft, kTopEdge) == getCornerHeight(x, y-1, kBottomLeft, kBottomEdge)) &&
							((getTerrainType(x, y, kTopEdge)>>terrainBits) == (getTerrainType(x, y-1, kBottomEdge)>>terrainBits)));
			
			break;
		case kBottomEdge:
			if (y+1 >= height)
				return false;
			return ((getCornerHeight(x, y, kBottomRight, kBottomEdge) == getCornerHeight(x, y+1, kTopRight, kTopEdge)) &&
							(getCornerHeight(x, y, kBottomLeft, kBottomEdge) == getCornerHeight(x, y+1, kTopLeft, kTopEdge)) &&
							((getTerrainType(x, y, kBottomEdge)>>terrainBits) == (getTerrainType(x, y+1, kTopEdge)>>terrainBits)));
			break;
	}
	return false;
}

bool Map::adjacentCorners(long x, long y, tCorner corner) const
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
		return false;
	switch (corner)
	{
		case kNone:
			return true;
		case kTopLeft:
			if (((x >= 1) && (y >= 1) && (adjacentEdges(x, y, kLeftEdge)) && (adjacentEdges(x, y, kTopEdge)) &&
					 (adjacentEdges(x, y-1, kLeftEdge)) && (adjacentEdges(x-1, y, kTopEdge))) &&
					(((adjacentEdges(x-1, y, kInternalEdge)) || (getSplit(x-1, y) == kBackwardSplit)) &&
					 ((adjacentEdges(x, y-1, kInternalEdge)) || (getSplit(x, y-1) == kBackwardSplit)) &&
					 ((adjacentEdges(x-1, y-1, kInternalEdge)) || (getSplit(x-1, y-1) == kForwardSplit)) &&
					 ((adjacentEdges(x, y, kInternalEdge)) || (getSplit(x, y) == kForwardSplit))))
				return true;
			return false;
		case kTopRight:
			if (((y >= 1) && (x < getMapWidth()-1) && (adjacentEdges(x, y, kRightEdge)) && (adjacentEdges(x, y, kTopEdge)) &&
					 (adjacentEdges(x, y-1, kRightEdge)) && (adjacentEdges(x+1, y, kTopEdge))) &&
					(((adjacentEdges(x+1, y, kInternalEdge)) || (getSplit(x+1, y) == kForwardSplit)) &&
					 ((adjacentEdges(x, y-1, kInternalEdge)) || (getSplit(x, y-1) == kForwardSplit)) &&
					 ((adjacentEdges(x+1, y-1, kInternalEdge)) || (getSplit(x+1, y-1) == kBackwardSplit)) &&
					 ((adjacentEdges(x, y, kInternalEdge)) || (getSplit(x, y) == kBackwardSplit))))
				return true;
			return false;
		case kBottomLeft: return adjacentCorners(x-1, y+1, kTopRight);
		case kBottomRight: return adjacentCorners(x+1, y+1, kTopLeft);
		default: return false;
	}
	return false;
}

bool Map::canStep(long x1, long y1, long x2, long y2) const
{
	if ((abs(x1-x2) > 1) || (abs(y1-y2) > 1))
		return false;
	switch (x1-x2) {
		case 0: //return true;
			switch (y1-y2) {
				case 0: return true;
				case 1: return adjacentEdges(x1, y1, kTopEdge);
				case -1: return adjacentEdges(x1, y1, kBottomEdge);
			}
			break;
		case 1: //return adjacentEdges(x1, y1, kLeftEdge);
			switch (y1-y2) {
				case 0: return adjacentEdges(x1, y1, kLeftEdge);
				case 1: return adjacentCorners(x1, y1, kTopLeft);
				case -1: return adjacentCorners(x1, y1, kBottomLeft);
			}
			break;
		case -1: //return adjacentEdges(x1, y1, kRightEdge);
			switch (y1-y2) {
				case 0: return adjacentEdges(x1, y1, kRightEdge);
				case 1: return adjacentCorners(x1, y1, kTopRight);
				case -1: return adjacentCorners(x1, y1, kBottomRight);
			}
			break;
	}
	return false;
}

/**
* Toggles whether the land is draw when you call OpenGLDraw
 */
void Map::setDrawLand(bool dLand)
{
	drawLand = dLand;
}

/**
* Choose the tileset used for land colors. Tilesets named xxxTile will draw
 * the map as independant tiles as opposed to a smooth connected map.
 */
void Map::setTileSet(tTileset ts)
{
	updated = true; // force the display list to re-draw
	tileSet = ts;
}

/**
* Get the tileset used for land colors.
 */
tTileset Map::getTileSet()
{
	return tileSet;
}


//void Map::setDrawAbstractions(int abstractions)
//{
//	numAbstractions = abstractions&0x7;
//}
	
/**
* Does actual OpenGL drawing of the map
 *
 * If drawLand has been set (on by default) the ground will be drawn using
 * the appropriate mode:   kPolygons, kLines, kPoints
 * kPolygon is the default mode. The map is cached in a display list unless
 * it changes.
 */
void Map::OpenGLDraw(int , tDisplay how)
{
	glDisable(GL_LIGHTING);
	if (drawLand)
	{
		if (updated)
		{
			updated = false;
			if (dList)
				glDeleteLists(dList, 1);
			dList = 0;
		}
		
		if (dList)
		{
			glCallList(dList);
		}
		else {
			if (verbose)
				printf("Drawing land into display list\n");
			dList = glGenLists(1);
			glNewList(dList, GL_COMPILE_AND_EXECUTE);
			
			if (tileSet == kFast)
			{
				drawLandQuickly();
			}
			else {
				for (int y = 0; y < height; y++)
				{
					for (int x = 0; x < width; x++)
					{
						drawTile(&land[x][y], x, y, how);
					}
				}
			}
			glEndList();
			// printf("Done\n");
		}
	}
}

/**
* Get the openGL coordinates of a given tile.
 *
 * Given a tile in (x, y) coordinates, it returns the OpenGL space coordinates of
 * that tile along with the radius of the tile square. The map is drawn in the
 * x<->z plane, with the y plane up.
 */
void Map::getOpenGLCoord(int _x, int _y, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const
{
	double _scale;
	if (height > width)
		_scale = 1/(double)height;
	else
		_scale = 1/(double)width;
	x = (2*_x-width)*_scale;
	y = (2*_y-height)*_scale;
	z = -(double)0.5*(land[_x][_y].tile1.corners[0]+land[_x][_y].tile2.corners[0])*(_scale);//+(double)land[_x][_y].tile1.corners[1]/(2*_scale));
																																												 //	x = -((double)_x/_scale+1.0/(2.0*_scale)-.5);
																																												 //	z = -((double)land[_x][_y].tile1.corners[0]/(2*_scale)+(double)land[_x][_y].tile1.corners[1]/(2*_scale));
																																												 //	//(double)land[_x][_y].tile1.corners[2]/(2*_scale)+(double)land[_x][_y].tile2.corners[0]/(2*_scale))/2.0;
																																												 //	y = (double)-_y/_scale+1.0/(2.0*_scale)-.5;
		radius = _scale;
}

/**
* Returns the scale multiplier between openGL coordinates and map coordinates.
 * If you measure a distance in openGL coordinate space, you can multiply it by
 * this value to convert it to map space, where the distance between adjacent tiles
 * is 1.
 */
double Map::getCoordinateScale()
{
	//	double scale;
	if (height > width)
		return (double)height/2.0;
	return (double)width/2.0;
}

void Map::getPointFromCoordinate(point3d loc, int &px, int &py) const
{
	double _x, _y;
	double _scale;
	if (height > width)
		_scale = 1/(double)height;
	else
		_scale = 1/(double)width;
	_x = (loc.x/_scale+(double)width)/2.0;
	_y = (loc.y/_scale+(double)height)/2.0;
	px = (int)(_x+0.5); // round off!
	py = (int)(_y+0.5);
	/*
	 px = (int)(((loc.x/2.0)+0.5)*((double)width));
	 py = (int)(((loc.y/2.0)+0.5)*((double)height));
	 */
	if ((px < 0) || (py < 0) || (px >= width) || (py >= height))
	{
		px = py = -1;
	}
}

/**
* Draw a single tile.
 *
 * draws a single tile which is at (x, y) in the land
 * (technically we don't have to pass the tile, we could get it
		* from the x,y coordinates)
 */
void Map::drawTile(Tile *t, int x, int y, tDisplay how)
{
	GLdouble xx, yy, zz, rr;
	getOpenGLCoord(x,y,xx,yy,zz,rr);
	
	switch (how) {
		case kPolygons:
			if (t->split == kNoSplit)
				glBegin(GL_QUADS);
			else
				glBegin(GL_TRIANGLES);
			break;
		case kLines:
			glBegin(GL_LINE_LOOP);
			break;
		case kPoints:
		default:
			glBegin(GL_POINTS);
			break;
	}
	switch (t->split) {
		case kNoSplit:
			if (t->tile1.type != kOutOfBounds)
			{
				if ((tileSet == kWinterTile) || 
						(tileSet == kFallTile))
					rr *= 0.9;   // Leave empty grid lines between
				
				doVertexColor(t->tile1.type, t->tile1.corners[0], !adjacentCorners(x, y, kTopLeft));
				doNormal(t->split, &t->tile1, x, y);
				glVertex3f(xx-rr, yy-rr, -rr*t->tile1.corners[0]);
				
				doVertexColor(t->tile1.type, t->tile1.corners[1], !adjacentCorners(x, y, kBottomLeft));
				glVertex3f(xx-rr, yy+rr, -rr*t->tile1.corners[1]);
				
				doVertexColor(t->tile1.type, t->tile1.corners[2], !adjacentCorners(x, y, kBottomRight));
				glVertex3f(xx+rr, yy+rr, -rr*t->tile1.corners[2]);
				
				doVertexColor(t->tile1.type, t->tile2.corners[0], !adjacentCorners(x, y, kTopRight));
				glVertex3f(xx+rr, yy-rr, -rr*t->tile2.corners[0]);
			}
			break;
		case kForwardSplit:
			if (t->tile1.type != kOutOfBounds)
			{
				doNormal(t->split, &t->tile1, x, y);
				doVertexColor(t->tile1.type, t->tile1.corners[0]);
				glVertex3f(xx-rr, yy-rr, -rr*t->tile1.corners[0]);
				doVertexColor(t->tile1.type, t->tile1.corners[1]);
				glVertex3f(xx-rr, yy+rr, -rr*t->tile1.corners[1]);
				doVertexColor(t->tile1.type, t->tile1.corners[2]);
				glVertex3f(xx+rr, yy-rr, -rr*t->tile1.corners[2]);
			}
			if (how == kLines)
			{ glEnd(); glBegin(GL_LINE_LOOP); }
			if (t->tile2.type != kOutOfBounds)
			{
				doNormal(t->split, &t->tile2, x, y);
				doVertexColor(t->tile2.type, t->tile2.corners[0]);
				glVertex3f(xx+rr, yy-rr, -rr*t->tile2.corners[0]);
				doVertexColor(t->tile2.type, t->tile2.corners[1]);
				glVertex3f(xx+rr, yy+rr, -rr*t->tile2.corners[1]);
				doVertexColor(t->tile2.type, t->tile2.corners[2]);
				glVertex3f(xx-rr, yy+rr, -rr*t->tile2.corners[2]);
			}
			break;
		case kBackwardSplit:
			if (t->tile1.type != kOutOfBounds)
			{
				doNormal(t->split, &t->tile1, x, y);
				doVertexColor(t->tile1.type, t->tile1.corners[0]);
				glVertex3f(xx-rr, yy-rr, -rr*t->tile1.corners[0]);
				//glVertex3f((double)x/width-.5, (double)2.0*t->tile1.corners[0]/(height+width), (double)y/height-.5);
				doVertexColor(t->tile1.type, t->tile1.corners[1]);
				glVertex3f(xx-rr, yy+rr, -rr*t->tile1.corners[1]);
				//glVertex3f((double)x/width-.5, (double)2.0*t->tile1.corners[1]/(height+width), (double)y/height+1.0/height-.5);
				doVertexColor(t->tile1.type, t->tile1.corners[2]);
				glVertex3f(xx+rr, yy+rr, -rr*t->tile1.corners[2]);
				//glVertex3f((double)x/width+1.0/width-.5, (double)2.0*t->tile1.corners[2]/(height+width), (double)y/height+1.0/height-.5);
			}
			if (how == kLines)
			{ glEnd(); glBegin(GL_LINE_LOOP); }
			
			if (t->tile2.type != kOutOfBounds)
			{
				doNormal(t->split, &t->tile2, x, y);
				doVertexColor(t->tile2.type, t->tile2.corners[0]);
				glVertex3f(xx+rr, yy-rr, -rr*t->tile2.corners[0]);
				//glVertex3f((double)x/width+1.0/width-.5, (double)2.0*t->tile2.corners[0]/(height+width), (double)y/height-.5);
				doVertexColor(t->tile2.type, t->tile2.corners[1]);
				glVertex3f(xx+rr, yy+rr, -rr*t->tile2.corners[1]);
				//glVertex3f((double)x/width+1.0/width-.5, (double)2.0*t->tile2.corners[1]/(height+width), (double)y/height+1.0/height-.5);
				doVertexColor(t->tile2.type, t->tile2.corners[2]);
				glVertex3f(xx-rr, yy-rr, -rr*t->tile2.corners[2]);
				//glVertex3f((double)x/width-.5, (double)2.0*t->tile2.corners[2]/(height+width), (double)y/height-.5);
			}
			break;
	}
	glEnd();
}

/**
* Using OpenGL set the correct color for a particular vertex.
 *
 * calls the appropriate openGL functions to set the draw color according
 * to the tile height/type
 */
void Map::doVertexColor(tTerrain type, int vHeight, bool darken)
{
	double scaleH = (10.0-vHeight)/10.0;
	double red=0, green=0, blue=0, alpha = 1.0;
	switch (type)
	{
		case kOutOfBounds:
			red = 0; green = 0.2; blue = 0; alpha = .5; // kOutOfBounds
			break;
		case kOutOfBounds2:
			red = .9; green = .9; blue = .9; alpha = .5; // kOutOfBounds2
			break;
		case kWater:
			red = 0; green = (random()%10)/100.0; blue = scaleH*1-(random()%10)/100.0;
			break;
		case kGround:
			if ((tileSet == kFall) || (tileSet == kFallTile))
			{
				double r1=0, r2=0, r3=0;
				if (tileSet == kFall)
				{
					r1 = (random()%10-5)/100.0;
					r2 = (random()%10-5)/100.0;
					r3 = (random()%5)/100.0;
				}
				red = scaleH*.5+r1;
				green = scaleH*.25+r2;
				blue = r3; // kGround
			}
			else {
				double r1=0, r2=0, r3=0;
				if (tileSet == kWinter)
				{
					r1 = -(random()%8)/100.0;
					r2 = -(random()%8)/100.0;
					r3 = -(random()%8)/100.0;
				}
				red = scaleH*0.9+r1;
				green = scaleH*0.9+r2;
				blue = scaleH*0.9+r3;
			}
			break;
		case kTrees:
			red = (random()%10)/100.0;
			green = .45*scaleH-(random()%10)/100.0;
			blue = (random()%10)/100.0;
			break;
		case kSwamp:
			red = (scaleH*.5+(random()%10-5)/100.0)*0.9;
			green = (scaleH*.25+(random()%10-5)/100.0)*0.9;
			blue = (80+random()%10-5)/100.0;
			break;
		case kGrass:
			red = (random()%10)/100.0;
			green = scaleH-(random()%10)/100.0;
			blue = (random()%10)/100.0;
			break;
		case kBlight:
			red = (50+random()%10-5)/100.0;
			green = (50+random()%10-5)/100.0;
			blue = (50+random()%10-5)/100.0;
			break;
		default:
			break;
	}
	if ((darken) && (type != kGround))
		glColor4f(.5*red, .5*green, .5*blue, alpha);
	else if ((darken) && (type == kGround))
		glColor4f(.8*red, .8*green, .8*blue, alpha);
	else
		glColor4f(red, green, blue, alpha);
}

/**
* does a rough approximation of the normal for a particular halfTile.
 * (if I recall, this isn't perfect...)
 */
void Map::doNormal(tSplit split, halfTile *t, int /*x*/, int /*y*/)
{
	recVec n,pa,pb;
	
	pa.x = 0;
	pa.y = (double)(t->corners[1]-t->corners[0])/(height+width);
	pa.z = 1/height;
	
	pb.x = 1/width;
	switch (split) {
		case kNoSplit:
		case kBackwardSplit:
			pb.z = 1/height;
			break;
		case kForwardSplit:
			pb.z = 0;
			break;
	}
	pb.y = (double)(t->corners[2]-t->corners[0])/(height+width);
	pa.normalise();
	pb.normalise();
	
	n.x = pb.y * pa.z - pb.z * pa.y;
	n.y = pb.z * pa.x - pb.x * pa.z;
	n.z = pb.x * pa.y - pb.y * pa.x;
	n.normalise();
	
	glNormal3f(n.x,n.y,n.z);
}

void Map::drawLandQuickly()
{
	GLdouble xx, yy, zz, rr;
	glBegin(GL_QUADS);
	glColor3f(0.5, 0.5, 0.5);
	for (int y = 0; y < height; y++)
	{
		if (getTerrainType(0, y) == kGround)
		{
			getOpenGLCoord(0, y, xx, yy, zz, rr);
			glVertex3f(xx-rr, yy-rr, zz);
			glVertex3f(xx-rr, yy+rr, zz);
		}
		for (int x = 1; x < width; x++)
		{
			if (getTerrainType(x, y) != getTerrainType(x-1, y))
			{
				if (getTerrainType(x-1, y) == kGround)
				{
					getOpenGLCoord(x, y, xx, yy, zz, rr);
					glVertex3f(xx-rr, yy+rr, zz);
					glVertex3f(xx-rr, yy-rr, zz);
				}
				if (getTerrainType(x, y) == kGround)
				{
					getOpenGLCoord(x, y, xx, yy, zz, rr);
					glVertex3f(xx-rr, yy-rr, zz);
					glVertex3f(xx-rr, yy+rr, zz);
				}
			}
		}
		if (getTerrainType(width-1, y) == kGround)
		{
			getOpenGLCoord(width-1, y, xx, yy, zz, rr);
			glVertex3f(xx+rr, yy+rr, zz);
			glVertex3f(xx+rr, yy-rr, zz);
		}
	}

//  this will draw the lines for the map, but can be too busy
//	for (int y = 0; y < height; y++)
//	{
//		getOpenGLCoord(0, y, xx, yy, zz, rr);
//		glVertex3f(xx-rr, yy-rr, zz);
//		getOpenGLCoord(width-1, y, xx, yy, zz, rr);
//		glVertex3f(xx+rr, yy-rr, zz);
//	}
//	for (int y = 0; y < height; y++)
//	{
//		getOpenGLCoord(x, 0, xx, yy, zz, rr);
//		glVertex3f(xx-rr, yy-rr, zz);
//		getOpenGLCoord(x, height-1, xx, yy, zz, rr);
//		glVertex3f(xx-rr, yy+rr, zz);
//	}
	glEnd();
}

/**
* Sets the abstract graph node number for this tile.
 *
 * Because we have a graph representation of the map as well, we need some
 * way to get back and forth between the representations. This function will
 * set the unique data (nodeNum) for a tile/half tile so that we can go
 * from a tile in the map to a node in the graph.
 */
void Map::setNodeNum(int num, int x, int y, tCorner corner)
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
	{
		printf("ERROR -- trying to set invalid node number!\n");
		return;
	}
	if ((corner == kBottomRight) || (corner == kTopRight))
		land[x][y].tile2.node = num;
	land[x][y].tile1.node = num;
}

/**
* Gets the abstract graph node number for this tile.
 *
 * Because we have a graph representation of the map as well, we need some
 * way to get back and forth between the representations. This function will
 * get the unique data (nodeNum) for a tile/half tile so that we can go
 * from a tile in the map to a node in the graph.
 */
int Map::getNodeNum(int x, int y, tCorner corner)
{
	if ((x < 0) || (y < 0) || (x >= width) || (y >= height))
	{
		//printf("ERROR -- trying to get invalid node number!\n");
		return -1;
	}
	if ((corner == kBottomRight) || (corner == kTopRight))
		return land[x][y].tile2.node;
	return land[x][y].tile1.node;
}

/*
 *
 * Lookup table for the edge widths.
 *
 * The edges are as follows:
 *
 *  ---------
 *  |   |   |
 *  ---------
 *  |  -|-  |
 *  ---------
 *
 * To cross the bottom middle edge.
 *
 * Tile types:
 *   Empty: 0
 *   Forward slash: 1
 *   Backslash: 2
 *
 *
 * To translate into the lookup table index, multiply
 * tile 1's type by 27, multiply tile 2's type by 9,
 * multiply tile 3's type by 3, and add all the values together,
 * along with tile 4.
 *
 * So you end up with: 27*T1 + 9*T2 + 3*T3 + T4
 *
 * For example,
 *
 *  ---------
 *  | \ | / |
 *  ---------
 *  |   | \ |
 *  ---------
 *
 * 27*2 + 9*1 + 3*0 + 2 = 65
 *
 * The edge width will be found in index 65.
 *
 */

extern "C" {
	
  float edgewidth[81] = {
    TWO,
    ROOT_TWO,
    ONE,
    ONE,
    ONE,
		
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
    ROOT_TWO,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ROOT_TWO,
    ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
		
    ROOT_TWO,
    ONE,
    ROOT_TWO,
    ROOT_TWO,
    ONE,
		
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
    ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE,
		
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
		
    ROOT_TWO,
    ROOT_TWO,
    ONE,
    ONE,
    ONE,
		
    ONE_OVER_ROOT_TWO,
    ROOT_TWO,
    ROOT_TWO,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
		
    ONE,
    ONE,
    ONE,
    ONE,
    ONE,
		
    ONE,
    ONE,
    ONE_OVER_ROOT_TWO,
    ONE,
    ONE,
		
    ONE
  };
}

/**
* Returns the edge width between (x, y) and (x+1, y)
 */
float Map::getEdgeWidthX(int x, int y)
{
	// Boundary values
	if (x < 0 || y-1 < 0 || x+1 >= width || y+1 >= height)
		return 0.0f;
	
	int index1, index2;	
	
	// Check if the edge at (x+1, y) is adjacent (i.e. same height and terrain type)
	if (!adjacentEdges(x, y, kRightEdge))
	{
		// different height or terrain, so undefined
		return 0.0f;
	}
	
	// Check to see that the top edges are both adjacent.  If either is not, get the edge width using the bottom two tiles
	if (!adjacentEdges(x, y, kTopEdge) || !adjacentEdges(x+1, y, kTopEdge))
	{
		//(x+1, y+1), (x, y+1), (x+1, y), (x, y)  
		index1 = (27*getSplit(x+1, y+1)) + (9*getSplit(x, y+1)) + (3*getSplit(x+1, y)) + (getSplit(x, y));
		return edgewidth[index1];
	}
	
	// Check to see that the bottom edges are both adjacent.  If either is not, get the edge width using the top two tiles
	if (!adjacentEdges(x, y, kBottomEdge) || !adjacentEdges(x+1, y, kBottomEdge))
	{
		//(x, y-1), (x+1, y-1), (x, y) (x+1, y)
		index1 = (27*getSplit(x, y-1)) + (9*getSplit(x+1, y-1)) + (3*getSplit(x, y)) + (getSplit(x+1, y));
		return edgewidth[index1];
	}
	
	// Otherwise, return the minimum of the edge widths from the top edges and bottom edges
	index1 = (27*getSplit(x+1, y+1)) + (9*getSplit(x, y+1)) + (3*getSplit(x+1, y)) + (getSplit(x, y));
	index2 = (27*getSplit(x, y-1)) + (9*getSplit(x+1, y-1)) + (3*getSplit(x, y)) + (getSplit(x+1, y));
	
	return (edgewidth[index1] < edgewidth[index2]) ? (edgewidth[index1]) : (edgewidth[index2]);
}



/**
* Returns the edge width between (x, y) and (x, y+1)
 */
float Map::getEdgeWidthY(int x, int y)
{
	// Boundary values
	if (x-1 < 0 || y < 0 || x+1 >= width || y+1 >= height)
		return 0.0f;
	
	int index1, index2;	
	
	// Check if the edge at (x, y+1) is adjacent (i.e. same height and terrain type)
	if (!adjacentEdges(x, y, kBottomEdge))
	{
		// different height or terrain, so undefined
		return 0.0f;
	}
	
	// Check to see that the right edges are both adjacent.  If either is not, get the edge width using the left two tiles
	if (!adjacentEdges(x, y, kRightEdge) || !adjacentEdges(x, y+1, kRightEdge))
	{
		//(x-1, y-1), (x-1, y), (x, y+1), (x, y)  
		index1 = (27*getSplit(x-1, y-1)) + (9*getSplit(x-1, y)) + (3*getSplit(x, y+1)) + (getSplit(x, y));
		return edgewidth[index1];
	}
	
	// Check to see that the left edges are both adjacent.  If either is not, get the edge width using the right two tiles
	if (!adjacentEdges(x, y, kLeftEdge) || !adjacentEdges(x, y+1, kLeftEdge))
	{
		//(x+1, y), (x+1, y+1), (x, y) (x, y+1)
		index1 = (27*getSplit(x+1, y)) + (9*getSplit(x+1, y+1)) + (3*getSplit(x, y)) + (getSplit(x, y+1));
		return edgewidth[index1];
	}
	
	// Otherwise, return the minimum of the edge widths from the top edges and bottom edges
	index1 = (27*getSplit(x-1, y-1)) + (9*getSplit(x-1, y)) + (3*getSplit(x, y+1)) + (getSplit(x, y));
	index2 = (27*getSplit(x+1, y)) + (9*getSplit(x+1, y+1)) + (3*getSplit(x, y)) + (getSplit(x, y+1));
	
	return (edgewidth[index1] < edgewidth[index2]) ? (edgewidth[index1]) : (edgewidth[index2]);
}


/**
* makeMaze(map, pathsize)
 *
 * A cheap function I hacked together to make psuedo mazes.
 * The only good values for pathSize are 1 and 3.
 */
void makeMaze(Map *map, int pathSize)
{
	int width = map->getMapWidth();
	int height = map->getMapHeight();
	map->setRectHeight(0, 0, width-1, height-1, 0, kGround);
	int pathWidth = 1;
	for (int t = 0; (pathSize>>t); t++)
		pathWidth = t+1;
	for (int i = 0; i < 6*width*height; i++)
	{
		long x = (random()%width)&(~pathSize); // only path on even tiles
		long y = (random()%height)&(~pathSize);
		
		if (map->getHeight(x, y) <= 1)
		{
			switch(random()%4)
			{
				case 0: // NORTH
					if ((x >= 2*pathWidth) && (map->getHeight(x-2*pathWidth, y)+map->getHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->setHeight(x-2, y+t, map->getHeight(x-2, y)+1);
							map->setHeight(x-1, y+t, map->getHeight(x-1, y)+1);
							map->setHeight(x, y+t, map->getHeight(x, y)+1);
						}
					}
					break;
				case 1: // SOUTH
					if ((x < width-2*pathWidth) && (map->getHeight(x+2*pathWidth, y)+map->getHeight(x, y) < 2))
					{ 
						for (int t = 0; t < pathWidth; t++)
						{
							map->setHeight(x+2, y+t, map->getHeight(x+2, y)+1);
							map->setHeight(x+1, y+t, map->getHeight(x+1, y)+1);
							map->setHeight(x, y+t, map->getHeight(x, y)+1);
						}
					}
					break;
				case 2: // WEST
					if ((y >= 2*pathWidth) && (map->getHeight(x, y-2*pathWidth)+map->getHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->setHeight(x+t, y-2, map->getHeight(x, y-2)+1);
							map->setHeight(x+t, y-1, map->getHeight(x, y-1)+1);
							map->setHeight(x+t, y, map->getHeight(x, y)+1);
						}
					}
					break;
				case 3: // EAST
					if ((y < height-2*pathWidth) && (map->getHeight(x, y+2*pathWidth)+map->getHeight(x, y) < 2))
					{
						for (int t = 0; t < pathWidth; t++)
						{
							map->setHeight(x+t, y+2, map->getHeight(x, y+2)+1);
							map->setHeight(x+t, y+1, map->getHeight(x, y+1)+1);
							map->setHeight(x+t, y, map->getHeight(x, y)+1);
						}
					}
					break;
			}
		}
	}
	for (int x = 0; x < width; x++)
		for (int y = 0; y < height; y++)
			if (map->getHeight(x, y) > 0)
			{
				map->setHeight(x, y, 0);
				map->setTerrainType(x, y, kOutOfBounds);
			}
}

