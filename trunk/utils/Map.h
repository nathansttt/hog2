/*
 * $Id: map.h,v 1.20 2007/03/07 22:01:05 nathanst Exp $
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

// HOG File

#ifndef MAP_H
#define MAP_H

static const double ONE = 1.0f;
static const double TWO = 2.0f;
static const double ROOT_TWO = 1.414213562f;//1.5f?
static const double ONE_OVER_ROOT_TWO = 1.0/ROOT_TWO;//0.707106781f;

#include <cassert>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <unistd.h>
#include <iostream>

#include "GLUtil.h"
#include "Graph.h"

enum tTileset {
	kFall,
	kFallTile,
	kWinter,
	kWinterTile,
	kFast
};

enum tDisplay {
  kPolygons,
  kLines,
  kPoints
};

// types of ground
//  bit definition:
//  low 4 bits are subcategories
//  high bits are large categories

const int terrainBits = 4;

enum tTerrain {
  kOutOfBounds =0x0, // not part of map
  kOutOfBounds2=0x1, // not part of map - different color
  kWater=0x10,     // water
  kGround=0x20,     // ground
  kSwamp=0x21,
	kGrass=0x22,
	kBlight=0x23,
  kTrees=0x30,
  kUndefined // mixed type due to split tile
};

inline bool CanPass(long a, long b)
{ return (((a&b)&0xF0) != 0); }

// ground split types
enum tSplit {
  kNoSplit=0,
  kForwardSplit=1, // ie like forward slash
  kBackwardSplit=2     // ie like back slash
};

// specifying which side of the split you want
enum tSplitSide {
  kWholeTile = 1,
  kLeftSide = 2,
  kRightSide = 3
};

enum tEdge {
  kInternalEdge,
  kLeftEdge,
  kRightEdge,
  kTopEdge,
  kBottomEdge
};

// other constants
enum {
	kUndefinedHeight = -999
};

const int kNoGraphNode = -1;

// corner types
enum tCorner {
  kNone = 0,
  kTopLeft = 1,
  kTopRight = 2,
  kBottomLeft = 3,
  kBottomRight = 4
};

class halfTile {
public:
  halfTile();

  long corners[3];
  tTerrain type;
  long node;
};

class Tile {
public:
	Tile();
	halfTile tile1, tile2;
	tSplit split;
};

enum tMapType {
	kOctile,
	kOctileCorner,
	kSokoban,
	kRaw
};

/*
 *
 * All the set/get functions should be fairly obvious, with the following
 * behavior:
 *
 * Setting the height/type without a split for a tile will reset it to be a
 * non-split tile.
 *
 * If you specify a split for a tile that is not split, 
 */

/**
 * A tile-based representation of the world.
 */

class Map {
public:
  Map(long width, long height);
  Map(const char *filename);
	Map(Map *);
	Map(FILE *);
  Map(std::istringstream &data);
  ~Map();
  void load(const char *filename);
  void load(FILE *f);
	void setSizeMultipler(int _sizeMultiplier)
	{ sizeMultiplier = _sizeMultiplier; }
	void scale(long newWidth, long newHeight);
  void save(std::stringstream &data);
  void save(const char *filename);
  void save(FILE *f);
	Map *clone() { return new Map(this); }
	const char *getMapName();
	void print(int scale = 1);
	/** return the width of the map */
  inline long getMapWidth() const { return width; }
	/** return the height of the map */
  inline long getMapHeight() const { return height; }

	void setTileSet(tTileset ts);
	tTileset getTileSet();
	
  Tile &getTile(long x, long y);
	
  tSplit getSplit(long x, long y) const;
  void setSplit(long x, long y, tSplit split);


  // returns kUndefined if tile is split and you request kWholeTile
  long getTerrainType(long x, long y, tSplitSide split = kWholeTile) const;
  long getTerrainType(long x, long y, tEdge side) const;
	void setTerrainType(int32_t x1, int32_t y1,
						int32_t x2, int32_t y2, tTerrain t);

  // if tile is not split and you specify a split side, nothing happens
  // if tile is split and you specify kWholeTile, the split remains,
  // and the terrain is applied to both sides
  void setTerrainType(long x, long y, tTerrain type, tSplitSide split = kWholeTile);

  // returns kUndefinedHeight if the tile is split and you specify
  // the whole tile
  long getHeight(long x, long y, tSplitSide split = kWholeTile);

  // if you specify a split, and the tile isn't split that direction,
  // the tile will be split that direction, and then the height applied
  void setHeight(long x, long y, long height, tSplitSide split = kWholeTile);

  // returns kUndefinedHeight if the split is inconsistant with the tile type
  long getCornerHeight(long x, long y, tCorner which, tEdge edge) const;
  long getCornerHeight(long x, long y, tCorner which, tSplitSide split = kWholeTile) const;
  void setCornerHeight(long x, long y, tCorner which, long height, tSplitSide split = kWholeTile);


  // ===============================================
  void smoothSetRectHeight(long x1, long y1, long x2, long y2, long h, tTerrain type = kGround);
  void setRectHeight(long x1, long y1, long x2, long y2, long h, tTerrain type = kGround);
  bool adjacentEdges(long x, long y, tEdge edge) const;
  bool adjacentCorners(long x, long y, tCorner corner) const;
  // returns whether we can step between two locations or not
  bool canStep(long x1, long y1, long x2, long y2) const;
  
  void OpenGLDraw(tDisplay how = kPolygons) const;
  void getOpenGLCoord(int _x, int _y, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const;
  void getOpenGLCoord(float _x, float _y, GLdouble &x, GLdouble &y, GLdouble &z, GLdouble &radius) const;
  void getPointFromCoordinate(point3d loc, int &px, int &py) const;
	double getCoordinateScale();

  void setDrawLand(bool land);
  bool getDrawLand() { return drawLand; }
  void drawTile(Tile *t, int x, int y, tDisplay how) const;
  void doVertexColor(tTerrain type, int height, bool darken = false) const;
  void doNormal(tSplit split, halfTile *t, int x, int y) const;

  float getEdgeWidthX(int x, int y);
  float getEdgeWidthY(int x, int y);

  int getNodeNum(int x, int y, tCorner c = kNone);
  void setNodeNum(int num, int x, int y, tCorner c = kNone);
  int getRevision() { return revision; }
private:
	void loadRaw(FILE *f, int height, int width);
	void loadOctile(FILE *f, int height, int width);
	void loadOctileCorner(FILE *f, int height, int width);
	void saveOctile(FILE *f);
	void saveRaw(FILE *f);
	bool tryLoadRollingStone(FILE *f);
	bool isLegalStone(char c);
	void paintRoomInside(int x, int y);
	void drawLandQuickly() const;
	int width, height;
  Tile **land;
  bool drawLand;
	mutable GLuint dList;
	mutable bool updated;
	int sizeMultiplier;
  int revision;
	char map_name[128];
	tMapType mapType;
	tTileset tileSet;
};

void MakeMaze(Map *map);
void MakeMaze(Map *map, int pathSize);
void BuildRandomRoomMap(Map *map, int roomSize);

#endif
