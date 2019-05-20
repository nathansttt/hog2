//
//  DynamicWeightedGrid.h
//  Dynamic Weighted Abstraction
//
//  Created by Nathan Sturtevant on 5/16/19.
//  Copyright © 2019 University of Denver. All rights reserved.
//

#ifndef DynamicWeightedGrid_h
#define DynamicWeightedGrid_h

#include <stdio.h>
#include "Map2DEnvironment.h"

namespace DWG {
	
	enum TerrainType {
		kImpassable = 0,
		kGround = 1,
		kTrees = 2,
		kSwamp = 3,
		kRoad = 4,
		kWater = 5
	};
	
	struct edge {
//		edge(int f, int t) :from(f), to(t) {}
		int sectorFrom, regionFrom;
		int sectorTo, regionTo;
	};
	
	const uint8_t kNoRegion = 0xFF;
	
	template <int sectorSize>
	struct SectorData {
		uint8_t cells[sectorSize][sectorSize];
		uint8_t region[sectorSize][sectorSize];
		std::vector<edge> edges;
		std::vector<xyLoc> regionCenters;
	};
	
	// TODO: Build DynamicWeightedGridAbstractionEnvironment that you can use for search
	// this code builds the abstraction and draws it, but can't search it
	template <int sectorSize>
	class DynamicWeightedGrid : public SearchEnvironment<xyLoc, tDirection> {
	public:
		DynamicWeightedGrid(int width, int height)
		:mWidth(width), mHeight(height)
		{
			sectors.resize(GetNumSectors());
			for (uint16_t y = 0; y < mHeight; y++)
			{
				for (uint16_t x = 0; x < mWidth; x++)
				{
					SetTerrainTypeNoRepair({x, y}, kGround);
				}
			}
			BuildAbstraction();
		}
//		DynamicWeightedGrid(Map *map, int theSectorSize);
		int GetWidth() const { return mWidth; }
		int GetHeight() const { return mHeight; }
		void SetTerrainType(const xyLoc &l, TerrainType t)
		{
			int sec = GetSector(l);
			int x, y;
			GetSectorOffset(l, x, y);
			if (sectors[sec].cells[x][y] != t)
			{
				sectors[sec].cells[x][y] = t;
				GetRegions(sec);
				GetEdges(sec);
				GetEdges(sec-1);
				GetEdges(sec+1);
				GetEdges(sec-GetNumYSectors());
				GetEdges(sec-GetNumYSectors()+1);
				GetEdges(sec-GetNumYSectors()-1);
				GetEdges(sec+GetNumYSectors());
				GetEdges(sec+GetNumYSectors()+1);
				GetEdges(sec+GetNumYSectors()-1);
			}
		}
		TerrainType GetTerrainType(const xyLoc &l) const
		{
			int sec = GetSector(l);
			int x, y;
			GetSectorOffset(l, x, y);
			return static_cast<TerrainType>(sectors[sec].cells[x][y]);
		}

		void GetSuccessors(const xyLoc &nodeID, std::vector<xyLoc> &neighbors) const { }
		void GetActions(const xyLoc &nodeID, std::vector<tDirection> &actions) const { }
		void ApplyAction(xyLoc &s, tDirection a) const { }
		bool InvertAction(tDirection &a) const { return false; }
		tDirection GetAction(const xyLoc &s1, const xyLoc &s2) const { return tDirection(); }
		/** Heuristic value between two arbitrary nodes. **/
		double HCost(const xyLoc &node1, const xyLoc &node2) const { }
		double GCost(const xyLoc &node1, const xyLoc &node2) const { }
		double GCost(const xyLoc &node, const tDirection &act) const { }
		bool GoalTest(const xyLoc &node, const xyLoc &goal) const { }
		
		uint64_t GetStateHash(const xyLoc &node) const { }
		uint64_t GetActionHash(tDirection act) const { }
		
		void Draw(Graphics::Display &display) const;
		void Draw(Graphics::Display &display, const xyLoc &) const;
		void OpenGLDraw() const {}
		void OpenGLDraw(const xyLoc&) const {}
		void OpenGLDraw(const xyLoc&, const tDirection&) const { }
		void GLDrawLine(const xyLoc &x, const xyLoc &y) const {}
		
		void GetPointFromCoordinate(point3d loc, int &px, int &py) const;

	private:
		void BuildAbstraction();
		void GetRegions(int sector);
		void GetEdges(int sector);
		xyLoc BFS(SectorData<sectorSize> &d, int x, int y, int whichRegion);
		void AddEdge(SectorData<sectorSize> &d, int s1, int r1, int s2, int r2);

		int GetNumSectors() const { return ((mWidth+sectorSize-1)/sectorSize)*((mHeight+sectorSize-1)/sectorSize); }
		int GetNumXSectors() const { return ((mWidth+sectorSize-1)/sectorSize); }
		int GetNumYSectors() const { return ((mHeight+sectorSize-1)/sectorSize); }
		int GetRegion(const xyLoc &l)
		{
			int x, y;
			GetSectorOffset(l, x, y);
			return sectors[GetSector(l)].region[x][y];
		}
		int GetSector(const xyLoc &l) const
		{
			int secx = l.x/sectorSize;
			int secy = l.y/sectorSize;
			return secy*GetNumYSectors()+secx;
		}
		void GetSectorOffset(const xyLoc &l, int &x, int &y) const
		{
			x = l.x%sectorSize;
			y = l.y%sectorSize;
		}
		void SetTerrainTypeNoRepair(const xyLoc &l, TerrainType t)
		{
			int sec = GetSector(l);
			int x, y;
			GetSectorOffset(l, x, y);
			if (sectors[sec].cells[x][y] != t)
			{
				sectors[sec].cells[x][y] = t;
			}
		}
		void GetCoordinate(const xyLoc &l, float &x, float &y, float &r) const;
		std::vector<SectorData<sectorSize>> sectors;
		int mWidth, mHeight;
		bool drawGrid;
	};

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::Draw(Graphics::Display &display) const
	{
		for (uint16_t y = 0; y < mHeight; y++)
		{
			for (uint16_t x = 0; x < mWidth; x++)
			{
				Draw(display, {x, y});
			}
		}
		for (int x = 0; x < mWidth; x+=sectorSize)
		{
			float xx1, yy1, rr, xx2, yy2;
			xyLoc l1(x, 0), l2(x, mHeight);
			GetCoordinate(l1, xx1, yy1, rr);
			GetCoordinate(l2, xx2, yy2, rr);
			display.DrawLine({xx1-rr, yy1-rr}, {xx2-rr, yy2-rr}, 0.5, Colors::darkgray);
		}
		for (int y = 0; y < mHeight; y+=sectorSize)
		{
			float xx1, yy1, rr, xx2, yy2;
			xyLoc l1(0, y), l2(mWidth, y);
			GetCoordinate(l1, xx1, yy1, rr);
			GetCoordinate(l2, xx2, yy2, rr);
			display.DrawLine({xx1-rr, yy1-rr}, {xx2-rr, yy2-rr}, 0.5, Colors::darkgray);
		}
		for (int s = 0; s < GetNumSectors(); s++)
		{
			const SectorData<sectorSize> &d = sectors[s];
			for (auto i : d.regionCenters)
			{
				float x, y, r;
				GetCoordinate(i, x, y, r);
				display.FillCircle({x, y}, r, Colors::gray);
			}
			for (auto e : d.edges)
			{
				xyLoc l1 = sectors[e.sectorFrom].regionCenters[e.regionFrom];
				xyLoc l2 = sectors[e.sectorTo].regionCenters[e.regionTo];
				
				float xx1, yy1, rr, xx2, yy2;
				GetCoordinate(l1, xx1, yy1, rr);
				GetCoordinate(l2, xx2, yy2, rr);
				display.DrawLine({xx1, yy1}, {xx2, yy2}, 0.5, Colors::green);
			}
		}
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::Draw(Graphics::Display &display, const xyLoc &l) const
	{
		float x, y, r;
		GetCoordinate(l, x, y, r);
		switch (GetTerrainType(l))
		{
			case kImpassable: display.FillSquare({x, y}, r, Colors::black); break;
			case kGround: display.FillSquare({x, y}, r, Colors::brown); break;
			case kTrees:  display.FillSquare({x, y}, r, Colors::darkgreen); break;
			case kSwamp:  display.FillSquare({x, y}, r, Colors::cyan); break;
			case kRoad:  display.FillSquare({x, y}, r, Colors::yellow); break;
			case kWater:  display.FillSquare({x, y}, r, Colors::blue); break;
			default: break;
		}
	}
	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetCoordinate(const xyLoc &l, float &x, float &y, float &radius) const
	{
		float _scale, xOffset, yOffset;
		if (mHeight > mWidth)
		{
			_scale = 2.0/(float)(mHeight);
			xOffset = (2.0-mWidth*_scale)*0.5;
			yOffset = 0;
		}
		else {
			_scale = 2.0/(float)(mWidth);
			yOffset = (2.0-mHeight*_scale)*0.5;
			xOffset = 0;
		}
		float epsilon = _scale/2.0;
		x = -1+l.x*_scale+epsilon+xOffset;
		y = -1+l.y*_scale+epsilon+yOffset;
		//	x = (2*_x-width)*_scale+epsilon;
		//	y = (2*_y-height)*_scale+epsilon;
		radius = epsilon;
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetPointFromCoordinate(point3d loc, int &px, int &py) const
	{
		double _x, _y;
		double _scale, xOffset, yOffset;
		if (mHeight > mWidth)
		{
			_scale = 2.0/(double)(mHeight);
			xOffset = (2.0-mWidth*_scale)*0.5;
			yOffset = 0;
		}
		else {
			_scale = 2.0/(double)(mWidth);
			yOffset = (2.0-mHeight*_scale)*0.5;
			xOffset = 0;
		}
		double epsilon = _scale/2.0;
		
		_x = (loc.x-epsilon+1-xOffset)/_scale;
		_y = (loc.y-epsilon+1-yOffset)/_scale;
		
		px = (int)(_x+0.5); // round off!
		py = (int)(_y+0.5);
		if ((px < 0) || (py < 0) || (px >= mWidth) || (py >= mHeight))
		{
			px = py = -1;
		}
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::BuildAbstraction()
	{
		for (int x = 0; x < GetNumSectors(); x++)
		{
			GetRegions(x);
		}
		for (int x = 0; x < GetNumSectors(); x++)
		{
			GetEdges(x);
		}
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetRegions(int sector)
	{
		SectorData<sectorSize> &d = sectors[sector];
		d.regionCenters.clear();
		int regionOffsetX = (sector%GetNumYSectors())*sectorSize;
		int regionOffsetY = (sector/GetNumYSectors())*sectorSize;

		for (int x = 0; x < sectorSize; x++)
		{
			for (int y = 0; y < sectorSize; y++)
			{
				d.region[x][y] = kNoRegion;
			}
		}
		int whichRegion = 0;
		for (int x = 0; x < sectorSize; x++)
		{
			for (int y = 0; y < sectorSize; y++)
			{
				if (d.region[x][y] == kNoRegion)
				{
					d.regionCenters.push_back(BFS(d, x, y, whichRegion));
					d.regionCenters.back().x += regionOffsetX;
					d.regionCenters.back().y += regionOffsetY;
					whichRegion++;
				}
			}
		}
	}

	template <int sectorSize>
	xyLoc DynamicWeightedGrid<sectorSize>::BFS(SectorData<sectorSize> &d, int x, int y, int whichRegion)
	{
		int avgx = 0, avgy = 0;
		float count = 0;
		std::vector<std::pair<int, int>> stack;
		stack.push_back({x, y});
		while (stack.size() > 0)
		{
			auto i = stack.back();
			stack.pop_back();
			if (d.region[i.first][i.second] == kNoRegion)
			{
				d.region[i.first][i.second] = whichRegion;
				count++;
				avgx += i.first;
				avgy += i.second;
				if (i.first+1 < sectorSize && (d.cells[i.first][i.second] == d.cells[i.first+1][i.second]))
					stack.push_back({i.first+1, i.second});
				if (i.first > 0 && (d.cells[i.first][i.second] == d.cells[i.first-1][i.second]))
					stack.push_back({i.first-1, i.second});
				if (i.second+1 < sectorSize && (d.cells[i.first][i.second] == d.cells[i.first][i.second+1]))
					stack.push_back({i.first, i.second+1});
				if (i.second > 0 && (d.cells[i.first][i.second] == d.cells[i.first][i.second-1]))
					stack.push_back({i.first, i.second-1});
			}
		}
		
		return xyLoc(avgx/count, avgy/count);
	}

	
	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::GetEdges(int sector)
	{
		if (sector < 0 || sector > GetNumSectors())
			return;
		SectorData<sectorSize> &d = sectors[sector];
		d.edges.clear();
		
		int regionOffsetX = (sector%GetNumYSectors())*sectorSize;
		int regionOffsetY = (sector/GetNumYSectors())*sectorSize;
		
		for (int x = 0; x < sectorSize; x++)
		{
			for (int y = 0; y < sectorSize; y++)
			{
				xyLoc middle(regionOffsetX+x, regionOffsetY+y);
				int r = GetRegion(middle);
				int s = GetSector(middle);
				
				for (int x1 = -1; x1 <= 1; x1++)
				{
					for (int y1 = -1; y1 <= 1; y1++)
					{
						xyLoc next(middle.x+x1, middle.y+y1);
						if (next.x >= mWidth || next.y >= mHeight)
							continue;
						int r1 = GetRegion(next);
						int s1 = GetSector(next);
						if (r1 != r || s1 != s)
						{
							AddEdge(d, s, r, s1, r1);
						}
					}
				}
			}
		}
	}

	template <int sectorSize>
	void DynamicWeightedGrid<sectorSize>::AddEdge(SectorData<sectorSize> &d, int s1, int r1, int s2, int r2)
	{
		for (auto &e : d.edges)
		{
			if (e.sectorFrom == s1 && e.sectorTo == s2 && e.regionFrom == r1 && e.regionTo == r2)
				return;
		}
		edge e = {s1, r1, s2, r2};
		d.edges.push_back(e);
	}

	
}
#endif /* DynamicWeightedGrid_h */
