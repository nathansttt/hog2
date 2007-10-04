/**
* A 2D map environment with edge costs weighted according to the 
* number of times a unit has passed over each edge. 
* 
* @file WeightedMap2DEnvironment.h
* @package hog2
* @brief A 2D map environment with edge costs weighted according to the number of times a unit has passed over each edge. 
* @author Renee Jansen
* @date 06/20/2007
*
* This file is part of HOG2.
* HOG : http://www.cs.ualberta.ca/~nathanst/hog.html
* HOG2: http://code.google.com/p/hog2/
*
* HOG2 is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; either version 2 of the License, or
* (at your option) any later version.
* 
* HOG2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with HOG2; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef WEIGHTEDMAP2DENVIRONMENT_H
#define WEIGHTEDMAP2DENVIRONMENT_H

#include "Map2DEnvironment.h" 
#include "BitVector.h"
#include "OccupancyInterface.h"
#include "Graph.h"
#include <ext/hash_map>
#include <cmath>

class Vector2D {
	public:
		Vector2D(float _x,float _y):x(_x), y(_y) {Normalize();}
		Vector2D():x(0),y(0) {}
		void Set(float _x, float _y) { x=_x; y=_y; Normalize();}
		bool operator==(const Vector2D &rhs) {return ((x == rhs.x)&&(y==rhs.y));}
		std::ostream& operator <<(std::ostream & out)
		{
			out << "(" << x << ", " << y << ")";
			return out;	
		}
		
		friend Vector2D operator *(const Vector2D& vec, const double num)
		{
			Vector2D returnme(vec.x * num, vec.y * num);
			return returnme;
		}
		
		friend Vector2D operator *(const double num, const Vector2D& vec)
		{
			Vector2D returnme(vec.x * num, vec.y * num);
			return returnme;
		}
		
		friend Vector2D operator +(const Vector2D& v1, const Vector2D& v2)
		{
			Vector2D returnme(v1.x + v2.x, v1.y + v2.y);
			returnme.Normalize();
			return returnme;
		}
	//private:
		float x, y;
		void Normalize()
		{
			if((x==0)&&(y==0))
				return;
			float magnitude = sqrt(x*x + y*y);
			x /= magnitude;
			y /= magnitude;
		}
};


namespace AngleUtil {

	class AngleSearchNode{
		public:
			AngleSearchNode(xyLoc &s,uint64_t key)
				:node(s), hashKey(key) {}
			xyLoc node;
			uint64_t hashKey;
	};

	struct SearchNodeEqual {
		bool operator()(const AngleSearchNode &i1, const AngleSearchNode &i2)
		{ return (i1.node == i2.node); } };
		
	struct SearchNodeHash {
		size_t operator()(const AngleSearchNode &x) const
		{ return (size_t)(x.hashKey); } };
	
	 typedef __gnu_cxx::hash_map<AngleUtil::AngleSearchNode,Vector2D, AngleUtil::SearchNodeHash, AngleUtil::SearchNodeEqual> AngleLookupTable;
};

/** Edge labels */
enum {
 	kForwardCount = 2, // The number of units passing from From to To
 	kBackwardCount = 3 // The number of units passing from To to From
};


class WeightedMap2DEnvironment : public AbsMapEnvironment 
{
public:
	WeightedMap2DEnvironment(MapAbstraction *ma);
	WeightedMap2DEnvironment(AbsMapEnvironment *ame);
	virtual ~WeightedMap2DEnvironment();
	void ApplyAction(xyLoc &s, tDirection dir);
	virtual double GCost(xyLoc &node1, xyLoc &node2);
	//virtual BaseMapOccupancyInterface* GetOccupancyInterface(){std::cout<<"Returning "<<oi<<std::endl;return oi;}
	virtual BaseMapOccupancyInterface* GetOccupancyInfo(){return oi;}
	void OpenGLDraw(int window);
	void OpenGLDraw(int window, xyLoc &l) { AbsMapEnvironment::OpenGLDraw(window, l); }
	void OpenGLDraw(int window, xyLoc &l, GLfloat r, GLfloat g, GLfloat b) {AbsMapEnvironment::OpenGLDraw(window,l,r,g,b);}
	void OpenGLDraw(int window, xyLoc& s, tDirection &dir);
	void OpenGLDraw(int window, xyLoc& s, tDirection &dir, GLfloat r, GLfloat g, GLfloat b) {AbsMapEnvironment::OpenGLDraw(window,s,dir,r,g,b);}
	void DrawEdge(int window, edge* e);
	
	void UpdateAngle(xyLoc &old, xyLoc &s);
	
	/** Set the cost of moving in the "wrong" direction */ 
	void SetWeight(double wt) { diffWeight = wt; }
	/** Set the weight (proportion) of the old angle */ 
	void SetProportionOld(double prop) { assert((prop>=0) && (prop <=1)); oldProportion = prop; }
private:
	BaseMapOccupancyInterface* oi;
	
	typedef __gnu_cxx::hash_map<AngleUtil::AngleSearchNode,Vector2D, AngleUtil::SearchNodeHash, AngleUtil::SearchNodeEqual> AngleLookupTable;
	AngleLookupTable angleLookup;
	
	double diffWeight;
	double oldProportion;
};

typedef UnitSimulation<xyLoc, tDirection, WeightedMap2DEnvironment> UnitWeightedMapSimulation;

// Weighting with node angles




#endif

