/*
 * hpaStar.h
 *
 * Created by Renee Jansen on 05/16/06
 *
 */

#ifndef HPASTAR_H
#define HPASTAR_H

#include "searchAlgorithm.h"
#include "clusterAbstraction.h"

/** 
 * HPA* algorithm as described in (Botea,Mueller,Schaeffer 2004). 
 *
 * Needs a clusterAbstraction to run - use setAbstraction before 
 * doing any pathfinding
 */

class hpaStar : public searchAlgorithm {
public:
	
	// From HPA* original code - smoothwizard.h
	typedef enum {
		NORTH,
		EAST,
		SOUTH,
		WEST,
		NE,
		SE,
		SW,
		NW} Direction;

	typedef enum {
		BEGIN,
		END,
		TWO_BACK} SmoothType;

	hpaStar();
	virtual ~hpaStar(){}

	virtual path* getPath(graphAbstraction *aMap, node* from, node* to, reservationProvider *rp = 0);

	virtual const char *getName() { return algName; } 
	void setUpSearch(node* from, node* to);
	path* findAbstractPath(node* from, node* to);
	path* findMapPath(path* abPath,node* from,node* to);
	void cleanUpSearch(); 
	path* smoothPath(path* p);
	void setSmoothing(bool smooth);
	void setPartialPathLimit(int limit) { partialLimit = limit; 
		sprintf(algName,"HPA*(%d)",partialLimit); }
	int getPartialPathLimit() { return partialLimit; }
	/**
	 * Set the smoothing type.
	 *
	 * Whenever we update the path with a better subpath, 
	 * if s = BEGIN, smoothing is restarted from the beginning of this subpath
	 * if s = END, smoothing is restarted from the end of the new subpath
	 * if s = TWO_BACK, we go back two from the end of the new subpath
	 */
	void setSmoothType(SmoothType s) { smType = s; }

	// Ensure that the abstraction is a clusterAbstraction
	void setAbstraction(clusterAbstraction* _m) { m = _m; } 

protected:
	char algName[30];
	clusterAbstraction* m;
	int partialLimit;
	node* fromnum;
	node* tonum;
	std::vector<node*> lookup;
	path* nextPathNode(node* n, int dir);
	node* getNextNode(int x, int y, int dir);
	bool smoothing;
	SmoothType smType;
	bool nextInLookup(int last, int curr, std::vector<node*> lookup);
	int backTwoNodes(int i, std::vector<node*> lookup);
	void findMinMax(path* p);
	int minx;
	int maxx;
	int miny;
	int maxy;
};

#endif
