/*
 * hpaStar.cpp
 * 
 * Created by Renee Jansen on 06/16/06
 *
 */

#include "hpaStar.h"
#include "aStar3.h"

#include "aStar.h"
#include "timer.h"

const static int verbose = 0;
const int MAX_INT = 2147483647;

hpaStar::hpaStar()
{
	partialLimit = -1;
	fromnum = 0; 
	tonum = 0;
	smoothing = true;
	smType = END;

	sprintf(algName,"HPA*(%d)",partialLimit);
}

/**
 * Set whether we want to do path smoothing. Default is true. 
 */
void hpaStar::setSmoothing(bool smooth){
	smoothing = smooth;
}

/**
 * Returns the HPA* path between from and to. 
 * 
 * Before calling this function, set the abstraction with hpaStar::setAbstraction. The abstraction
 * must be of type clusterAbstraction.
 * 
 * Reservation provider isn't used 
 */
path* hpaStar::getPath(graphAbstraction *aMap, node* from, node* to, reservationProvider *rp)
{
	// Make sure a cluster abstraction has been set & it's the same as the abstraction being passed
	// in to this function
	assert(m==aMap);

	if((from==to) || (from ==0) || (to==0)){
		if(verbose) std::cout<<"Returning an empty path\n";
		return 0;
	}

	fromnum = 0; 
	tonum = 0;

	Timer t;
	//if the nodes are in the same cluster, just do A*
	//this takes care of the case where both nodes are in some enclosed
	//area inside a cluster

	if(m->getClusterIdFromNode(from)==m->getClusterIdFromNode(to))
	{
		aStarOld astar;
		path* p = astar.getPath(m,from,to);
		nodesExpanded = astar.getNodesExpanded();
		nodesTouched = astar.getNodesTouched();
		if(verbose)std::cout<<"Nodes are inside the same cluster\n";
		return p;
	}

	setUpSearch(from, to);

	// Check if there's a path at all
	if(!m->pathable(from,to)){
		if(verbose) std::cout<<"Not pathable\n";
		cleanUpSearch();
		return 0;
 	}

 	else
		if(verbose) std::cout<<"Pathable\n";

	//	t.startTimer();
	path* abPath = findAbstractPath(fromnum,tonum);
	//std::cout<<"abstract path "<<t.endTimer()<<std::endl;

	if(!abPath){
		cleanUpSearch();
		return 0;
	}
	path* mapPath = 0; 
	if(partialLimit > 0){
		path *trav = abPath;
		path *thisPart = new path(trav->n);
		for (int x = 0; x < partialLimit-1; x++){
			if (trav->next) {
				trav = trav->next;
				thisPart->tail()->next = new path(trav->n);
			}
		}
		

		node* upper = trav->tail()->n;
		point3d s(upper->getLabelF(kXCoordinate),upper->getLabelF(kYCoordinate),-1);
		int px;
		int py;
		m->getMap()->getPointFromCoordinate(s,px,py);

		node* newTo = m->getNodeFromMap(px,py);
		mapPath = findMapPath(thisPart, from, newTo);
		delete thisPart;
	}
	else
	{
		//	t.startTimer();
		mapPath = findMapPath(abPath,from,to);
		//std::cout<<"Found map path "<<t.endTimer()<<std::endl;
	}
	
	path* finalPath = 0; 

	if (smoothing)
	{
		//t.startTimer();
		finalPath = smoothPath(mapPath);
		//std::cout<<"Smoothing: "<<t.endTimer()<<std::endl<<std::endl;
		delete mapPath;
	}
	else {
		finalPath = mapPath;
		mapPath = 0;
	}

	cleanUpSearch();
	
	delete abPath;

	return finalPath; 
}

/*
 * setUpSearch sets the number of nodes expanded and touched to 0
 * and inserts the start and goal nodes into the abstract graph
 */ 
void hpaStar::setUpSearch(node* from, node* to)
{
	if(verbose) std::cout<<"HPA*: setting up search\n";
	// set nodes expanded & nodes touched to 0
	nodesExpanded = 0; 
	nodesTouched = 0;

	int touched =0;
	int expanded = 0; 
	// insert the nodes 
	//	Timer t; 
	//	t.startTimer();
	fromnum = m->insertNode(from,expanded,touched);
	nodesExpanded += expanded;
	nodesTouched += touched;
	tonum = m->insertNode(to,expanded,touched);
	nodesExpanded += expanded;
	nodesTouched += touched;
	//	std::cout<<"Time taken to insert: "<<t.endTimer();
}

/*
 * finds a path on the abstract level of a map
 */
path* hpaStar::findAbstractPath(node* from, node* to)
{
	if(verbose)	std::cout<<"HPA*: finding the abstract path\n";
	aStarOld astar;

	path* p = astar.getPath(m,fromnum, tonum); 
	nodesExpanded = astar.getNodesExpanded();
	nodesTouched = astar.getNodesTouched();

	if(verbose){
		std::cout<<"Abstract path: ";
		if(p)
			p->print();
		else
			std::cout<<"Null";
		std::cout<<std::endl;
	}

	return (p);
}

/* 
 * finds the map-level path corresponding to the abstract path
 * It uses cached paths for each edge in the abstract path. 
 */ 
path* hpaStar::findMapPath(path* abPath,node* from,node* to)
{
	graph *g = m->getAbstractGraph(1);
	path* curr = abPath;

	path* returnme = 0;

	node* n = curr->n;
	curr = curr->next;
	node* n2 = curr->n;

	edge* e = g->findEdge(n->getNum(), n2->getNum());

	path* lowlevel = m->getCachedPath(e);

	if (lowlevel)
	{ // edge was cached
		if ((lowlevel->n == m->getLowLevelNode(n2)) &&
			 (lowlevel->tail()->n == m->getLowLevelNode(n)))
		{
			// The cached path is backwards - have to reverse it
			lowlevel = lowlevel->reverse();
		}

		returnme = lowlevel;
	}
	else {
		returnme = new path(m->getLowLevelNode(n), new path(m->getLowLevelNode(n2)));
	}

	while(curr->next){
		n = n2;
		curr = curr->next;
		n2 = curr->n;

		/*		
		point3d s(n->getLabelF(kXCoordinate),n->getLabelF(kYCoordinate),-1);
		int px;
		int py;
		m->getMap()->getPointFromCoordinate(s,px,py);

		point3d t(n2->getLabelF(kXCoordinate),n2->getLabelF(kYCoordinate),-1);
		
		m->getMap()->getPointFromCoordinate(t,px,py);
		*/

		e = g->findEdge(n->getNum(), n2->getNum());

		lowlevel = m->getCachedPath(e);
		
		
		if(lowlevel)
		{

			if ((lowlevel->n == m->getLowLevelNode(n2)) &&
					(lowlevel->tail()->n == m->getLowLevelNode(n)))
			{
				lowlevel = lowlevel->reverse();
			}
			
			path *tailend = returnme->tail();
			if(tailend->n == lowlevel->n)
			{
				tailend->next = lowlevel->next;
				lowlevel->next = 0;
				delete lowlevel;
			}
			else{
				returnme->tail()->next = lowlevel;
			}			
		}
		else
		{
			returnme->tail()->next = new path(m->getLowLevelNode(n2));
		}
	}
	return returnme;
}

/**
 * Remove the start & goal nodes from the graph
 */
void hpaStar::cleanUpSearch()
{
	// remove nodes from map
	if((fromnum!=0) && (tonum != 0)){
		//		Timer t;
		//t.startTimer();
		m->removeNodes(fromnum,tonum);
		//std::cout<<"It took "<<t.endTimer()<<" to remove nodes\n";
	}
	fromnum = 0;
	tonum = 0;
}

/**
 * from HPA* smoothwizard.cpp
 *
 * smoothen the path by replacing parts of the path by 
 * straight lines.
 */
path* hpaStar::smoothPath(path* p)
{
	if(verbose) std::cout<<"Smoothing the path\n";

	//	findMinMax(p);
	int clusterSize = m->getClusterSize();

	// put the path nodes in a vector
	lookup.clear();
	int i=0; 
	//assert(pcopy);
	for (path *tmp = p; tmp != 0; tmp = tmp->next)
	{
		lookup.push_back(tmp->n);		
		// set key=index in lookup
		tmp->n->setLabelL(kTemporaryLabel,i);
		i++;
	}

	unsigned int n = 0; 
	path* smooth = 0;

	while(true){
		//Skip blanks
		while(lookup[n]==0 && n<lookup.size()-1)
			n++;

		if(n>=lookup.size()-1){ 
			break;
		}

		unsigned int last = lookup[n]->getLabelL(kTemporaryLabel);

		// Node's temporary label != last --> this node occurs more than
		// once. Cut out the in between path & go back to beginning of while loop
		if (last!=n)
		{
			std::cout<<"in here\n";
 			for(unsigned int i=n; i<last && i<lookup.size(); i++)
			{
 				lookup[i]=0;
			}
			n = last;
			continue;
		}

		int currX = lookup[n]->getLabelL(kFirstData);
		int currY = lookup[n]->getLabelL(kFirstData+1); 
		minx = currX - clusterSize;
		maxx = currX + clusterSize;																		 
		miny = currY - clusterSize;
		maxy = currY + clusterSize;

		int dir;
		for(dir = NORTH; dir <= NW; dir++)
		{
			// get a shortcut if it exists
			path* pathToNode = nextPathNode(lookup[n],dir);

			// paste the shortcut into our current path
			if (pathToNode)
			{
				int last = pathToNode->tail()->n->getLabelL(kTemporaryLabel);
				int curr = pathToNode->n->getLabelL(kTemporaryLabel);
				if (last > curr && !nextInLookup(last, curr,lookup))
				{
					// make sure it's not the next one					
					unsigned int index = n;
					path* pathCopy = pathToNode;
					//path* backup = pathCopy;
					unsigned int end = pathToNode->tail()->n->getLabelL(kTemporaryLabel);

					while(pathCopy->next){
						//make sure we're not overwriting anything
						assert(index <= end);

						lookup[index]=pathCopy->n;
						pathCopy->n->setLabelL(kTemporaryLabel,index);
						pathCopy = pathCopy->next;
						index++;
					}
					assert(index <= end);

					lookup[index]=pathCopy->n;
					pathCopy->n->setLabelL(kTemporaryLabel,index);
					
					index++;	

					while(index<=end){
						lookup[index]= 0;
						index++;
					}

					if(smType==END){
						n = end;
					}
					else if(smType==TWO_BACK)
						n = backTwoNodes(end,lookup);
					else		
						n++; 
					
					delete pathToNode; pathToNode = 0;
					//delete backup;
					break;

				}
				else if(dir==NW){
					n++;
				}
				delete pathToNode;
			} 
			else if(dir==NW){
				n++;
			}
		} //end for every direction


	}

	//Create smoothed path from lookup table
	for(unsigned int i=0; i<lookup.size(); i++){
		if(lookup[i]!=0){
			if(!smooth)
				smooth = new path(lookup[i],0);
			else
				smooth->tail()->next = new path(lookup[i],0);
		}
	}

	return smooth;
}

/**
 * Find the index of the node two nodes back in the path
 */
int hpaStar::backTwoNodes(int i, std::vector<node*> lookup)
{
	i--;
	while(lookup[i]==NULL){
		i--;
	}
	i--;
	while(lookup[i]==NULL){
		i--;
	}
	return i;
}


/**
 * find out whether last is the next 'real' index in the lookup table after
 * curr. This is to make sure we don't keep replacing little paths due to null's in
 * the lookup table. 
 */
bool hpaStar::nextInLookup(int last, int curr, std::vector<node*> lookup)
{
	if(last<curr) 
		return false;
	
	for(int i=curr+1; i<=last; i++)	//
	{
		if(i==last)
			return true;
		if(lookup[i]!=NULL)
			return false;
	}
	return true;	
}

/**
 * shoot a ray in direction dir and see if you hit the path
 * Return the better path if you find it; 0 if you hit a  
 * wall or obstacle (ie. if you won't find the path) 
 */ 
path* hpaStar::nextPathNode(node* n, int dir)
{
	graph *g = m->getAbstractGraph(0);

	int px = n->getLabelL(kFirstData);
	int py = n->getLabelL(kFirstData+1);
	
	node* next = getNextNode(px,py,dir);

	if(next){
		nodesTouched++;

		int nextKey = next->getLabelL(kTemporaryLabel);
		edge* e = g->findEdge(n->getNum(), next->getNum());
		
		if(e && (nextKey >= 0) && (nextKey < static_cast<int>(lookup.size())) && (lookup[nextKey]==next)){
			//we're done - we found the path
			return new path(n,new path(next));
		}
		else{	// look further for a path node		
			if(e){
				path * p = nextPathNode(next,dir);				 
				if(p){
					return new path(n,p);
				}
				else // haven't found a path node
					return 0;
			}
			else // no edge here
				return 0; 			
		}
	}
	else{ // we won't hit the path 
		return 0;
	}
}

/**
 * get the next node from map coordinates (x,y) in direction
 * dir. Will return 0 if there's no node here. 
 */
node* hpaStar::getNextNode(int x, int y, int dir)
{
 	if((x < minx) || (x > maxx) || (y < miny) || (y>maxy))
 		return 0;

	switch(dir)
	{
	case NORTH:
		return m->getNodeFromMap(x,y+1);
		break;
	case EAST:
		return m->getNodeFromMap(x+1,y);
		break;
	case SOUTH:
		return m->getNodeFromMap(x, y-1);
		break;
	case WEST:
		return m->getNodeFromMap(x-1, y);
		break;
	case NE:
		return m->getNodeFromMap(x+1, y+1);
		break;
	case SE:
		return m->getNodeFromMap(x+1, y-1);
		break;
	case SW:
		return m->getNodeFromMap(x-1, y-1);
		break;
	case NW:
		return m->getNodeFromMap(x-1, y+1);
		break;
	default:
		return 0;
	}
}

void hpaStar::findMinMax(path* p)
{
	minx = MAX_INT;
	miny = MAX_INT;
	maxx = -1;
	maxy = -1;
	
	path* pcopy = p;
	
	while(pcopy->next)
	{
		int x = pcopy->n->getLabelL(kFirstData);
		int y = pcopy->n->getLabelL(kFirstData+1);

		if(x < minx) minx = x;
		if(x > maxx) maxx = x;
		
		if(y < miny) miny = y;
		if(y > maxy) maxy = y;

		pcopy = pcopy->next;
	}
	
	int x = pcopy->n->getLabelL(kFirstData);
	int y = pcopy->n->getLabelL(kFirstData+1);

	if(x < minx) minx = x;
	if(x > maxx) maxx = x;
	
	if(y < miny) miny = y;
	if(y > maxy) maxy = y;
	
	assert((minx != MAX_INT)&&(miny != MAX_INT)&&(maxx != -1)&&(maxy != -1));

}
