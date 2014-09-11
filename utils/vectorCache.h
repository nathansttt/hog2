/*
 *  vectorCache.h
 *  games
 *
 *  Created by Nathan Sturtevant on 8/25/10.
 *  Copyright 2010 NS Software. All rights reserved.
 *
 */

template<class storage>
class vectorCache {
public:
	vectorCache() { count = 0; }
	~vectorCache();
	std::vector<storage> *getItem();
	void returnItem(std::vector<storage> *);
private:
	std::vector<std::vector<storage> *> freeList;
	int count;
};


template<class storage>
vectorCache<storage>::~vectorCache<storage>()
{
	for (unsigned int x = 0; x < freeList.size(); x++)
		delete freeList[x];
	freeList.resize(0);
}

template<class storage>
std::vector<storage> *vectorCache<storage>::getItem()
{
	if (freeList.size() > 0)
	{
		std::vector<storage> *theItem = freeList.back();
		freeList.pop_back();
//		printf("CACHE: REALLOC: %p\n", theItem);
		return theItem;
	}
	else {
//		printf("%d items allocated\n", ++count);
		std::vector<storage> *newItem = new std::vector<storage>();
//		printf("CACHE: ALLOC: %p\n", newItem);
		return newItem;
//		theCache.resize(theCache.size()+1);
//		printf("CACHE: ALLOC: %p\n", &theCache[theCache.size()-1]);
//		return &theCache[theCache.size()-1];
	}
//	printf("CACHE: PTR: %p\n", (void*)0);
	return 0;
}

template<class storage>
void vectorCache<storage>::returnItem(std::vector<storage> *item)
{
//	printf("CACHE: FREE: %p\n", item);
	item->clear();
	freeList.push_back(item);
}

