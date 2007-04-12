/*
 * $Id: statCollection.h,v 1.15 2006/09/18 06:20:15 nathanst Exp $
 *
 *  statCollection.h
 *  hog
 *
 *  Created by Nathan Sturtevant on 6/1/05.
 *  Copyright 2005 Nathan Sturtevant, University of Alberta. All rights reserved.
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
 *
 */

#include <vector>

#ifndef STATCOLLECTION_H
#define STATCOLLECTION_H

typedef union { double fval; long lval; } statValue;
enum storedType { floatStored, longStored };
class stat {
public:
	int category, owner;
	statValue value;
	storedType sType;
};

/**
* The statCollection class is for collecting stats across different parts of
 * the simulation. This class aggregates results and allows access to the
 * collected information.
 */ 

class statCollection {
public:
	statCollection();
	~statCollection();
	void addStat(const char *category, const char *owner, double value);
	void addStat(const char *category, const char *owner, long value);
	void sumStat(const char *category, const char *owner, double value);
	void sumStat(const char *category, const char *owner, long value);
	
	void clearAllStats();
	//	void clearOwnerStats(const char *owner); // not define for now; can be defined if needed
	//	void clearCategoryStats(const char *category);
	
	int getNumStats() const;
	const stat *getStatNum(int which) const;
	
	int lookupCategory(const char *category) const;
	int lookupOwner(const char *owner) const;
	const char *lookupCategoryID(int id) const;
	const char *lookupOwnerID(int id) const;
	bool lookupStat(const char *category, const char *owner, statValue &) const;
	bool lookupStat(unsigned int index, statValue &) const;
	
	void addFilter(char *category); // this is an include filter
	void addIncludeFilter(char *category); // include only added categories
	void addExcludeFilter(char *category); // exclude only added categories
	void clearFilters();
	void enablePrintOutput(bool pO) { printOutput = pO; }
	
	int findNextStat(const char *category, const char *owner, int startIndex=0) const;
	int findPrevStat(const char *category, const char *owner, int startIndex=-1) const;
	int findNextStat(const char *what, bool findCategory, int startIndex = 0) const;
	int findPrevStat(const char *what, bool findCategory, int startIndex = -1) const;
	
	void printStatsTable() const;
	
private:
		int addCategory(const char *category);
	int addOwner(const char *owner);
	bool passFilter(const char *category) const;
	statValue *getLastStat(const char *category, const char *owner);
	
	std::vector<const char *> categories;
	std::vector<const char *> owners;
	std::vector<const char *> includeFilters;
	std::vector<const char *> excludeFilters;
	std::vector<stat> stats;
	bool printOutput;
};

#endif
