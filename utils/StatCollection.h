/*
 * $Id: StatCollection.h,v 1.15 2006/09/18 06:20:15 nathanst Exp $
 *
 *  StatCollection.h
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
#include <string>

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
* The StatCollection class is for collecting stats across different parts of
 * the simulation. This class aggregates results and allows access to the
 * collected information.
 */ 

class StatCollection {
public:
	StatCollection();
	~StatCollection();
	void AddStat(const char *category, const char *owner, double value);
	void AddStat(const char *category, const char *owner, long value);
	void SumStat(const char *category, const char *owner, double value);
	void SumStat(const char *category, const char *owner, long value);
	
	void ClearAllStats();
	//	void clearOwnerStats(const char *owner); // not define for now; can be defined if needed
	//	void clearCategoryStats(const char *category);
	
	int GetNumStats() const;
	const stat *GetStatNum(int which) const;
	
	int LookupCategory(const char *category) const;
	int LookupOwner(const char *owner) const;
	const char *lookupCategoryID(int id) const;
	const char *LookupOwnerID(int id) const;
	bool LookupStat(const char *category, const char *owner, statValue &) const;
	bool LookupStat(unsigned int index, statValue &) const;
	
	void AddFilter(const char *category); // this is an include filter
	void AddIncludeFilter(const char *category); // include only added categories
	void AddExcludeFilter(const char *category); // exclude only added categories
	void ClearFilters();
	void EnablePrintOutput(bool pO) { printOutput = pO; }
	
	int FindNextStat(const char *category, const char *owner, int startIndex=0) const;
	int FindPrevStat(const char *category, const char *owner, int startIndex=-1) const;
	int FindNextStat(const char *what, bool findCategory, int startIndex = 0) const;
	int FindPrevStat(const char *what, bool findCategory, int startIndex = -1) const;
	
	void PrintStatsTable() const;
	
private:
		int addCategory(const char *category);
	int addOwner(const char *owner);
	bool passFilter(const char *category) const;
	statValue *getLastStat(const char *category, const char *owner);
	
	std::vector<std::string> categories;
	std::vector<std::string> owners;
	std::vector<std::string> includeFilters;
	std::vector<std::string> excludeFilters;
	std::vector<stat> stats;
	bool printOutput;
};

#endif
