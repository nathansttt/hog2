//
//  Transit.hpp
//  hog2 mac native demos
//
//  Created by Nathan Sturtevant on 5/20/18.
//  Copyright © 2018 NS Software. All rights reserved.
//

#ifndef Transit_hpp
#define Transit_hpp

#include <stdio.h>
#include "Heuristic.h"
#include "Map2DEnvironment.h"

class Transit : public Heuristic<xyLoc> {
public:
	virtual double HCost(const state &a, const state &b) const;
};

#endif /* Transit_hpp */
