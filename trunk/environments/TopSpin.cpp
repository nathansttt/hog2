/*
 *  TopSpin.cpp
 *  hog2
 *
 *  Created by Nathan Sturtevant on 4/1/08.
 *  Copyright 2008 __MyCompanyName__. All rights reserved.
 *
 */

#include "TopSpin.h"

int TopSpinGraphHeuristic::THmode = 0;

TopSpin::TopSpin(int m, int k, TopSpinGraphHeuristic *tsh)
:GraphEnvironment(new Graph(), tsh)
{
	length = m;
	flipSize = k;
	directed = false;
	
	std::vector<int> config(m);
	for (unsigned int x = 0; x < config.size(); x++)
		config[x] = x;
	GetState(config);
}

TopSpin::TopSpin(int m, int k)
:GraphEnvironment(new Graph(), new TopSpinGraphHeuristic())
{
	length = m;
	flipSize = k;
	directed = false;
	
	std::vector<int> config(m);
	for (unsigned int x = 0; x < config.size(); x++)
		config[x] = x;
	GetState(config);
}

TopSpin::~TopSpin()
{
}

void TopSpin::GetSuccessors(graphState &stateID, std::vector<graphState> &neighbors)
{
	if (!data[stateID].expanded)
	{
		// add neighbors to graph
		ExpandNode(stateID);
	}
	return GraphEnvironment::GetSuccessors(stateID, neighbors);
}

void TopSpin::GetActions(graphState &stateID, std::vector<graphMove> &actions)
{
	if (!data[stateID].expanded)
	{
		// add neighbors to graph
		ExpandNode(stateID);
	}
	return GraphEnvironment::GetActions(stateID, actions);
}

bool TopSpin::GoalTest(graphState &state, graphState &goal)
{
	return (state == goal);
}

std::vector<int> &TopSpin::GetState(graphState gs)
{
	return data[gs].config;
}

graphState TopSpin::GetState(std::vector<int> &configuration)
{
	int zeroLoc = -1;
	for (unsigned int x = 0; x < configuration.size(); x++)
	{
		if (configuration[x] == 0)
		{
			zeroLoc = x;
			break;
		}
	}
	return GetState(configuration, zeroLoc);
}

graphState TopSpin::GetState(std::vector<int> &configuration, int zeroLoc)
{
	static std::vector<int> config(configuration.size());
	config.resize(configuration.size());
	
	assert(zeroLoc != -1);
	for (unsigned int x = 0; x < config.size(); x++)
		config[x] = configuration[(zeroLoc+x)%configuration.size()];
	
	uint64_t hash = GetStateHash(config);
	if (hashTable.find(hash) != hashTable.end())
		return hashTable[hash];
	node *n;
	g->AddNode(n = new node(""));
	data.resize(n->GetNum()+1);
	data[n->GetNum()].config = config;
	data[n->GetNum()].hashKey = hash;
	data[n->GetNum()].expanded = false;
	hashTable[hash] = n->GetNum();
	//printf("Added item %lld; hash table size is now %d\n", hash, hashTable.size());
	return n->GetNum();
}

void TopSpin::ExpandNode(graphState &stateID)
{
	//printf("Expanding %lu\n", stateID);
	for (int x = 0; x < length; x++)
	{
		Flip(data[stateID].config, x, flipSize);
		// will create the state if it doesn't exist already!
		graphState s = GetState(data[stateID].config);
		Flip(data[stateID].config, x, flipSize);
		if (!g->FindEdge(s, stateID))
			g->AddEdge(new edge(s, stateID, 1));
	}
	data[stateID].expanded = true;
}

void TopSpin::Flip(std::vector<int> &arrangement, int index, int radius)
{
	while (radius > 1)
	{
		int tmp = arrangement[index];
		int otherSide = (index+radius-1)%arrangement.size();
		arrangement[index] = arrangement[otherSide];
		arrangement[otherSide] = tmp;
		radius-=2;
		index = (index+1)%arrangement.size();
	}
}

uint64_t TopSpin::GetStateHash(graphState &state)
{
//	if (!data[state].expanded)
//		ExpandNode(state);
	return data[state].hashKey;
}

graphState TopSpin::Dual(graphState s)
{
	if( s == 0) return 0;

	std::vector<int> cfg = data[s].config;
	std::vector<int> dualCfg = cfg;
	for(unsigned int x=0;x<cfg.size();x++) {
		dualCfg[cfg[x]] = x;
	}

	// normalize 0 to start
	//int zeroLoc = -1;
	//for (unsigned int x = 0; x < dualCfg.size(); x++)
	//{
	//	if (dualCfg[x] == 0)
	//	{
	//		zeroLoc = x;
	//		break;
	//	}
	//}
	//assert(zeroLoc != -1);
	//std::vector<int> dualCfg2(dualCfg.size());
	//for (unsigned int x = 0; x < dualCfg.size(); x++)
	//	cfg[x] = dualCfg[(zeroLoc+x)%dualCfg.size()];

	return GetState(dualCfg);  // it seems that we can't avoid storing the dual (for now) !
}

uint64_t TopSpin::GetStateHash(std::vector<int> &config)
{
	return GetStateHash(&config[0], config.size());
}

uint64_t TopSpin::GetStateHash(int *config, int config_size)
{
	// need to handle rotation!
	
//	static pdb_rank_t pdb_index( const int8_t *pdbpos, const int num_tiles,
//								const int pdb_size )
//	{
	uint64_t rank;
	int i, end;
	int MAX_TILES = config_size;
	int num_tiles = config_size;
	int8_t tiles[ MAX_TILES ], loc[ MAX_TILES ];
	
	//memset( loc, pdb_size, num_tiles );
	i = 0;
	do {
		loc[ tiles[ i ] = config[ i ] ] = i; 
	} while( ++i != num_tiles );
	
	rank = tiles[ i = 0 ];
	end = num_tiles - 1;
	while( i != num_tiles - 1 )
	{
		tiles[ loc[ end ] ] = tiles[ i ];
		loc[ tiles[ i ] ] = loc[ end ];
		i++; end--;
		
		rank = rank * ( num_tiles - i ) + tiles[ i ];
	}
	
	return rank;
//	}
}

uint64_t TopSpin::GetPDBHash(graphState &state, int pdb_size)
{
	return GetPDBHash(GetState(state), pdb_size);
}

uint64_t TopSpin::GetPDBHash(std::vector<int> &config, int pdb_size)
{
	std::vector<int> pdb_pos(config.size());
	for (unsigned int x = 0; x < config.size(); x++)
		pdb_pos[config[x]] = x;
	
	int MAX_TILES = config.size();
	uint64_t rank;
	int num_tiles = config.size();
	int i, end;
	int8_t tiles[ MAX_TILES ], loc[ MAX_TILES ];
	
	memset( loc, pdb_size, num_tiles );
	i = 0;
	do {
		//loc[ tiles[ i ] = config[ i ] ] = i;
		loc[ tiles[ i ] = pdb_pos[ i ] ] = i;
	} while( ++i != pdb_size );
	
	rank = tiles[ i = 0 ];
	end = num_tiles - 1;
	while( i != pdb_size - 1 ) {
		tiles[ loc[ end ] ] = tiles[ i ];
		loc[ tiles[ i ] ] = loc[ end ];
		i++; end--;
		
		rank = rank * ( num_tiles - i ) + tiles[ i ];
	}
	
	return rank;
}

uint64_t TopSpin::GetPDBSize(int puzzleSize, int pdb_size)
{
	pdb_size--;
	uint64_t size;
	int i;
	
	size = 1;
	for( i = puzzleSize - pdb_size; i < puzzleSize; i++ ) {
		size *= i;
	}
	return size;
}

TopSpinGraphHeuristic::TopSpinGraphHeuristic()
:ts(0), pdb(-1)
{
}

TopSpinGraphHeuristic::TopSpinGraphHeuristic(int psize, int spin, int pdbStates)
:ts(0), pdb(pdbStates)
{
	char filename[256];
	sprintf(filename, "TS_%d_%d_%d.db", psize, spin, pdb);
	FILE *f = fopen(filename, "r");
	if (!f)
	{
		printf("Can't load PDB!\n");
		exit(0);
	}
	
	DB.resize(ts->GetPDBSize(psize, pdb));
	fread(&(DB[0]), sizeof(uint8_t), ts->GetPDBSize(psize, pdb), f);
	fclose(f);
}


double TopSpinGraphHeuristic::HCost(graphState &s1, graphState &s2)
{
	graphState sd;

	if (ts == 0)
		return 0;
	if ((s1 != 0) && (s2 != 0))
	{
		printf("Error -- one state must be canonical goal state\n");
		exit(0);
	}
	if (s1 == 0)
	{
		//return DB[ts->GetPDBHash(s2, pdb)];
		if(THmode == 0)
			sd = s2;
		else
			sd = ts->Dual(s2);

		//printf("Returning heur value!: %d\n", DB[ts->GetPDBHash(sd, pdb)]);
		return DB[ts->GetPDBHash(sd, pdb)];
	}
	//	printf("Returning heur value!: %d\n", DB[ts->GetPDBHash(s1, pdb)]);
	//return DB[ts->GetPDBHash(s1, pdb)];
	if(THmode == 0)
		sd = s1;
	else
		sd = ts->Dual(s1);
	//printf("Returning heur value!: %d\n", DB[ts->GetPDBHash(sd, pdb)]);
	return DB[ts->GetPDBHash(sd, pdb)];
}


//void switch_to_dual( const game_t *game, state_t state )
//{
//#if 1
//	int i;
//	int8_t t;
//	
//	for( i = 0; i != game->num_tiles; i++ ) {
//		t = state[ i ];
//		state[ i ] = state[ i + game->num_tiles ];
//		state[ i + game->num_tiles ] = t;
//	}
//#else
