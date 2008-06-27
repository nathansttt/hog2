#include "MyHash.h"

unsigned long hash_permutation( unsigned int num_players, unsigned int *num_player_states, unsigned int *player_states ) {
	unsigned long offset = 1, hash = 0;
	for( unsigned int i = 0; i < num_players; i++ ) {
		hash += player_states[i] * offset;
		offset *= num_player_states[i];
	}
	return hash;
}

void dehash_permutation( unsigned long hash, unsigned int num_players, unsigned int *num_player_states, unsigned int *player_states ) {
	unsigned long offset = 1;
	for( unsigned int i = 0; i < num_players; i++ ) {
		offset = num_player_states[i];
		player_states[i] = hash%offset;
		hash /= offset;
	}
}
