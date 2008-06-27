
// These functions can be used when an unknown amount of "for"-loops
// has to be used (e.g. num_players many). Then you can encode one
// for loop and dehash the current value...
unsigned long hash_permutation( unsigned int num_players, unsigned int *num_player_states, unsigned int *player_states );
void dehash_permutation( unsigned long hash, unsigned int num_players, unsigned int *num_player_states, unsigned int *player_states );

