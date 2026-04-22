#pragma once
#include "include.h"

// ================================================================================================

// Solves by greedy construction following triangulation edges
void find_init_patrol_1(Patrol& patrol, const Polygon& polygon);

// Solves by greedily selecting vertex with best visibility polygon
void find_init_patrol_2(Patrol& patrol, const Polygon& polygon);

// TBD
void find_init_patrol_3(Patrol& patrol, const Polygon& polygon);

// ================================================================================================

// Reduces by pruning longest edges
void reduce_1(std::vector<Patrol>& solution, const Polygon& polygon, int n);

// ================================================================================================
