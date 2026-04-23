#ifndef __SOLUTION_H
#define __SOLUTION_H
#include "include.h"

// ================================================================================================

// Solves by greedy construction following triangulation edges
void find_init_patrol_1(Patrol& patrol, const Polygon& polygon);

// Solves by greedily selecting vertex with best visibility polygon
void find_init_patrol_2(Patrol& patrol, const Polygon& polygon);

// ================================================================================================

// Reduces solution for n = 1 to n > 1
void reduce(std::vector<Patrol>& solution, const Polygon& polygon, int n);

// ================================================================================================
#endif
