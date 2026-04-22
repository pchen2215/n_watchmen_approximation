#pragma once
#include "include.h"

// Finds the visibility polygon from a point in the polygon
Polygon visibility(const Point& pt, const Polygon& polygon);

// Updates the unseen areas
void update_unseen(const Polygon& seen, std::vector<Polygon>& unseen);