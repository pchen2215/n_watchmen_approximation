#pragma once

#include "include.h"

// ================================================================================================

// Distance squared between two points
inline Kernel::FT dist2(const Point& p1, const Point& p2) {
    return CGAL::squared_distance(p1, p2);
}

// ================================================================================================

// Checks if the line segment from pt1 to pt2 crosses (collinear/touching is fine) with the polygon
inline bool crosses(const Point& pt1, const Point& pt2, const Polygon& p) {
    Kernel::Segment_2 seg(pt1, pt2);
    for (auto it = p.edges_begin(); it != p.edges_end(); it++) {
        auto res = CGAL::intersection(*it, seg);
        if (res) {
            if (const Point* p = std::get_if<Point>(&*res)) {
                if (*p == pt1 || *p == pt2 || *p == it->source() || *p == it->target()) {
                    continue;
                } else return true;
            }
        }
    }
    return false;
}

// ================================================================================================

// Finds the total area of the polygons
inline double total_area(const std::vector<Polygon>& unseen) {
    double count = 0;
    for (const Polygon& poly: unseen) {
        count += CGAL::to_double(poly.area());
    }
    return count;
}

// ================================================================================================
