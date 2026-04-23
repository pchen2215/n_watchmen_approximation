#ifndef __UTILS_H
#define __UTILS_H

#include "include.h"

// ================================================================================================

// Distance squared between two points
inline Kernel::FT dist2(const Point& p1, const Point& p2) {
    return CGAL::squared_distance(p1, p2);
}

// ================================================================================================

// Checks if the line segment from pt1 to pt2 crosses the border of the polygon.
// Segments that are colinear with a polygon or touch one or more edges without crossing are fine.
inline bool crosses(const Point& pt1, const Point& pt2, const Polygon& poly) {
    Kernel::Segment_2 seg(pt1, pt2);

    for (auto it = poly.edges_begin(); it != poly.edges_end(); it++) {
        auto res = CGAL::intersection(*it, seg);
        if (res) {
            if (const Point* p = std::get_if<Point>(&*res)) {
                if (*p == pt1 || *p == pt2) {
                    continue;
                } else return crosses(pt1, *p, poly) || crosses(*p, pt2, poly);
            }
        }
    }

    return poly.bounded_side(CGAL::midpoint(seg)) == CGAL::ON_UNBOUNDED_SIDE;
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

inline PointGraph to_graph(const Patrol& p) {
    PointGraph graph;
    for (const PatrolEdge& pe: p) {
        if (pe.first == pe.second) {
            graph[pe.first];
            continue;
        }
        graph[pe.first].insert(pe.second);
        graph[pe.second].insert(pe.first);
    }
    return graph;
}

// ================================================================================================

inline void print_graph(const PointGraph& pg) {
    for (auto& [src, edges]: pg) {
        std::cout << "Point " << src << ":\n";
        for (const Point& tgt: edges) {
            std::cout << "  edge to " << tgt << "\n";
        }
    }
}

// ================================================================================================
#endif
