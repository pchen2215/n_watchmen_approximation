#pragma once

#include <string>
#include <cassert>
#include <vector>
#include <fstream>
#include <sstream>
#include <set>
#include <map>
#include <unordered_set>
#include <stdexcept>
#include <algorithm>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Arrangement_2.h>
#include <CGAL/Arr_segment_traits_2.h>
#include <CGAL/Arr_naive_point_location.h>
#include <CGAL/Simple_polygon_visibility_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/intersections.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point;
typedef Kernel::Segment_2                                 Segment;
typedef CGAL::Polygon_2<Kernel>                           Polygon;
typedef CGAL::Polygon_with_holes_2<Kernel>                HoleyPolygon;

typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel> CDT;
typedef CGAL::Arr_segment_traits_2<Kernel>                 ArrTraits;
typedef CGAL::Arrangement_2<ArrTraits>                     Arrangement;
typedef CGAL::Simple_polygon_visibility_2<Arrangement, CGAL::Tag_true> VisibilityQuery;

using PatrolEdge = std::pair<Point, Point>; // first == second indicates stationary guard
using Patrol = std::vector<PatrolEdge>;

using PointGraph = std::map<Point, std::set<Point>>; // adjacency list graph
using FaceSet = std::unordered_set<CDT::Face_handle>;
