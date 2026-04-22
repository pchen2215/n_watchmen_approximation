#include "visibility.h"

// ================================================================================================

// Converts an arrangement face boundary back into a simple CCW polygon
static Polygon polygon_from_face(const Arrangement::Face_handle& face) {
    assert(!face->is_unbounded());

    std::vector<Point> vertices;
    auto curr = face->outer_ccb();
    auto start = curr;
    do {
        const Point& pt = curr->source()->point();
        if (vertices.empty() || vertices.back() != pt) {
            vertices.push_back(pt);
        }
        ++curr;
    } while (curr != start);

    if (vertices.size() > 1 && vertices.front() == vertices.back()) {
        vertices.pop_back();
    }

    Polygon polygon(vertices.begin(), vertices.end());
    assert(!polygon.is_empty());
    assert(polygon.is_simple());
    if (polygon.orientation() == CGAL::CLOCKWISE) {
        polygon.reverse_orientation();
    }
    return polygon;
}

// ================================================================================================

// Finds a boundary halfedge incident to the given vertex and oriented along the polygon interior
static Arrangement::Halfedge_const_handle boundary_halfedge_for_vertex(const Arrangement::Vertex_const_handle& vertex) {
    auto begin = vertex->incident_halfedges();
    auto curr = begin;
    do {
        if (!curr->face()->is_unbounded()) {
            return curr;
        }
        ++curr;
    } while (curr != begin);

    throw std::runtime_error("Failed to find a polygon boundary halfedge for the query vertex.");
}

// ================================================================================================

Polygon visibility(const Point& pt, const Polygon& polygon) {
    const auto side = polygon.bounded_side(pt);
    if (side == CGAL::ON_UNBOUNDED_SIDE) {
        throw std::runtime_error("Visibility query point must lie on or inside the polygon.");
    }

    std::vector<Segment> segments;
    segments.reserve(polygon.size());
    for (int i = 0; i < polygon.size(); i++) {
        segments.emplace_back(polygon[i], polygon[(i + 1) % polygon.size()]);
    }

    Arrangement env;
    CGAL::insert_non_intersecting_curves(env, segments.begin(), segments.end());

    CGAL::Arr_naive_point_location<Arrangement> point_location(env);
    CGAL::Arr_point_location_result<Arrangement>::Type location = point_location.locate(pt);

    VisibilityQuery visibility_query(env);
    Arrangement output;

    if (const auto* face = std::get_if<Arrangement::Face_const_handle>(&location)) {
        Arrangement::Face_handle out_face = visibility_query.compute_visibility(pt, *face, output);
        return polygon_from_face(out_face);
    }

    if (const auto* edge = std::get_if<Arrangement::Halfedge_const_handle>(&location)) {
        Arrangement::Halfedge_const_handle boundary = *edge;
        if (boundary->face()->is_unbounded()) {
            boundary = boundary->twin();
        }
        Arrangement::Face_handle out_face = visibility_query.compute_visibility(pt, boundary, output);
        return polygon_from_face(out_face);
    }

    const auto* vertex = std::get_if<Arrangement::Vertex_const_handle>(&location);
    assert(vertex != nullptr);
    Arrangement::Halfedge_const_handle boundary = boundary_halfedge_for_vertex(*vertex);
    Arrangement::Face_handle out_face = visibility_query.compute_visibility(pt, boundary, output);
    return polygon_from_face(out_face);
}

// ================================================================================================

void update_unseen(const Polygon& seen, std::vector<Polygon>& unseen) {
    std::vector<HoleyPolygon> result;
    for (const Polygon& poly: unseen) {
        std::vector<HoleyPolygon> diff;
        CGAL::difference(poly, seen, std::back_inserter(result));
    }

    unseen.clear();
    for (const HoleyPolygon& poly: result) {
        unseen.push_back(poly.outer_boundary());
    }
}

// ================================================================================================
