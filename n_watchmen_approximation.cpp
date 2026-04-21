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

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point;
typedef Kernel::Segment_2                                 Segment;
typedef CGAL::Polygon_2<Kernel>                           Polygon;

typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel> CDT;
typedef CGAL::Arr_segment_traits_2<Kernel>                 ArrTraits;
typedef CGAL::Arrangement_2<ArrTraits>                     Arrangement;
typedef CGAL::Simple_polygon_visibility_2<Arrangement, CGAL::Tag_true> VisibilityQuery;

using PatrolEdge = std::pair<Point, Point>; // first == second indicates stationary guard
using Patrol = std::vector<PatrolEdge>;

using PointGraph = std::map<Point, std::set<Point>>; // adjacency list graph
using FaceSet = std::unordered_set<CDT::Face_handle>;

// ================================================================================================

// Reads a simple polygon from disk and normalizes it to CCW orientation.
Polygon load_polygon(const std::string& path);

// Solver function
void find_patrols_1(std::vector<Patrol>& solution, const Polygon& polygon, const int n);

// Finds the visibility polygon from a point in the polygon.
Polygon visibility(const Point& pt, const Polygon& polygon);

// Verifies that the specified patrol is fully connected
bool verify(const Patrol& p);

// Extracts a point graph and internal faces from a triangulation.
void extract_triangulation_info(const CDT& cdt, PointGraph& graph, FaceSet& internal);

// Development helper for exporting polygons in the same simple point format as input.
void write_polygon(const Polygon& polygon, std::ostream& ostr);

// ================================================================================================

// Distance squared between two points
Kernel::FT dist2(const Point& p1, const Point& p2) { return CGAL::squared_distance(p1, p2); }

// Converts an arrangement face boundary back into a simple CCW polygon.
Polygon polygon_from_face(const Arrangement::Face_handle& face) {
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

// Finds a boundary halfedge incident to the given vertex and oriented along the polygon interior.
Arrangement::Halfedge_const_handle boundary_halfedge_for_vertex(const Arrangement::Vertex_const_handle& vertex) {
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

int main(int argc, char** argv) {
    assert(argc == 4);
    int n = std::stoi(argv[1]);

    // Read input polygon
    Polygon polygon = load_polygon(argv[2]);

    // Begin solving
    std::vector<Patrol> solution;
    find_patrols_1(solution, polygon, n);
    assert(solution.size() == n);

    // Verify solution patrols are connected
    for (const Patrol& p: solution) {
        assert(verify(p));
    }

    // Collect solution stats
    std::vector<double> patrol_len;
    double total_len = 0;
    for (const Patrol& p: solution) {
        double& len = patrol_len.emplace_back();
        for (const PatrolEdge& pe: p) {
            if (pe.first == pe.second) { continue; }
            len += std::sqrt(CGAL::to_double(CGAL::squared_distance(pe.first, pe.second)));
        }
        total_len += len;
    }

    // Output solution
    {
        std::ofstream ostr;
        ostr.open(argv[3]);
        ostr << "FOUND TOTAL PATROL CONFIGURATION OF LENGTH " << total_len << '\n';
        for (int i = 0; i < solution.size(); i++) {
            ostr << "PATROL " << i << ":\n  LENGTH " << patrol_len[i] << '\n';
            for (const PatrolEdge& pe: solution[i]) {
                ostr << "  (" << pe.first.x() << ", " << pe.first.y() << ')';
                if (pe.first == pe.second) {
                    ostr << "\n";
                    continue;
                }
                ostr << " <--> (" << pe.second.x() << ", " << pe.second.y() << ")\n";
            }
        }
    }

    return EXIT_SUCCESS;
}

// ================================================================================================

Polygon load_polygon(const std::string& path) {
    std::ifstream istr(path);
    if (!istr.is_open()) {
        throw std::runtime_error("Failed to open polygon file: " + path);
    }

    std::vector<Point> points;
    std::string line;
    while (std::getline(istr, line)) {
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos) {
            line.erase(comment);
        }

        std::replace(line.begin(), line.end(), ',', ' ');

        std::istringstream line_stream(line);
        double x = 0;
        double y = 0;
        if (!(line_stream >> x >> y)) {
            continue;
        }

        points.emplace_back(x, y);
    }

    Polygon polygon(points.begin(), points.end());
    if (polygon.size() < 3) {
        throw std::runtime_error("Input must contain at least three polygon vertices.");
    }
    if (!polygon.is_simple()) {
        throw std::runtime_error("Input polygon must be simple for the week-2 visibility implementation.");
    }
    if (polygon.orientation() == CGAL::CLOCKWISE) {
        polygon.reverse_orientation();
    }

    return polygon;
}

// ================================================================================================

void find_patrols_1(std::vector<Patrol>& solution, const Polygon& polygon, const int n) {
    // Triangulate polygon
    CDT cdt;
    for (int i = 0; i < polygon.size(); i++) {
        const Point& p1 = polygon[i];
        const Point& p2 = polygon[(i + 1) % polygon.size()];
        cdt.insert_constraint(p1, p2);
    }

    PointGraph graph;
    FaceSet faces;
    extract_triangulation_info(cdt, graph, faces);

    // Helper functions
    auto count_incident_faces = [](const Point& pt, const FaceSet& faces) {
        int count = 0;
        for (CDT::Face_handle face: faces) {
            const Point& p1 = face->vertex(0)->point();
            const Point& p2 = face->vertex(1)->point();
            const Point& p3 = face->vertex(2)->point();
            if (pt == p1 || pt == p2 || pt == p3) { count++; }
        }
        return count;
    };
    auto prune_incident_faces = [](const Point& pt, FaceSet& faces) {
        auto it = faces.begin();
        while (it != faces.end()) {
            const Point& p1 = (*it)->vertex(0)->point();
            const Point& p2 = (*it)->vertex(1)->point();
            const Point& p3 = (*it)->vertex(2)->point();
            if (pt == p1 || pt == p2 || pt == p3) {
                it = faces.erase(it);
            } else it++;
        }
    };

    // Find starting vertex
    std::set<Point> chosen;
    {
        Point start;
        int max = 0;
        for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
            const Point& pt = *it;
            int incident = count_incident_faces(pt, faces);
            if (max < incident) {
                start = pt;
                max = incident;
            }
        }
        chosen.insert(start);
        prune_incident_faces(start, faces);
    }

    // Begin constructing patrol
    Patrol& patrol = solution.emplace_back();
    while (!faces.empty()) {
        PatrolEdge next;
        int max = -1;

        for (const Point& src: chosen) {
            for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
                const Point& tgt = *it;
                if (chosen.find(tgt) != chosen.end()) { continue; }
                
                int incident = count_incident_faces(tgt, faces);
                if (max < incident) {
                    next = { src, tgt };
                    max = incident;
                } else if (max == incident) {
                    if (dist2(src, tgt) < dist2(next.first, next.second)) {
                        next = { src, tgt };
                    }
                }
            }
        }

        prune_incident_faces(next.second, faces);
        chosen.insert(next.second);
        patrol.push_back(next);
    }
    if (chosen.size() == 1) { patrol.emplace_back(*chosen.begin(), *chosen.begin()); }

    if (n == 1) { return; }

    // Transform n = 1 solution to graph
    graph.clear();
    int remaining_edges = 0;
    for (const PatrolEdge& pe: solution[0]) {
        if (pe.first == pe.second) {
            graph[pe.first];
            continue;
        }

        graph[pe.first].insert(pe.second);
        graph[pe.second].insert(pe.first);
        remaining_edges++;
    }

    // Prune longest edges
    for (int i = 1; i < n; i++) {
        PatrolEdge max;
        double max_len = 0;
        for (const auto& [src, edges]: graph) {
            if (edges.empty()) { continue; }
            for (const Point& tgt: edges) {
                const double len = CGAL::to_double(dist2(src, tgt));
                if (max_len < len) {
                    max = { src, tgt };
                    max_len = len;
                }
            }
        }

        graph[max.first].erase(max.second);
        graph[max.second].erase(max.first);
        remaining_edges--;
        if (remaining_edges == 0) { break; }
    }

    // Convert graph back to solution
    solution.clear();
    while (!graph.empty()) {
        Patrol& patrol = solution.emplace_back();

        // Check for stationary patrol
        const Point& start = graph.begin()->first;
        if (graph[start].empty()) {
            patrol.emplace_back(start, start);
            graph.erase(start);
            continue;
        }

        std::vector<Point> to_visit;
        to_visit.push_back(start);
        while (!to_visit.empty()) {
            Point src = to_visit.back();
            to_visit.pop_back();

            while (!graph[src].empty()) {
                Point tgt = *graph[src].begin();
                patrol.emplace_back(tgt, src);
                graph[src].erase(tgt);
                graph[tgt].erase(src);
                to_visit.push_back(tgt);
            }

            graph.erase(src);
        }
    }

    // For completely stationary configs (len == 0), add filler guards to reach n
    while (solution.size() < n) {
        Patrol& filler = solution.emplace_back();
        filler.emplace_back(polygon[0], polygon[0]);
    }
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

bool verify(const Patrol& p) {
    if (p.size() == 1) { return true; }
    
    PointGraph graph;
    for (const PatrolEdge& pe: p) {
        if (pe.first == pe.second) {
            graph[pe.first];
            continue;
        }

        graph[pe.first].insert(pe.second);
        graph[pe.second].insert(pe.first);
    }

    std::set<Point> visited;
    std::vector<Point> to_visit;
    to_visit.push_back(p[0].first);
    while (!to_visit.empty()) {
        Point pt = to_visit.back();
        to_visit.pop_back();
        if (visited.find(pt) != visited.end()) { continue; }

        visited.insert(pt);
        for (const Point& next: graph[pt]) {
            to_visit.push_back(next);
        }
    }

    return visited.size() == graph.size();
}

// ================================================================================================

void extract_triangulation_info(const CDT& cdt, PointGraph& graph, FaceSet& internal) {
    assert(graph.empty() && internal.empty());
    
    FaceSet external;
    {
        FaceSet visited;
        std::vector<CDT::Face_handle> to_visit;

        CDT::Face_handle start;
        {
            auto begin = cdt.incident_faces(cdt.infinite_vertex());
            auto curr = begin;
            do {
                int i = curr->index(cdt.infinite_vertex());
                if (cdt.is_constrained({ curr, i })) {
                    start = curr->neighbor(i);
                    break;
                }
            } while (curr != begin);
            internal.insert(start);
            to_visit.push_back(start);
        }

        while (!to_visit.empty()) {
            CDT::Face_handle curr = to_visit.back();
            to_visit.pop_back();

            if (visited.find(curr) != visited.end()) { continue; }
            visited.insert(curr);

            bool is_external = external.find(curr) != external.end();
            for (int i = 0; i < 3; i++) {
                CDT::Face_handle neighbor = curr->neighbor(i);

                if (visited.find(neighbor) != visited.end()) { continue; }
                if (cdt.is_infinite(neighbor)) { continue; }

                bool crossing = cdt.is_constrained({ curr, i });
                if (crossing) {
                    if (is_external) {
                        internal.insert(neighbor);
                    } else external.insert(neighbor);
                } else {
                    if (is_external) {
                        external.insert(neighbor);
                    } else internal.insert(neighbor);
                }

                to_visit.push_back(neighbor);
            }

        }
    }

    for (CDT::Face_handle face: internal) {
        for (int i = 0; i < 3; i++) {
            const Point& p1 = face->vertex(i)->point();
            const Point& p2 = face->vertex((i + 1) % 3)->point();
            graph[p1].insert(p2);
            graph[p2].insert(p1);
        }
    }
}

// ================================================================================================

void write_polygon(const Polygon& polygon, std::ostream& ostr) {
    for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
        ostr << it->x() << ' ' << it->y() << '\n';
    }
}

// ================================================================================================
