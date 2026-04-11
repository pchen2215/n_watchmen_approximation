#include <string>
#include <cassert>
#include <vector>
#include <fstream>
#include <set>
#include <map>
#include <unordered_set>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Polygon_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point;
typedef CGAL::Polygon_2<Kernel>                           Polygon;

typedef CGAL::Constrained_Delaunay_triangulation_2<Kernel> CDT;

using PatrolEdge = std::pair<Point, Point>; // first == second indicates stationary guard
using Patrol = std::vector<PatrolEdge>;

using PointGraph = std::map<Point, std::set<Point>>; // adjacency list graph
using FaceSet = std::unordered_set<CDT::Face_handle>;

// ================================================================================================

// Solver function
void find_patrols(std::vector<Patrol>& solution, const std::vector<Point>& polygon, const int n);

// Finds the visibility polygon from a point in the polygon.
Polygon visibility(const Point& pt, const std::vector<Point>& polygon);

// Verifies that the specified patrol is fully connected
bool verify(const Patrol& p);

// Extracts a point graph and internal faces from a triangulation.
void extract_triangulation_info(const CDT& cdt, PointGraph& graph, FaceSet& internal);

// ================================================================================================

Kernel::FT dist2(const Point& p1, const Point& p2) { return CGAL::squared_distance(p1, p2); }

// ================================================================================================

int main(int argc, char** argv) {
    assert(argc == 4);
    int n = std::stoi(argv[1]);

    // Read input polygon
    std::vector<Point> polygon;
    {
        std::ifstream istr;
        istr.open(argv[2]);
        
        double x = 0;
        double y = 0;
        while (istr >> x) {
            istr >> y;
            polygon.emplace_back(x, y);
        }
    }

    // Begin solving
    std::vector<Patrol> solution;
    find_patrols(solution, polygon, n);
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
            ostr << "PATROL " << i << ":\n";
            ostr << "  LENGTH " << patrol_len[i] << '\n';
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

void find_patrols(std::vector<Patrol>& solution, const std::vector<Point>& polygon, const int n) {
    // Triangulate polygon
    CDT cdt;
    std::set<Point> orig_pts;
    for (int i = 0; i < polygon.size(); i++) {
        const Point& p1 = polygon[i];
        const Point& p2 = polygon[(i + 1) % polygon.size()];
        cdt.insert_constraint(p1, p2);
        orig_pts.insert(p1);
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
        for (const Point& pt: polygon) {
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
            for (const Point& tgt: polygon) {
                if (chosen.find(tgt) != chosen.end()) { continue; }
                
                int incident = count_incident_faces(tgt, faces);
                if (max < incident) {
                    next = { src, tgt };
                    max = incident;
                } else if (max == incident) {
                    
                    // TODO: tiebreak by distance

                }
            }
        }

        prune_incident_faces(next.second, faces);
        chosen.insert(next.second);
        patrol.push_back(next);
    }
    if (chosen.size() == 1) { patrol.emplace_back(*chosen.begin(), *chosen.begin()); }

    if (n == 1) { return; }

    // TODO: implement for 1 < n

}

// ================================================================================================

Polygon visibility(const Point& pt, const std::vector<Point>& polygon) {

    // TODO: implement
    assert(false);

    return Polygon(); // temp
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
