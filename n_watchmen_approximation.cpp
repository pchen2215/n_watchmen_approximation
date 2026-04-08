#include <string>
#include <cassert>
#include <vector>
#include <fstream>
#include <set>
#include <map>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point;
typedef CGAL::Polygon_2<Kernel>                           Polygon;

using PatrolEdge = std::pair<Point, Point>; // first == second indicates stationary guard
using Patrol = std::vector<PatrolEdge>;

// ================================================================================================

// Solver function
void find_patrols(std::vector<Patrol>& solution, const std::vector<Point>& polygon, const int n);

// Finds the visibility polygon from a point in the polygon.
Polygon visibility(const Point& pt, const std::vector<Point>& polygon);

// Verifies that the specified patrol is fully connected
bool verify(const Patrol& p);

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

    // TODO: implement
    assert(false);

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
    
    std::map<Point, std::set<Point>> graph;
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
