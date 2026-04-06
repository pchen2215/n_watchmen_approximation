#include <string>
#include <cassert>
#include <vector>
#include <fstream>
#include <set>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;
typedef Kernel::Point_2                                   Point;

using PatrolEdge = std::pair<Point, Point>; // first == second indicates stationary guard
using Patrol = std::vector<PatrolEdge>;

// Solver function
void find_patrols(std::vector<Patrol>& solution, const std::vector<Point>& polygon, const int n) {
    
    // TEMPORARY FUNCTION BODY -- AWAITING ACTUAL IMPLEMENTATION

    Patrol patrol;

    PatrolEdge edge = { Point(1, 1), Point(1, 2) };
    patrol.push_back(edge);

    edge = { Point(1, 2), Point(3, 5) };
    patrol.push_back(edge);

    solution.push_back(patrol);

    patrol.clear();
    edge = { Point(87.5, 79.2), Point(87.5, 79.2) };
    patrol.push_back(edge);

    solution.push_back(patrol);

    // TEMPORARY FUNCTION BODY -- AWAITING ACTUAL IMPLEMENTATION

}

// Verifies that the specified patrol is fully connected
bool verify(const Patrol& p) {
    if (p.size() == 1) { return p[0].first == p[0].second; }

    // TODO: IMPLEMENT
    // possible solution is to construct graph from the patrol and check if the graph is connected

    return true; // TEMP RETURN
}

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