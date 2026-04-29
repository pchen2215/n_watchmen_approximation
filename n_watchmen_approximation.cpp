#include "include.h"
#include "solution.h"
#include "utils.h"
#include "visibility.h"

// ================================================================================================

// Reads a simple polygon from disk and normalizes it to CCW orientation.
Polygon load_polygon(const std::string& path);

// Verifies that the specified patrol is fully connected
bool verify(const Patrol& p);
bool verify(const std::vector<Patrol>& s, const Polygon& p);

// Development helper for exporting polygons in the same simple point format as input.
void write_polygon(const Polygon& polygon, std::ostream& ostr);

// ================================================================================================

int main(int argc, char** argv) {
    assert(argc == 4 || argc == 5);
    int n = std::stoi(argv[1]);

    // Read input polygon
    Polygon polygon = load_polygon(argv[2]);

    // Begin solving
    std::vector<Patrol> solution;
    find_init_patrol_2(solution.emplace_back(), polygon);
    reduce(solution, polygon, n);
    assert(solution.size() == n);

    // Verify solution
    for (const Patrol& p: solution) { assert(verify(p)); }
    assert(verify(solution, polygon));

    // Collect solution stats
    std::vector<double> patrol_len;
    double total_len = 0;
    for (const Patrol& p: solution) {
        double& len = patrol_len.emplace_back();
        for (const PatrolEdge& pe: p) {
            if (pe.first == pe.second) { continue; }
            len += std::sqrt(CGAL::to_double(dist2(pe.first, pe.second)));
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

        ostr.close();
        if (argc == 5) {
            std::vector<std::vector<Point>> patrol_points;
            ostr.open(argv[4]);
            for (const Patrol& p: solution) {
                ostr << "PATROL\n";
                PointGraph graph = to_graph(p);
                patrol_points.emplace_back();

                std::set<Point> unvisited;
                for (auto it = graph.begin(); it != graph.end(); it++) {
                    unvisited.insert(it->first);
                }

                std::function<void(const Point&)> dfs = [&](const Point& curr) {
                    unvisited.erase(curr);
                    patrol_points.back().push_back(curr);
                    ostr << curr << "\n";
                    for (const Point& next: graph[curr]) {
                        if (unvisited.find(next) != unvisited.end()) {
                            dfs(next);
                        }
                    }
                };

                bool wrote = false;
                for (const PatrolEdge& pe: p) {
                    if (graph[pe.first].size() == 1) {
                        dfs(pe.first);
                        wrote = true;
                        break;
                    } else if (graph[pe.second].size() == 1) {
                        dfs(pe.second);
                        wrote = true;
                        break;
                    }
                }
                if (!wrote) { dfs(graph.begin()->first); }
            }
            ostr.close();
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

bool verify(const Patrol& p) {
    if (p.size() == 1) { return true; }
    PointGraph graph = to_graph(p);

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

bool verify(const std::vector<Patrol>& s, const Polygon& p) {
    std::vector<Polygon> unseen;
    unseen.push_back(p);
    for (const Patrol& patrol: s) {
        for (const PatrolEdge& pe: patrol) {
            update_unseen(visibility(pe.first, p), unseen);
            update_unseen(visibility(pe.second, p), unseen);
        }
    }
    return unseen.empty();
}

// ================================================================================================

void write_polygon(const Polygon& polygon, std::ostream& ostr) {
    for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); ++it) {
        ostr << it->x() << ' ' << it->y() << '\n';
    }
}

// ================================================================================================
