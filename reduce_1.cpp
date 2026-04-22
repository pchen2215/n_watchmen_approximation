#include "solution.h"
#include "utils.h"

void reduce_1(std::vector<Patrol>& solution, const Polygon& polygon, int n) {
    PointGraph graph;
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
        for (const auto& [src, edges] : graph) {
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
