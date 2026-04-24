#include "solution.h"
#include "utils.h"
#include "visibility.h"

enum reduction_type {
    NONE,
    REDUCTION_TYPE_EDGE,
    REDUCTION_TYPE_POINT
};

void reduce(std::vector<Patrol>& solution, const Polygon& polygon, int n) {
    PointGraph graph = to_graph(solution[0]);

    // Simplify initial solution
    if (graph.size() > 1) {
        
        // Check for leaf reductions
        auto leaf_reduction = [&]() {
            std::vector<Point> leaves;
            for (auto it = graph.begin(); it != graph.end(); it++) {
                if (it->second.size() == 1) {
                    leaves.push_back(it->first);
                }
            }

            for (const Point& leaf: leaves) {
                std::vector<Polygon> unseen;
                unseen.push_back(polygon);
                for (auto it = graph.begin(); it != graph.end(); it++) {
                    if (it->first == leaf) { continue; }
                    update_unseen(visibility(it->first, polygon), unseen);
                    if (unseen.empty()) { break; }
                }

                if (unseen.empty()) {
                    graph[*graph[leaf].begin()].erase(leaf);
                    graph.erase(leaf);
                }
            }
        };

        auto pipe_reduction = [&]() {
            std::set<Point> pipes;
            for (auto it = graph.begin(); it != graph.end(); it++) {
                if (it->second.size() == 2) {
                    pipes.insert(it->first);
                }
            }

            for (const Point& pipe: pipes) {
                std::vector<Polygon> unseen;
                unseen.push_back(polygon);
                for (auto it = graph.begin(); it != graph.end(); it++) {
                    if (it->first == pipe) { continue; }
                    update_unseen(visibility(it->first, polygon), unseen);
                    if (unseen.empty()) { break; }
                }

                if (unseen.empty()) {
                    Point p1 = *graph[pipe].begin();
                    Point p2 = *(++graph[pipe].begin());
                    if (!crosses(p1, p2, polygon)) {
                        graph[p1].insert(p2);
                        graph[p1].erase(pipe);
                        graph[p2].insert(p1);
                        graph[p2].erase(pipe);
                        graph.erase(pipe);
                    }
                }
            }
        };

        // Prune longest edges
        int i = 1;
        while (true) {
            leaf_reduction();
            pipe_reduction();
            if (i == n) { break; }

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
            bool has_edges = false;
            for (const auto& [src, edges]: graph) {
                if (!edges.empty()) {
                    has_edges = true;
                    break;
                }
            }
            if (!has_edges) { break; }
            i++;
        }
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
