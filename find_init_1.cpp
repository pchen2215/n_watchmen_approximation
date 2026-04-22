#include "solution.h"
#include "utils.h"

// ================================================================================================

// Extracts a point graph and internal faces from a triangulation
static void extract_triangulation_info(const CDT& cdt, PointGraph& graph, FaceSet& internal) {
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

void find_init_patrol_1(Patrol& patrol, const Polygon& polygon) {
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
}

// ================================================================================================
