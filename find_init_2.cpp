#include "solution.h"
#include "utils.h"
#include "visibility.h"

// ================================================================================================

void find_init_patrol_2(std::vector<Patrol>& solution, const Polygon& polygon) {
    Patrol& patrol = solution.emplace_back();

    // Populate candidate vertices
    std::set<Point> candidates;
    for (auto it = polygon.vertices_begin(); it != polygon.vertices_end(); it++) {
        candidates.insert(*it);
    }

    // Initialize unseen area
    std::vector<Polygon> unseen;
    unseen.push_back(polygon);
    
    // Find best initial point
    Point start;
    {
        double best = DBL_MAX;
        for (const Point& pt : candidates) {
            std::vector<Polygon> temp_unseen = unseen;
            update_unseen(visibility(pt, polygon), temp_unseen);
            double remaining = total_area(temp_unseen);
            if (remaining < best) {
                start = pt;
                best = remaining;
            }
        }
    }
    candidates.erase(start);
    update_unseen(visibility(start, polygon), unseen);

    // Early exit if single stationary guard is sufficient
    if (unseen.empty()) {
        patrol.emplace_back(start, start);
        return;
    }

    // Begin solving
    std::vector<Point> chosen;
    chosen.push_back(start);
    while (!unseen.empty()) {
        Point src;
        Point tgt;

        // Select candidate that would minimize unseen area if chosen
        double best = DBL_MAX;
        for (const Point& pt: candidates) {
            std::vector<Polygon> temp_unseen = unseen;
            update_unseen(visibility(pt, polygon), temp_unseen);
            double remaining = total_area(temp_unseen);

            // Check if good-coverage candidates can be reached from an existing chosen point
            if (remaining < best) {
                bool can_reach = false;
                double best2 = DBL_MAX;
                for (const Point& pt_ch: chosen) {
                    if (crosses(pt, pt_ch, polygon)) { continue; }
                    can_reach = true;
                    double dist = CGAL::to_double(dist2(pt, pt_ch));
                    if (dist < best2) {
                        src = pt_ch;
                        best2 = dist;
                    }
                }

                // Ignore unreachable candidates
                if (!can_reach) { continue; }
                tgt = pt;
                best = remaining;
            }
        }

        // Apply choice
        chosen.push_back(tgt);
        candidates.erase(tgt);
        patrol.emplace_back(src, tgt);
        update_unseen(visibility(tgt, polygon), unseen);
    }
}

// ================================================================================================
