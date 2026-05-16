#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include "CLI/CLI.hpp"

CLI::App app{"vrp"};

struct Point { double demand, x, y; };

double dist(const Point& a, const Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

double dist2(const Point& a, const Point& b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

std::tuple<size_t, size_t, double, std::vector<Point>> read_data(const std::string &filename) {
    std::ifstream file(filename);
    size_t n, v;
    double c;
    file >> n >> v >> c;

    std::vector<Point> pts(n);
    for (size_t i = 0; i < n; ++i) {
        file >> pts[i].demand >> pts[i].x >> pts[i].y;
    }

    return {n, v, c, std::move(pts)};
}

bool assign_routes_by_order(const std::vector<size_t> &order, size_t v, double c, const std::vector<Point> &pts, std::vector<std::vector<size_t>> &routes) {
    routes.assign(v, {});
    std::vector<double> remaining(v, c);

    for (size_t idx : order) {
        if (idx == 0 || idx >= pts.size()) {
            return false;
        }
        if (pts[idx].demand > c) {
            return false;
        }

        size_t best_vehicle = v;
        double best_remaining = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < v; ++j) {
            double after = remaining[j] - pts[idx].demand;
            if (after >= 0 && after < best_remaining) {
                best_remaining = after;
                best_vehicle = j;
            }
        }
        if (best_vehicle == v) {
            return false;
        }

        routes[best_vehicle].push_back(idx);
        remaining[best_vehicle] -= pts[idx].demand;
    }
    return true;
}

std::vector<std::vector<size_t>> assign_routes_nearest(size_t n, size_t v, double c, const std::vector<Point> &pts) {
    std::vector<bool> used(n, false);
    used[0] = true;
    size_t current = 0;

    std::vector<size_t> order;
    order.reserve(n - 1);

    for (size_t step = 1; step < n; ++step) {
        size_t best = 0;
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t j = 1; j < n; ++j) {
            if (!used[j]) {
                double d = dist2(pts[current], pts[j]);
                if (d < best_dist) {
                    best_dist = d;
                    best = j;
                }
            }
        }
        used[best] = true;
        order.push_back(best);
        current = best;
    }

    std::vector<std::vector<size_t>> routes;
    if (assign_routes_by_order(order, v, c, pts, routes)) {
        return routes;
    }

    std::vector<size_t> fallback_order;
    fallback_order.reserve(n - 1);
    for (size_t i = 1; i < n; ++i) {
        fallback_order.push_back(i);
    }
    std::sort(fallback_order.begin(), fallback_order.end(), [&](size_t a, size_t b) {
        return pts[a].demand > pts[b].demand;
    });
    assign_routes_by_order(fallback_order, v, c, pts, routes);
    return routes;
}

std::vector<std::vector<size_t>> assign_routes_sweep(size_t n, size_t v, double c, const std::vector<Point> &pts) {
    std::vector<std::pair<double, size_t>> angular_order;
    angular_order.reserve(n - 1);
    for (size_t i = 1; i < n; ++i) {
        double angle = std::atan2(pts[i].y - pts[0].y, pts[i].x - pts[0].x);
        angular_order.emplace_back(angle, i);
    }
    std::sort(angular_order.begin(), angular_order.end());

    std::vector<size_t> order;
    order.reserve(n - 1);
    for (auto &[angle, idx] : angular_order) {
        order.push_back(idx);
    }

    std::vector<std::vector<size_t>> routes;
    if (assign_routes_by_order(order, v, c, pts, routes)) {
        return routes;
    }

    std::sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return pts[a].demand > pts[b].demand;
    });
    assign_routes_by_order(order, v, c, pts, routes);
    return routes;
}

std::pair<double, std::vector<size_t>> tsp_route_greedy(const std::vector<Point> &pts, const std::vector<size_t> &nodes) {
    size_t m = nodes.size();
    if (m == 0) return {0.0, {}};

    std::vector<bool> used(m, false);
    std::vector<size_t> path;
    path.reserve(m);
    size_t current = 0;

    for (size_t step = 0; step < m; ++step) {
        size_t best = 0;
        double best_dist = std::numeric_limits<double>::infinity();
        for (size_t j = 0; j < m; ++j) {
            if (used[j]) continue;
            double d = dist2(pts[current], pts[nodes[j]]);
            if (d < best_dist) {
                best_dist = d;
                best = j;
            }
        }
        used[best] = true;
        current = nodes[best];
        path.push_back(current);
    }

    double cost = dist(pts[0], pts[path.front()]);
    for (size_t i = 1; i < m; ++i) {
        cost += dist(pts[path[i - 1]], pts[path[i]]);
    }
    cost += dist(pts[path.back()], pts[0]);

    return {cost, path};
}

void route_2opt(std::vector<size_t> &path, const std::vector<Point> &pts) {
    size_t m = path.size();
    if (m < 3) return;

    bool improved = true;
    while (improved) {
        improved = false;
        for (size_t i = 0; i + 1 < m; ++i) {
            for (size_t j = i + 1; j < m; ++j) {
                size_t a = (i == 0 ? 0 : path[i - 1]);
                size_t b = path[i];
                size_t c = path[j];
                size_t d = (j + 1 < m ? path[j + 1] : 0);

                double before = dist(pts[a], pts[b]) + dist(pts[c], pts[d]);
                double after = dist(pts[a], pts[c]) + dist(pts[b], pts[d]);
                if (after + 1e-12 < before) {
                    std::reverse(path.begin() + i, path.begin() + j + 1);
                    improved = true;
                    break;
                }
            }
            if (improved) break;
        }
    }
}

std::pair<double, std::vector<size_t>> tsp_route_2opt(const std::vector<Point> &pts, const std::vector<size_t> &nodes) {
    auto [cost, path] = tsp_route_greedy(pts, nodes);
    route_2opt(path, pts);
    if (!path.empty()) {
        cost = dist(pts[0], pts[path.front()]);
        for (size_t i = 1; i < path.size(); ++i) {
            cost += dist(pts[path[i - 1]], pts[path[i]]);
        }
        cost += dist(pts[path.back()], pts[0]);
    }
    return {cost, path};
}

std::pair<double, std::vector<std::vector<size_t>>> optimize_routes(const std::vector<Point> &pts, std::vector<std::vector<size_t>> routes) {
    double total_cost = 0.0;
    for (auto &route : routes) {
        auto [route_cost, optimized_route] = tsp_route_2opt(pts, route);
        route = std::move(optimized_route);
        total_cost += route_cost;
    }
    return {total_cost, routes};
}

std::pair<double, std::vector<std::vector<size_t>>> vrp_greedy(size_t n, size_t v, double c, const std::vector<Point> &pts) {
    std::vector<std::pair<double, std::vector<std::vector<size_t>>>> candidates;

    auto nearest_routes = assign_routes_nearest(n, v, c, pts);
    candidates.push_back(optimize_routes(pts, nearest_routes));

    auto sweep_routes = assign_routes_sweep(n, v, c, pts);
    candidates.push_back(optimize_routes(pts, sweep_routes));

    auto best = candidates.front();
    for (auto &candidate : candidates) {
        if (candidate.first < best.first) {
            best = candidate;
        }
    }

    return best;
}

int main(int argc, char* argv[]) {
    std::cout << std::fixed << std::setprecision(6);

    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [n, v, c, pts] = read_data(filename);
    auto [cost, routes] = vrp_greedy(n, v, c, pts);

    if (!routes.empty()) {
        std::cout << cost << '\n';
        for (size_t i = 0; i < routes.size(); ++i) {
            if (!routes[i].empty()) {
                std::cout << "Vehicle " << i + 1 << ": 0 ";
                for (size_t node : routes[i]) {
                    std::cout << node << " ";
                }
                std::cout << "0\n";
            }
        }
    }

    return 0;
}
