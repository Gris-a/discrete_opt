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
#include <random>

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


double get_total_cost(const std::vector<std::vector<size_t>>& routes, const std::vector<Point>& pts) {
    double total_cost = 0.0;
    for (const auto& route : routes) {
        if (route.empty()) continue;
        total_cost += dist(pts[0], pts[route.front()]);
        for (size_t i = 1; i < route.size(); ++i) {
            total_cost += dist(pts[route[i - 1]], pts[route[i]]);
        }
        total_cost += dist(pts[route.back()], pts[0]);
    }
    return total_cost;
}

std::pair<double, std::vector<std::vector<size_t>>> vrp_local_search(size_t n, size_t v, double c, const std::vector<Point>& pts, std::vector<std::vector<size_t>> initial_routes) 
{
    std::mt19937 rng(137);
    int max_iterations = 80000;
    
    auto best_routes = initial_routes;
    double best_cost = get_total_cost(best_routes, pts);
    
    auto current_routes = best_routes;
    double current_cost = best_cost;

    size_t num_to_remove = std::max<size_t>(1, static_cast<size_t>(n * 0.15));

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<std::vector<size_t>> loop_routes = current_routes;
        std::vector<bool> removed(n, false);
        std::vector<size_t> removed_customers;

        std::vector<size_t> all_customers;
        for (size_t i = 1; i < n; ++i) all_customers.push_back(i);
        std::shuffle(all_customers.begin(), all_customers.end(), rng);

        for (size_t i = 0; i < num_to_remove; ++i) {
            removed[all_customers[i]] = true;
            removed_customers.push_back(all_customers[i]);
        }

        std::vector<double> remaining_capacity(v, c);
        for (size_t j = 0; j < v; ++j) {
            std::vector<size_t> clean_route;
            for (size_t node : loop_routes[j]) {
                if (!removed[node]) {
                    clean_route.push_back(node);
                    remaining_capacity[j] -= pts[node].demand;
                }
            }
            loop_routes[j] = std::move(clean_route);
        }

        std::sort(removed_customers.begin(), removed_customers.end(), [&](size_t a, size_t b) {
            return pts[a].demand > pts[b].demand;
        });

        bool success = true;
        for (size_t cust : removed_customers) {
            size_t best_v = v;
            size_t best_pos = 0;
            double min_delta = std::numeric_limits<double>::infinity();

            for (size_t j = 0; j < v; ++j) {
                if (remaining_capacity[j] < pts[cust].demand) continue;

                for (size_t pos = 0; pos <= loop_routes[j].size(); ++pos) {
                    size_t prev = (pos == 0) ? 0 : loop_routes[j][pos - 1];
                    size_t next = (pos == loop_routes[j].size()) ? 0 : loop_routes[j][pos];

                    double delta = dist(pts[prev], pts[cust]) + dist(pts[cust], pts[next]) - dist(pts[prev], pts[next]);
                    if (delta < min_delta) {
                        min_delta = delta;
                        best_v = j;
                        best_pos = pos;
                    }
                }
            }

            if (best_v == v) {
                success = false;
                break;
            }

            loop_routes[best_v].insert(loop_routes[best_v].begin() + best_pos, cust);
            remaining_capacity[best_v] -= pts[cust].demand;
        }

        if (!success) continue;

        for (size_t j = 0; j < v; ++j) {
            route_2opt(loop_routes[j], pts);
        }

        double loop_cost = get_total_cost(loop_routes, pts);

        if (loop_cost < current_cost) {
            current_routes = loop_routes;
            current_cost = loop_cost;

            if (current_cost < best_cost) {
                best_routes = current_routes;
                best_cost = current_cost;
            }
        }
    }

    return {best_cost, best_routes};
}

std::pair<double, std::vector<std::vector<size_t>>> vrp_annealing(size_t n, size_t v, double c, const std::vector<Point>& pts, std::vector<std::vector<size_t>> initial_routes) 
{
    std::mt19937 rng(137);
    double T = 1000;
    const double alpha = 0.9995;
    int max_iterations = 200000;
    
    auto best_routes = initial_routes;
    double best_cost = get_total_cost(best_routes, pts);
    
    auto current_routes = best_routes;
    double current_cost = best_cost;

    size_t num_to_remove = std::max<size_t>(1, static_cast<size_t>(n * 0.25));

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::vector<std::vector<size_t>> loop_routes = current_routes;
        std::vector<bool> removed(n, false);
        std::vector<size_t> removed_customers;

        std::vector<size_t> all_customers;
        for (size_t i = 1; i < n; ++i) all_customers.push_back(i);
        std::shuffle(all_customers.begin(), all_customers.end(), rng);

        for (size_t i = 0; i < num_to_remove; ++i) {
            removed[all_customers[i]] = true;
            removed_customers.push_back(all_customers[i]);
        }

        std::vector<double> remaining_capacity(v, c);
        for (size_t j = 0; j < v; ++j) {
            std::vector<size_t> clean_route;
            for (size_t node : loop_routes[j]) {
                if (!removed[node]) {
                    clean_route.push_back(node);
                    remaining_capacity[j] -= pts[node].demand;
                }
            }
            loop_routes[j] = std::move(clean_route);
        }

        std::sort(removed_customers.begin(), removed_customers.end(), [&](size_t a, size_t b) {
            return pts[a].demand > pts[b].demand;
        });

        bool success = true;
        for (size_t cust : removed_customers) {
            size_t best_v = v;
            size_t best_pos = 0;
            double min_delta = std::numeric_limits<double>::infinity();

            for (size_t j = 0; j < v; ++j) {
                if (remaining_capacity[j] < pts[cust].demand) continue;

                for (size_t pos = 0; pos <= loop_routes[j].size(); ++pos) {
                    size_t prev = (pos == 0) ? 0 : loop_routes[j][pos - 1];
                    size_t next = (pos == loop_routes[j].size()) ? 0 : loop_routes[j][pos];

                    double delta = dist(pts[prev], pts[cust]) + dist(pts[cust], pts[next]) - dist(pts[prev], pts[next]);
                    if (delta < min_delta) {
                        min_delta = delta;
                        best_v = j;
                        best_pos = pos;
                    }
                }
            }

            if (best_v == v) {
                success = false;
                break;
            }

            loop_routes[best_v].insert(loop_routes[best_v].begin() + best_pos, cust);
            remaining_capacity[best_v] -= pts[cust].demand;
        }

        if (!success) continue;

        for (size_t j = 0; j < v; ++j) {
            route_2opt(loop_routes[j], pts);
        }

        double loop_cost = get_total_cost(loop_routes, pts);

        if (loop_cost < current_cost || std::exp((current_cost - loop_cost) / T) > std::uniform_real_distribution<>(0.0, 1.0)(rng)) {
            current_routes = loop_routes;
            current_cost = loop_cost;

            if (current_cost < best_cost) {
                best_routes = current_routes;
                best_cost = current_cost;
            }
        }

        T *= alpha;
    }

    return {best_cost, best_routes};
}

int main(int argc, char* argv[]) {
    std::cout << std::fixed << std::setprecision(6);

    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [n, v, c, pts] = read_data(filename);
    auto [gcost, groutes] = vrp_greedy(n, v, c, pts);
    auto [cost, routes] = vrp_annealing(n, v, c, pts, groutes);

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

    return 0;
}
