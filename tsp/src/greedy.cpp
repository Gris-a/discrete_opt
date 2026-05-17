#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <tuple>
#include <random>
#include <algorithm>
#include <numeric>
#include <iomanip>
#include <chrono>

#include "CLI/CLI.hpp"

CLI::App app{"tsp"};

struct Point { double x, y; };

double dist2(const Point &a, const Point &b) {
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return dx * dx + dy * dy;
}

double dist(const Point &a, const Point &b) {
    return std::sqrt(dist2(a, b));
}

std::tuple<size_t, std::vector<Point>> read_data(const std::string &filename) {
    std::ifstream file(filename);
    size_t n; file >> n;
    
    std::vector<Point> pts(n);
    for (size_t i = 0; i < n; ++i) {
        file >> pts[i].x >> pts[i].y;
    }

    return {n, std::move(pts)};
}

std::pair<double, std::vector<size_t>> tsp_greedy(size_t n, const std::vector<Point> &pts) {
    std::vector<size_t> path;
    double total_cost = 0.0;

    std::vector<bool> visited(n, false);

    size_t current = 0;
    path.push_back(current);
    visited[current] = true;

    for (size_t i = 1; i < n; ++i) {
        size_t best_next = 0;
        double min_dist = std::numeric_limits<double>::max();
        
        for (size_t j = 0; j < n; ++j) {
            if (!visited[j]) {
                if (double d = dist2(pts[current], pts[j]); d < min_dist) {
                    min_dist = d;
                    best_next = j;
                }
            }
        }
        
        path.push_back(best_next);
        visited[best_next] = true;

        total_cost += std::sqrt(min_dist);
        current = best_next;
    }
    
    total_cost += dist(pts[current], pts[0]);
    return {total_cost, path};
}

void tsp_2opt_internal(std::vector<size_t> &path, double &total_cost, size_t n, const std::vector<Point> &pts, std::chrono::steady_clock::time_point end_time) {
    bool improved = true;

    while (improved) {
        improved = false;
        for (size_t i = 0; i < n; ++i) {
            if (std::chrono::steady_clock::now() >= end_time) {
                return;
            }

            for (size_t j = i + 2; j < n; ++j) {
                if (i == 0 && j == n - 1) continue;

                size_t u1 = path[i];
                size_t v1 = path[(i + 1) % n];
                size_t u2 = path[j];
                size_t v2 = path[(j + 1) % n];

                double current_dist = dist(pts[u1], pts[v1]) + dist(pts[u2], pts[v2]);
                double new_dist = dist(pts[u1], pts[u2]) + dist(pts[v1], pts[v2]);
                double delta = new_dist - current_dist;

                if (delta < 0) {
                    std::reverse(path.begin() + i + 1, path.begin() + j + 1);
                    total_cost += delta;
                    improved = true;
                    break; 
                }
            }
            if (improved) break; 
        }
    }
}

std::pair<double, std::vector<size_t>> tsp_2opt(size_t n, const std::vector<Point> &pts, std::chrono::seconds time_limit = std::chrono::seconds(60)) {
    auto start = std::chrono::steady_clock::now();
    auto [total_cost, path] = tsp_greedy(n, pts);
    auto end_time = start + time_limit;
    tsp_2opt_internal(path, total_cost, n, pts, end_time);
    return {total_cost, path};
}

void annealing_4opt(std::vector<size_t> &path, std::mt19937 &gen) {
    size_t n = path.size();
    if (n < 8) return;

    std::uniform_int_distribution<size_t> dist_pos(2, n / 4);
    size_t pos1 = dist_pos(gen);
    size_t pos2 = pos1 + dist_pos(gen);
    size_t pos3 = pos2 + dist_pos(gen);

    std::vector<size_t> new_path;
    new_path.reserve(n);

    new_path.insert(new_path.end(), path.begin(), path.begin() + pos1);
    new_path.insert(new_path.end(), path.begin() + pos3, path.end());
    new_path.insert(new_path.end(), path.begin() + pos2, path.begin() + pos3);
    new_path.insert(new_path.end(), path.begin() + pos1, path.begin() + pos2);

    path = std::move(new_path);
}

double calculate_total_cost(const std::vector<size_t> &path, const std::vector<Point> &pts) {
    double cost = 0.0;
    size_t n = path.size();
    for (size_t i = 0; i < n; ++i) {
        cost += dist(pts[path[i]], pts[path[(i + 1) % n]]);
    }
    return cost;
}

std::pair<double, std::vector<size_t>> tsp_annealing(size_t n, const std::vector<Point> &pts, std::chrono::seconds time_limit = std::chrono::seconds(30)) {
    auto overall_start = std::chrono::steady_clock::now();
    auto [current_cost, current_path] = tsp_greedy(n, pts);
    
    auto overall_end = overall_start + time_limit;
    auto polish_end = std::min(overall_end, overall_start + std::chrono::seconds(20));
    tsp_2opt_internal(current_path, current_cost, n, pts, polish_end);

    double best_cost = current_cost;
    std::vector<size_t> best_path = current_path;

    std::mt19937 gen(1337);
    std::uniform_real_distribution<double> unif(0.0, 1.0);

    double T = 100;
    const double alpha = 0.95;

    while (std::chrono::steady_clock::now() < overall_end) {
        std::vector<size_t> candidate_path = current_path;
        annealing_4opt(candidate_path, gen);
        double candidate_cost = calculate_total_cost(candidate_path, pts);

        if (std::chrono::steady_clock::now() < overall_end) {
            auto remaining_end = overall_end;
            auto local_limit = std::min(remaining_end, std::chrono::steady_clock::now() + std::chrono::seconds(5));
            tsp_2opt_internal(candidate_path, candidate_cost, n, pts, local_limit);
        }

        double delta = candidate_cost - current_cost;
        if (delta < 0 || (unif(gen) < std::exp(-delta / T)))  {
            current_cost = std::move(candidate_cost);
            current_path = std::move(candidate_path);
            if (current_cost < best_cost) {
                best_cost = current_cost;
                best_path = current_path;
            }
        }
        
        T *= alpha;
    }

    return {best_cost, best_path};
}

int main(int argc, char* argv[]) {
    std::cout << std::fixed << std::setprecision(6);

    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [n, pts] = read_data(filename);
    auto [cost, path] = tsp_annealing(n, pts);
    
    std::cout << cost << '\n';
    for (const auto &v : path) {
        std::cout << v << ' ';
    }
    std::cout << '\n';
}