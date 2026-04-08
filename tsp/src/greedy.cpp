#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <tuple>
#include "CLI/CLI.hpp"

CLI::App app{"tsp"};

struct Point { double x, y; };

double dist(const Point& a, const Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
}

std::tuple<size_t, std::vector<Point>> read_data(const std::string &filename) {
    std::ifstream file(filename);
    size_t n;
    file >> n;
    
    std::vector<Point> pts(n);
    for (size_t i = 0; i < n; ++i) {
        file >> pts[i].x >> pts[i].y;
    }

    return {n, std::move(pts)};
}

std::pair<double, std::vector<size_t>> tsp_greedy(size_t n, const std::vector<Point>& pts) {
    std::vector<size_t> path;
    std::vector<bool> visited(n, false);
    double total_cost = 0.0;

    size_t current = 0;
    path.push_back(current);
    visited[current] = true;

    for (size_t i = 1; i < n; ++i) {
        size_t best_next = 0;
        double min_dist = 1e15;
        
        for (size_t j = 0; j < n; ++j) {
            if (!visited[j]) {
                double d = dist(pts[current], pts[j]);
                if (d < min_dist) {
                    min_dist = d;
                    best_next = j;
                }
            }
        }
        
        path.push_back(best_next);
        visited[best_next] = true;
        total_cost += min_dist;
        current = best_next;
    }
    
    total_cost += dist(pts[current], pts[0]);
    return {total_cost, path};
}

int main(int argc, char* argv[]) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [n, pts] = read_data(filename);
    auto [cost, path] = tsp_greedy(n, pts);
    
    std::cout << cost << '\n';
    for (const auto &v : path) {
        std::cout << v << ' ';
    }
    std::cout << '\n';
}