#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <tuple>
#include "CLI/CLI.hpp"

CLI::App app{"vrp"};

struct Point { double demand, x, y; bool visited = false; };

double dist(const Point& a, const Point& b) {
    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2));
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

std::pair<double, std::vector<std::vector<size_t>>> vrp_greedy(size_t n, size_t v, double c, std::vector<Point> pts) {
    double total_cost = 0.0;
    std::vector<std::vector<size_t>> routes(v);
    pts[0].visited = true;

    for (size_t i = 0; i < v; ++i) {
        double current_cap = c;
        size_t current_pos = 0;
        
        while (true) {
            size_t best_next = 0;
            double min_dist = 1e15;
            
            for (size_t j = 1; j < n; ++j) {
                if (!pts[j].visited && pts[j].demand <= current_cap) {
                    double d = dist(pts[current_pos], pts[j]);
                    if (d < min_dist) {
                        min_dist = d;
                        best_next = j;
                    }
                }
            }
            
            if (best_next == 0) break;
            
            routes[i].push_back(best_next);
            pts[best_next].visited = true;
            current_cap -= pts[best_next].demand;
            total_cost += min_dist;
            current_pos = best_next;
        }
        
        if (current_pos != 0) {
            total_cost += dist(pts[current_pos], pts[0]);
        }
    }

    return {total_cost, routes};
}

int main(int argc, char* argv[]) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [n, v, c, pts] = read_data(filename);
    auto [cost, routes] = vrp_greedy(n, v, c, pts);
    
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