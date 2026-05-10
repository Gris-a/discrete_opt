#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <unordered_set>
#include <queue>
#include <random>
#include <chrono>

#include "CLI/CLI.hpp"

CLI::App app{"set_cover_solver"};

struct Set {
    size_t id;
    size_t cost;
    std::unordered_set<size_t> elements;
};

struct SubsetEntry {
    size_t id;
    size_t cost;
    size_t uncovered_count;

    bool operator<(const SubsetEntry &other) const {
        return cost * other.uncovered_count > other.cost * uncovered_count;
    }
};

std::tuple<size_t, size_t, std::vector<Set>>
read_data(const std::string &filename) {
    std::ifstream file(filename);

    std::string line;
    std::getline(file, line);
    std::istringstream header_stream(line);
    
    size_t n, m; 
    header_stream >> n >> m;

    std::vector<Set> sets;
    sets.reserve(m);

    for (size_t i = 0; i < m; ++i) {
        if (!std::getline(file, line)) break;
        std::istringstream line_stream(line);
        
        size_t cost;
        line_stream >> cost;
        
        std::unordered_set<size_t> elements;
        size_t el;
        while (line_stream >> el) {
            elements.insert(el);
        }
        sets.push_back({i, cost, std::move(elements)});
    }

    return {n, m, std::move(sets)};
}


std::pair<size_t, std::vector<int>> solve_greedy(size_t n, size_t m, const std::vector<Set> &sets) {
    std::unordered_set<size_t> uncovered;
    for (size_t i = 0; i < n; ++i) uncovered.insert(i);
    
    size_t total_cost = 0;
    std::vector<int> taken(m, 0);

    while (!uncovered.empty()) {
        int best_set_idx = -1;
        double best_ratio = std::numeric_limits<double>::max();

        for (size_t i = 0; i < m; ++i) {
            if (taken[i]) continue;

            int new_elements = 0;
            for (size_t el : sets[i].elements) {
                if (uncovered.count(el)) new_elements++;
            }

            if (new_elements > 0) {
                double ratio = static_cast<double>(sets[i].cost) / new_elements;
                if (ratio < best_ratio) {
                    best_ratio = ratio;
                    best_set_idx = i;
                }
            }
        }

        if (best_set_idx == -1) break;

        taken[best_set_idx] = 1;
        total_cost += sets[best_set_idx].cost;
        for (size_t el : sets[best_set_idx].elements) {
            uncovered.erase(el);
        }
    }

    return {total_cost, taken};
}

int main(int argc, char** argv) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);
    
    CLI11_PARSE(app, argc, argv);

    auto [n, m, sets] = read_data(filename);
    auto [cost, taken] = solve_greedy(n, m, sets);

    std::cout << cost << "\n";
    for (size_t i = 0; i < m; ++i) {
        if (taken[i]) {
            std::cout << i << ' ';
        }
    }
    std::cout << std::endl;

    return 0;
}
