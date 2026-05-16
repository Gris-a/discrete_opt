#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <random>
#include <chrono>
#include <cmath>

#include "CLI/CLI.hpp"

CLI::App app{"set_cover_solver"};

struct Set {
    size_t id;
    size_t cost;
    std::vector<size_t> elements; 
};

std::tuple<size_t, size_t, std::vector<Set>> read_data(const std::string &filename) {
    std::ifstream file(filename);
    std::string line;
    if (!std::getline(file, line)) return {0, 0, {}};
    
    std::istringstream header_stream(line);
    size_t n, m; 
    header_stream >> n >> m;

    std::vector<Set> sets(m);
    for (size_t i = 0; i < m; ++i) {
        if (!std::getline(file, line)) break;
        std::istringstream line_stream(line);
        
        size_t cost;
        line_stream >> cost;
        
        std::vector<size_t> elements;
        size_t el;
        while (line_stream >> el) {
            elements.push_back(el);
        }
        sets[i] = {i, cost, std::move(elements)};
    }
    return {n, m, std::move(sets)};
}

void prune_redundant(size_t m, const std::vector<Set>& sets, std::vector<int>& taken, std::vector<size_t>& cover_count, size_t& current_cost) {
    for (size_t i = 0; i < m; ++i) {
        if (!taken[i]) continue;
        
        bool can_remove = true;
        for (size_t el : sets[i].elements) {
            if (cover_count[el] <= 1) {
                can_remove = false;
                break;
            }
        }
        
        if (can_remove) {
            taken[i] = 0;
            current_cost -= sets[i].cost;
            for (size_t el : sets[i].elements) {
                cover_count[el]--;
            }
        }
    }
}

std::pair<size_t, std::vector<int>> solve_greedy(size_t n, size_t m, const std::vector<Set> &sets) {
    std::vector<int> taken(m, 0);
    std::vector<size_t> cover_count(n, 0);
    size_t uncovered = n;
    size_t total_cost = 0;

    while (uncovered > 0) {
        int best_i = -1;
        double best_ratio = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < m; ++i) {
            if (taken[i]) continue;

            size_t fresh_elements = 0;
            for (size_t el : sets[i].elements) {
                if (cover_count[el] == 0) fresh_elements++;
            }

            if (fresh_elements > 0) {
                double ratio = static_cast<double>(sets[i].cost) / fresh_elements;
                if (ratio < best_ratio) {
                    best_ratio = ratio;
                    best_i = i;
                }
            }
        }

        if (best_i == -1) break;

        taken[best_i] = 1;
        total_cost += sets[best_i].cost;
        for (size_t el : sets[best_i].elements) {
            if (cover_count[el] == 0) uncovered--;
            cover_count[el]++;
        }
    }

    prune_redundant(m, sets, taken, cover_count, total_cost);
    return {total_cost, taken};
}

std::pair<size_t, std::vector<int>> solve_annealing(size_t n, size_t m, const std::vector<Set> &sets) {
    auto [best_cost, best_taken] = solve_greedy(n, m, sets);
    
    std::vector<int> current_taken = best_taken;
    size_t current_cost = best_cost;
    
    std::vector<size_t> cover_count(n, 0);
    size_t uncovered = n;
    for (size_t i = 0; i < m; ++i) {
        if (current_taken[i]) {
            for (size_t el : sets[i].elements) {
                if (cover_count[el] == 0) uncovered--;
                cover_count[el]++;
            }
        }
    }

    std::vector<int> trial_taken(m);
    std::vector<size_t> trial_cover(n);
    std::vector<size_t> active_sets;
    active_sets.reserve(m);

    std::mt19937_64 rng(137);
    double T = 10.0;
    const double alpha = 0.9995;
    size_t max_iterations = 100000;
    auto start_time = std::chrono::steady_clock::now();

    for (int iter = 0; iter < max_iterations; ++iter) {
        if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(50)) break;

        active_sets.clear();
        for (size_t i = 0; i < m; ++i) {
            if (current_taken[i]) active_sets.push_back(i);
        }
        if (active_sets.empty()) break;

        std::copy(current_taken.begin(), current_taken.end(), trial_taken.begin());
        std::copy(cover_count.begin(), cover_count.end(), trial_cover.begin());
        size_t trial_uncovered = uncovered;
        size_t trial_cost = current_cost;

        size_t k = std::min<size_t>(std::uniform_int_distribution<size_t>(1, 3)(rng), active_sets.size());
        for (size_t j = 0; j < k; ++j) {
            size_t idx = std::uniform_int_distribution<size_t>(0, active_sets.size() - 1)(rng);
            size_t set_id = active_sets[idx];
            if (trial_taken[set_id]) {
                trial_taken[set_id] = 0;
                trial_cost -= sets[set_id].cost;
                for (size_t el : sets[set_id].elements) {
                    trial_cover[el]--;
                    if (trial_cover[el] == 0) trial_uncovered++;
                }
            }
        }

        while (trial_uncovered > 0) {
            int best_i = -1;
            double best_ratio = std::numeric_limits<double>::infinity();

            for (size_t i = 0; i < m; ++i) {
                if (trial_taken[i]) continue;
                size_t fresh = 0;
                for (size_t el : sets[i].elements) {
                    if (trial_cover[el] == 0) fresh++;
                }
                if (fresh > 0) {
                    double ratio = static_cast<double>(sets[i].cost) / fresh;
                    if (ratio < best_ratio) {
                        best_ratio = ratio;
                        best_i = i;
                    }
                }
            }

            if (best_i == -1) break;

            trial_taken[best_i] = 1;
            trial_cost += sets[best_i].cost;
            for (size_t el : sets[best_i].elements) {
                if (trial_cover[el] == 0) trial_uncovered--;
                trial_cover[el]++;
            }
        }

        if (trial_uncovered > 0) continue;

        prune_redundant(m, sets, trial_taken, trial_cover, trial_cost);

        double delta = static_cast<double>(trial_cost) - static_cast<double>(current_cost);
        if (delta < 0 || std::uniform_real_distribution<double>(0.0, 1.0)(rng) < std::exp(-delta / T)) {
            current_taken = trial_taken;
            cover_count = trial_cover;
            uncovered = trial_uncovered;
            current_cost = trial_cost;

            if (current_cost < best_cost) {
                best_cost = current_cost;
                best_taken = current_taken;
            }
        }

        T *= alpha;
    }

    return {best_cost, best_taken};
}


int main(int argc, char** argv) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);
    
    CLI11_PARSE(app, argc, argv);

    auto [n, m, sets] = read_data(filename);
    auto [cost, taken] = solve_annealing(n, m, sets);

    std::cout << cost << "\n";
    for (size_t i = 0; i < m; ++i) {
        if (taken[i]) {
            std::cout << i << ' ';
        }
    }
    std::cout << std::endl;

    return 0;
}