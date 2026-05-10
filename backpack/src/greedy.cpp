#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <numeric>
#include <random>

#include "CLI/CLI.hpp"

CLI::App app{"backpack_solver"};

struct Item {
    size_t cost;
    size_t weight;
    size_t id;
};

std::tuple<size_t, size_t, std::vector<Item>> read_data(const std::string &filename) {
    std::ifstream file(filename);
    std::string line;
    
    std::getline(file, line);
    std::istringstream line_stream(line);

    size_t n, W; 
    line_stream >> n >> W;
    
    std::vector<Item> items;
    items.reserve(n);
    for (size_t i = 0; i < n; ++i) {
        std::getline(file, line);
        std::istringstream ls(line);
    
        size_t cost, weight;
        ls >> cost >> weight;
        items.push_back({cost, weight, i});
    }

    return {n, W, std::move(items)};
}

std::pair<size_t, std::vector<int>> solve_greedy(size_t n, size_t W, std::vector<Item> items) {
    std::sort(items.begin(), items.end(),
        [](const Item &lhs, const Item &rhs) {
            return static_cast<double>(lhs.cost) * rhs.weight > static_cast<double>(rhs.cost) * lhs.weight;
        }
    );

    size_t weight = 0, cost = 0;
    std::vector<int> taken(n, 0);
    
    for (const auto &item : items) {
        if (weight + item.weight <= W) {
            weight += item.weight;
            cost += item.cost;
            taken[item.id] = 1;
        }
    }
    
    return {cost, taken};
}

std::pair<size_t, std::vector<int>> solve_hybrid(size_t n, size_t W, const std::vector<Item>& items) {
    size_t k = std::min(n, (1 << 30) / W);
    
    std::vector<size_t> dp(W + 1, 0);
    size_t num_words = (W / 64) + 1;
    std::vector<std::vector<uint64_t>> choice(k, std::vector<uint64_t>(num_words, 0));

    for (size_t i = 0; i < k; ++i) {
        size_t w_i = items[i].weight;
        size_t c_i = items[i].cost;
        for (size_t w = W; w >= w_i; --w) {
            if (dp[w - w_i] + c_i > dp[w]) {
                dp[w] = dp[w - w_i] + c_i;
                choice[i][w / 64] |= (1ULL << (w % 64));
            }
        }
    }

    std::vector<int> taken(n, 0);
    size_t current_w = W;
    size_t total_cost = dp[W];
    
    for (int i = static_cast<int>(k) - 1; i >= 0; --i) {
        if ((choice[i][current_w / 64] >> (current_w % 64)) & 1) {
            taken[items[i].id] = 1;
            current_w -= items[i].weight;
        }
    }

    if (k < n) {
        std::vector<Item> remainder;
        for (size_t i = k; i < n; ++i) remainder.push_back(items[i]);
        
        std::sort(remainder.begin(), remainder.end(), [](const Item &a, const Item &b) {
            return static_cast<double>(a.cost) * b.weight > static_cast<double>(b.cost) * a.weight;
        });

        size_t weight_used = W - current_w;
        for (const auto& it : remainder) {
            if (weight_used + it.weight <= W) {
                weight_used += it.weight;
                total_cost += it.cost;
                taken[it.id] = 1;
            }
        }
    }

    return {total_cost, taken};
}

double fractional_bound(size_t level, size_t current_cost, size_t current_weight, size_t n, size_t W, const std::vector<Item>& items) {
    if (current_weight >= W) return 0;
    
    double bound = current_cost;
    size_t total_w = current_weight;

    size_t j = level;
    for (; j < n && total_w + items[j].weight <= W; j++) {
        total_w += items[j].weight;
        bound += items[j].cost;
    }
    
    if (j < n) {
        bound += (W - total_w) * (static_cast<double>(items[j].cost) / items[j].weight);
    }
    
    return bound;
}

void bb_dfs(size_t level, size_t current_cost, size_t current_weight, size_t n, size_t W, 
            const std::vector<Item>& items, std::vector<int>& current_taken, 
            size_t& best_cost, std::vector<int>& best_taken) {
                
    if (current_weight <= W && current_cost > best_cost) {
        best_cost = current_cost;
        best_taken = current_taken;
    }
    
    if (level == n) return;
    
    if (fractional_bound(level, current_cost, current_weight, n, W, items) <= best_cost) {
        return;
    }

    if (current_weight + items[level].weight <= W) {
        current_taken[items[level].id] = 1;
        bb_dfs(level + 1, current_cost + items[level].cost, current_weight + items[level].weight, 
               n, W, items, current_taken, best_cost, best_taken);
        current_taken[items[level].id] = 0;
    }

    bb_dfs(level + 1, current_cost, current_weight, n, W, items, current_taken, best_cost, best_taken);
}

std::pair<size_t, std::vector<int>> solve_bb(size_t n, size_t W, std::vector<Item> items) {
    std::sort(items.begin(), items.end(),
        [](const Item &lhs, const Item &rhs) {
            return static_cast<double>(lhs.cost) * rhs.weight > static_cast<double>(rhs.cost) * lhs.weight;
        }
    );

    std::vector<int> current_taken(n, 0);
    std::vector<int> best_taken(n, 0);
    size_t best_cost = 0;

    bb_dfs(0, 0, 0, n, W, items, current_taken, best_cost, best_taken);

    return {best_cost, best_taken};
}

int main(int argc, char** argv) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);
    
    auto [n, W, items] = read_data(filename);
    auto [cost, result] = solve_bb(n, W, items);

    std::cout << cost << "\n";
    for (size_t i = 0; i < n; ++i) {
        if (result[i]) {
            std::cout << i << ' ';
        }
    }
    std::cout << std::endl;
    
    return 0;
}