#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>

#include "CLI/CLI.hpp"

CLI::App app{"compiler"};


struct Item {
    size_t weight;
    size_t cost;
};


std::tuple<size_t, size_t, std::vector<Item>>
read_data(const std::string &filename) {
    std::ifstream file(filename);
    std::string line;
    
    std::getline(file, line);
    std::istringstream line_stream(line);

    size_t n, W; line_stream >> n >> W;
    
    std::vector<Item> items;
    for (size_t i = 0; i < n; ++i) {
        std::getline(file, line);
        std::istringstream line_stream(line);
    
        items.emplace_back();
        line_stream >> items.back().cost >> items.back().weight;
    }

    return {n, W, std::move(items)};
}

size_t backpack_greedy(size_t n, size_t W, std::vector<Item> items) {
    std::sort(items.begin(), items.end(),
        [](const Item &lhs, const Item &rhs) {
            return lhs.cost * rhs.weight > rhs.cost * lhs.weight;
        }
    );

    size_t weight = 0, cost = 0;
    for (const auto &item : items) {
        if (weight + item.weight <= W) {
            weight += item.weight, cost += item.cost;
        }
    }
    
    return cost;
}

int main(int argc, char** argv) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);
    
    auto [n, W, items] = read_data(filename);
    std::cout << backpack_greedy(n, W, items) << std::endl;
}