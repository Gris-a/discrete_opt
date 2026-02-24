#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <chrono>

struct SubsetEntry {
    size_t id, cost, size;

    friend bool operator==(const SubsetEntry &lhs, const SubsetEntry &rhs) {
        return lhs.size * rhs.cost == rhs.size * lhs.cost;
    }

    friend auto operator<=>(const SubsetEntry &lhs, const SubsetEntry &rhs) {
        return (lhs.size * rhs.cost <=> rhs.size * lhs.cost);
    }
};

std::tuple<std::vector<size_t>, size_t, double>
set_cover_greedy(size_t n, const std::vector<std::unordered_set<size_t>> &subsets, const std::vector<size_t> &costs) {
    std::vector<size_t> covering; size_t total_cost = 0;
    auto start = std::chrono::high_resolution_clock::now();

    std::vector<std::vector<size_t>> subsets_covering(n);
    for (size_t id = 0; id < subsets.size(); ++id) {
        for (const auto &element : subsets[id]) {
            subsets_covering[element].push_back(id);
        }
    }

    std::unordered_set<size_t> update_indexes;
    std::vector<size_t> subsets_uncovered(subsets.size());
    for (size_t id = 0; id < subsets.size(); ++id) {
        subsets_uncovered[id] = subsets[id].size();
    }

    std::priority_queue<SubsetEntry> heap;
    for (size_t id = 0; id < subsets.size(); ++id) {
        heap.push(SubsetEntry{id, costs[id], subsets_uncovered[id]});
    }

    std::vector<bool> coverage(n, false);
    for (size_t remaining = n; remaining;) {
        SubsetEntry entry;

        while (true) {
            entry = heap.top(); heap.pop();
            if ((entry.size != 0) && (entry.size == subsets_uncovered[entry.id])) {
                break;
            }
        }

        covering.push_back(entry.id);
        total_cost += costs[entry.id];

        for (const auto &element : subsets[entry.id]) {
            if (coverage[element]) continue;

            coverage[element] = true;
            --remaining;
            
            for (const auto &id : subsets_covering[element]) {
                update_indexes.insert(id);
                --subsets_uncovered[id];
            }
        }

        for (const auto &id : update_indexes) {
            heap.push(SubsetEntry{id, costs[id], subsets_uncovered[id]});
        }
        update_indexes.clear();
    }

    auto end = std::chrono::high_resolution_clock::now();
    double elapsed = std::chrono::duration<double>(end - start).count();

    return {std::move(covering), total_cost, elapsed};
}

std::tuple<size_t, std::vector<std::unordered_set<size_t>>, std::vector<size_t>>
read_instance(const std::string &filename) {
    std::ifstream file(filename);
    std::string line;
    
    std::getline(file, line);
    std::istringstream line_stream(line);

    size_t n, m; line_stream >> n >> m;

    std::vector<std::unordered_set<size_t>> subsets(m);
    std::vector<size_t> costs(m);
    
    for (size_t id = 0; id < m; ++id) {
        std::getline(file, line);
        std::istringstream line_stream(line);
    
        line_stream >> costs[id];

        size_t element;
        while (line_stream >> element) {
            subsets[id].insert(element);
        }
    }

    return {n, std::move(subsets), std::move(costs)};
}

struct Test {
    std::string filename;
    size_t bound1, bound2;
};

static std::vector<Test> test_suit = {
    Test{"data/sc_157_0", 130000, 94402},
    Test{"data/sc_330_0", 29, 24},
    Test{"data/sc_1000_11", 240, 147},
    Test{"data/sc_5000_1", 70, 31},
    Test{"data/sc_10000_2", 280, 167},
    Test{"data/sc_10000_5", 120, 64},
};

const static size_t bound0_score = 0;
const static size_t bound1_score = 3;
const static size_t bound2_score = 5;

int
main(int argc, char **argv) {
    if (argc > 1) {
        test_suit.clear();
        test_suit.emplace_back(argv[1], std::atoll(argv[2]), std::atoll(argv[3]));
    }

    for (const auto &test : test_suit) {
        std::cout << "running test " << test.filename << "\t";

        auto [n, subsets, costs] = read_instance(test.filename);
        auto [covering, cost, elapsed] = set_cover_greedy(n, subsets, costs);
        
        std::cout << elapsed << "s\t";
        if (cost <= test.bound2) {
            std::cout << bound2_score << "/" << bound2_score << '\n';
        } else if (cost < test.bound1) {
            std::cout << bound1_score << "/" << bound2_score << '\n';
        } else {
            std::cout << bound0_score << "/" << bound2_score << '\n';
        }

        std::cout << "cost = " << cost << '\n';
    }
}