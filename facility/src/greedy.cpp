#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <tuple>
#include "CLI/CLI.hpp"

CLI::App app{"facility"};

struct Facility { double cost, cap, x, y; bool open = false; double used_cap = 0; };
struct Customer { double demand, x, y; };

double dist(double x1, double y1, double x2, double y2) {
    return std::sqrt(std::pow(x1 - x2, 2) + std::pow(y1 - y2, 2));
}

std::tuple<size_t, size_t, std::vector<Facility>, std::vector<Customer>> read_data(const std::string &filename) {
    std::ifstream file(filename);
    size_t N, M; 
    file >> N >> M;
    
    std::vector<Facility> facs(N);
    for (size_t i = 0; i < N; ++i) {
        file >> facs[i].cost >> facs[i].cap >> facs[i].x >> facs[i].y;
    }
    
    std::vector<Customer> custs(M);
    for (size_t i = 0; i < M; ++i) {
        file >> custs[i].demand >> custs[i].x >> custs[i].y;
    }

    return {N, M, std::move(facs), std::move(custs)};
}

std::pair<double, std::vector<int>> facility_greedy(size_t N, size_t M, std::vector<Facility> facs, const std::vector<Customer>& custs) {
    std::vector<int> assignments(M, -1);
    double total_cost = 0.0;

    for (size_t i = 0; i < M; ++i) {
        int best_fac = -1;
        double best_score = 1e15;

        for (size_t j = 0; j < N; ++j) {
            if (facs[j].used_cap + custs[i].demand <= facs[j].cap) {
                double d = dist(custs[i].x, custs[i].y, facs[j].x, facs[j].y);
                double score = d + (facs[j].open ? 0 : facs[j].cost); 
                
                if (score < best_score) {
                    best_score = score;
                    best_fac = j;
                }
            }
        }
        
        if (best_fac != -1) {
            assignments[i] = best_fac;
            facs[best_fac].used_cap += custs[i].demand;
            if (!facs[best_fac].open) {
                facs[best_fac].open = true;
                total_cost += facs[best_fac].cost;
            }
            total_cost += dist(custs[i].x, custs[i].y, facs[best_fac].x, facs[best_fac].y);
        }
    }

    return {total_cost, assignments};
}

int main(int argc, char* argv[]) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [N, M, facs, custs] = read_data(filename);
    auto [cost, assignments] = facility_greedy(N, M, facs, custs);
    
    std::cout << cost << '\n';
    for (const auto &f : assignments) {
        std::cout << f << ' ';
    }
    std::cout << '\n';
}