#include <vector>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <tuple>
#include <limits>
#include <random>

#include "CLI/CLI.hpp"

CLI::App app{"facility"};

struct Facility { double cost, cap, x, y; bool open = false; double used_cap = 0; };
struct Customer { double demand, x, y; };

double dist(double x1, double y1, double x2, double y2) {
    double dx = x1 - x2;
    double dy = y1 - y2;
    return std::sqrt(dx * dx + dy * dy);
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
        double best_score = std::numeric_limits<double>::infinity();

        for (size_t j = 0; j < N; ++j) {
            if (facs[j].used_cap + custs[i].demand <= facs[j].cap) {
                double d = dist(custs[i].x, custs[i].y, facs[j].x, facs[j].y);
                double score = d + (facs[j].open ? 0 : facs[j].cost);

                if (score < best_score) {
                    best_score = score;
                    best_fac = (int)j;
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

std::pair<double, std::vector<int>> facility_regret2(size_t N, size_t M, std::vector<Facility> facs, const std::vector<Customer>& custs) {
    std::vector<int> assignments(M, -1);
    std::vector<bool> assigned(M, false);
    double total_cost = 0.0;

    for (size_t step = 0; step < M; ++step) {
        int best_cust = -1;
        double max_regret = -1.0;
        int best_fac_for_cust = -1;

        for (size_t i = 0; i < M; ++i) {
            if (assigned[i]) continue;

            int first_fac = -1;
            int second_fac = -1;
            double first_score = std::numeric_limits<double>::infinity();
            double second_score = std::numeric_limits<double>::infinity();

            for (size_t j = 0; j < N; ++j) {
                if (facs[j].used_cap + custs[i].demand <= facs[j].cap) {
                    double d = dist(custs[i].x, custs[i].y, facs[j].x, facs[j].y);
                    double score = d + (facs[j].open ? 0 : facs[j].cost);

                    if (score < first_score) {
                        second_score = first_score;
                        second_fac = first_fac;
                        first_score = score;
                        first_fac = (int)j;
                    } else if (score < second_score) {
                        second_score = score;
                        second_fac = (int)j;
                    }
                }
            }

            if (first_fac == -1) continue;

            double regret = 0.0;
            if (second_fac == -1) {
                regret = std::numeric_limits<double>::infinity();
            } else {
                regret = second_score - first_score;
            }

            if (regret > max_regret || best_cust == -1) {
                max_regret = regret;
                best_cust = (int)i;
                best_fac_for_cust = first_fac;
            }
        }

        if (best_cust == -1) break;

        assignments[best_cust] = best_fac_for_cust;
        assigned[best_cust] = true;
        facs[best_fac_for_cust].used_cap += custs[best_cust].demand;
        if (!facs[best_fac_for_cust].open) {
            facs[best_fac_for_cust].open = true;
        }
    }

    std::vector<bool> fac_open_check(N, false);
    for (size_t i = 0; i < M; ++i) {
        int f = assignments[i];
        if (f != -1) {
            if (!fac_open_check[f]) {
                fac_open_check[f] = true;
                total_cost += facs[f].cost;
            }
            total_cost += dist(custs[i].x, custs[i].y, facs[f].x, facs[f].y);
        }
    }

    return {total_cost, assignments};
}

std::pair<double, std::vector<int>> facility_annealing(size_t N, size_t M, const std::vector<Facility>& facs, const std::vector<Customer>& custs) {
    auto start_time = std::chrono::steady_clock::now();
    double time_limit = 60.0;

    std::vector<std::vector<double>> dist_matrix(M, std::vector<double>(N));
    for (size_t i = 0; i < M; ++i) {
        for (size_t j = 0; j < N; ++j) {
            dist_matrix[i][j] = dist(custs[i].x, custs[i].y, facs[j].x, facs[j].y);
        }
    }

    auto [init_cost, init_assign] = facility_regret2(N, M, facs, custs);

    std::vector<int> current_assign = init_assign;
    double current_cost = init_cost;

    std::vector<int> best_assign = current_assign;
    double best_cost = current_cost;

    std::vector<double> used_cap(N, 0.0);
    std::vector<int> open_count(N, 0);
    for (size_t i = 0; i < M; ++i) {
        int f = current_assign[i];
        if (f != -1) {
            used_cap[f] += custs[i].demand;
            open_count[f]++;
        }
    }

    std::vector<size_t> open_facs; 
    open_facs.reserve(N);
    std::vector<size_t> affected_custs;
    affected_custs.reserve(M);
    std::vector<int> new_assignments;
    new_assignments.reserve(M);
    std::vector<double> temp_used_cap(N);
    std::vector<int> temp_open_count(N);

    std::mt19937 rng(137);
    std::uniform_real_distribution<double> dist_u(0.0, 1.0);
    std::uniform_int_distribution<size_t> rand_cust(0, M - 1);
    std::uniform_int_distribution<size_t> rand_fac(0, N - 1);

    double T = 900;
    const double alpha = 0.99995;
    size_t max_iterations = 2500000;

    for (size_t iter = 0; iter < max_iterations; ++iter) {
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - start_time).count();
        if (elapsed > time_limit) break;

        double prob = dist_u(rng);
        if (prob < 0.4) {
            size_t i = rand_cust(rng);
            size_t new_f = rand_fac(rng);
            int old_f = current_assign[i];

            if (old_f == (int)new_f || old_f == -1) continue;

            if (used_cap[new_f] + custs[i].demand <= facs[new_f].cap) {
                double delta = dist_matrix[i][new_f] - dist_matrix[i][old_f];
                if (open_count[old_f] == 1) delta -= facs[old_f].cost;
                if (open_count[new_f] == 0) delta += facs[new_f].cost;

                if (delta < 0 || dist_u(rng) < std::exp(-delta / T)) {
                    current_assign[i] = (int)new_f;
                    used_cap[old_f] -= custs[i].demand;
                    used_cap[new_f] += custs[i].demand;
                    open_count[old_f]--;
                    open_count[new_f]++;
                    current_cost += delta;

                    if (current_cost < best_cost) {
                        best_cost = current_cost;
                        best_assign = current_assign;
                    }
                }
            }
        } else if (prob < 0.8) {
            size_t i1 = rand_cust(rng);
            size_t i2 = rand_cust(rng);
            if (i1 == i2) continue;

            int f1 = current_assign[i1];
            int f2 = current_assign[i2];
            if (f1 == f2 || f1 == -1 || f2 == -1) continue;

            if (used_cap[f1] - custs[i1].demand + custs[i2].demand <= facs[f1].cap &&
                used_cap[f2] - custs[i2].demand + custs[i1].demand <= facs[f2].cap) {
                
                double delta = dist_matrix[i1][f2] + dist_matrix[i2][f1] - dist_matrix[i1][f1] - dist_matrix[i2][f2];

                if (delta < 0 || dist_u(rng) < std::exp(-delta / T)) {
                    current_assign[i1] = f2;
                    current_assign[i2] = f1;
                    used_cap[f1] = used_cap[f1] - custs[i1].demand + custs[i2].demand;
                    used_cap[f2] = used_cap[f2] - custs[i2].demand + custs[i1].demand;
                    current_cost += delta;

                    if (current_cost < best_cost) {
                        best_cost = current_cost;
                        best_assign = current_assign;
                    }
                }
            }
        } else {
            open_facs.clear();
            for (size_t j = 0; j < N; ++j) {
                if (open_count[j] > 0) open_facs.push_back(j);
            }

            if (open_facs.size() <= 1) continue;

            std::uniform_int_distribution<size_t> rand_open(0, open_facs.size() - 1);
            size_t closed_f = open_facs[rand_open(rng)];

            affected_custs.clear();
            for (size_t i = 0; i < M; ++i) {
                if (current_assign[i] == (int)closed_f) {
                    affected_custs.push_back(i);
                }
            }

            std::copy(used_cap.begin(), used_cap.end(), temp_used_cap.begin());
            std::copy(open_count.begin(), open_count.end(), temp_open_count.begin());
            new_assignments.resize(affected_custs.size());

            double delta = -facs[closed_f].cost;
            temp_used_cap[closed_f] = 0.0;
            temp_open_count[closed_f] = 0;

            bool feasible = true;

            for (size_t idx = 0; idx < affected_custs.size(); ++idx) {
                size_t i = affected_custs[idx];
                delta -= dist_matrix[i][closed_f];

                int best_new_f = -1;
                double best_score = std::numeric_limits<double>::infinity();

                for (size_t j = 0; j < N; ++j) {
                    if (j == closed_f) continue;
                    if (temp_used_cap[j] + custs[i].demand <= facs[j].cap) {
                        double score = dist_matrix[i][j] + (temp_open_count[j] == 0 ? facs[j].cost : 0);
                        if (score < best_score) {
                            best_score = score;
                            best_new_f = (int)j;
                        }
                    }
                }

                if (best_new_f == -1) {
                    feasible = false;
                    break;
                }

                new_assignments[idx] = best_new_f;
                delta += dist_matrix[i][best_new_f];
                if (temp_open_count[best_new_f] == 0) delta += facs[best_new_f].cost;

                temp_used_cap[best_new_f] += custs[i].demand;
                temp_open_count[best_new_f]++;
            }

            if (feasible) {
                if (delta < 0 || dist_u(rng) < std::exp(-delta / T)) {
                    for (size_t idx = 0; idx < affected_custs.size(); ++idx) {
                        current_assign[affected_custs[idx]] = new_assignments[idx];
                    }
                    std::copy(temp_used_cap.begin(), temp_used_cap.end(), used_cap.begin());
                    std::copy(temp_open_count.begin(), temp_open_count.end(), open_count.begin());
                    current_cost += delta;

                    if (current_cost < best_cost) {
                        best_cost = current_cost;
                        best_assign = current_assign;
                    }
                }
            }
        }
    
        T *= alpha;
    }

    return {best_cost, best_assign};
}

int main(int argc, char* argv[]) {
    std::cout << std::fixed << std::setprecision(6);

    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [N, M, facs, custs] = read_data(filename);
    auto [cost, assignments] = facility_annealing(N, M, facs, custs);

    std::cout << cost << '\n';
    for (const auto &f : assignments) {
        std::cout << f << ' ';
    }
    std::cout << '\n';
}