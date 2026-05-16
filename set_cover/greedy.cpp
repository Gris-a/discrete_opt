#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>
#include <algorithm>
#include <queue>
#include <random>
#include <cmath>

#include "CLI/CLI.hpp"

CLI::App app{"coloring"};

std::tuple<size_t, size_t, std::vector<std::vector<size_t>>>
read_data(const std::string &filename) {
    std::ifstream file(filename);
    std::string line;
    
    std::getline(file, line);
    std::istringstream line_stream(line);
    size_t n, m; line_stream >> n >> m;
    
    std::vector<std::vector<size_t>> adj(n);
    for (size_t i = 0; i < m; ++i) {
        std::getline(file, line);
        std::istringstream line_stream(line);
        size_t x, y; line_stream >> x >> y;

        adj[x].push_back(y);
        adj[y].push_back(x);
    }

    return {n, m, std::move(adj)};
}

std::pair<size_t, std::vector<ssize_t>> greedy(size_t n, const std::vector<std::vector<size_t>> &adj) {
    std::vector<ssize_t> coloring(n, -1);
    ssize_t max_color = -1;

    for (size_t u = 0; u < n; ++u) {
        std::vector<bool> used(n, false);

        for (size_t v : adj[u]) {
            if (coloring[v] != -1) {
                used[coloring[v]] = true;
            }
        }

        ssize_t color = 0;
        while (used[color]) ++color;
        
        coloring[u] = color;
        max_color = std::max(max_color, color);
    }

    return {max_color, coloring};
}

std::pair<size_t, std::vector<ssize_t>> dsatur(size_t n, const std::vector<std::vector<size_t>> &adj) {
    std::vector<ssize_t> color(n, -1);
    std::vector<int> degree(n);
    std::vector<std::vector<bool>> adj_color_used(n, std::vector<bool>(n, false));
    std::vector<int> saturation(n, 0);
    std::vector<bool> uncolored(n, true);

    for(size_t i = 0; i < n; ++i) degree[i] = adj[i].size();

    ssize_t max_c = -1;

    for(size_t step = 0; step < n; ++step) {
        int best_u = -1;
        int max_sat = -1;
        int max_deg = -1;

        for(size_t u = 0; u < n; ++u) {
            if(uncolored[u]) {
                if(saturation[u] > max_sat || (saturation[u] == max_sat && degree[u] > max_deg)) {
                    max_sat = saturation[u];
                    max_deg = degree[u];
                    best_u = u;
                }
            }
        }

        ssize_t c = 0;
        while(adj_color_used[best_u][c]) c++;
        
        color[best_u] = c;
        uncolored[best_u] = false;
        max_c = std::max(max_c, c);

        for(size_t v : adj[best_u]) {
            if(uncolored[v]) {
                if(!adj_color_used[v][c]) {
                    adj_color_used[v][c] = true;
                    saturation[v]++;
                }
                degree[v]--; 
            }
        }
    }
    return {max_c, color};
}

std::pair<size_t, std::vector<ssize_t>> rlf(size_t n, const std::vector<std::vector<size_t>> &adj) {
    std::vector<ssize_t> color(n, -1);
    std::vector<bool> uncolored(n, true);
    size_t uncolored_count = n;
    ssize_t current_color = 0;

    while(uncolored_count > 0) {
        std::vector<bool> V(n, false);
        std::vector<bool> U(n, false);

        for(size_t i=0; i<n; ++i) if(uncolored[i]) V[i] = true;

        int best_v = -1;
        int max_deg = -1;
        for(size_t i = 0; i < n; ++i) {
            if(V[i]) {
                int deg = 0;
                for(size_t u : adj[i]) if(uncolored[u]) deg++;
                if(deg > max_deg) { max_deg = deg; best_v = i; }
            }
        }

        while(best_v != -1) {
            color[best_v] = current_color;
            uncolored[best_v] = false;
            uncolored_count--;
            V[best_v] = false;

            for(size_t u : adj[best_v]) {
                if(V[u]) { V[u] = false; U[u] = true; }
            }

            best_v = -1;
            int max_u_deg = -1;
            for(size_t i=0; i<n; ++i) {
                if(V[i]) {
                    int u_deg = 0;
                    for(size_t u : adj[i]) if(U[u]) u_deg++;
                    if(u_deg > max_u_deg) { max_u_deg = u_deg; best_v = i; }
                }
            }
        }
        current_color++;
    }
    return {current_color - 1, color};
}

void apply_kempe_chains(size_t n, const std::vector<std::vector<size_t>> &adj, size_t &max_color, std::vector<ssize_t> &color) {
    bool improved = true;
    while(improved) {
        improved = false;
        std::vector<size_t> target_vertices;
        
        for(size_t i = 0; i < n; ++i) {
            if(color[i] == max_color) target_vertices.push_back(i);
        }

        if (target_vertices.empty()) break;

        bool any_resolved = false;
        size_t resolved_count = 0;

        for(size_t v : target_vertices) {
            bool v_resolved = false;
            
            for(ssize_t c = 0; c < max_color; ++c) {
                bool found = false;
                for(size_t u : adj[v]) {
                    if(color[u] == c) { found = true; break; }
                }
                if(!found) {
                    color[v] = c;
                    v_resolved = true;
                    break;
                }
            }

            if(v_resolved) {
                any_resolved = true;
                resolved_count++;
                continue;
            }

            for(ssize_t c1 = 0; c1 < max_color && !v_resolved; ++c1) {
                std::vector<size_t> neighbors_c1;
                for(size_t u : adj[v]) {
                    if(color[u] == c1) neighbors_c1.push_back(u);
                }

                if(neighbors_c1.size() == 1) {
                    size_t start_node = neighbors_c1[0];
                    
                    for(ssize_t c2 = 0; c2 < max_color; ++c2) {
                        if(c1 == c2) continue;
                        
                        std::vector<size_t> chain;
                        std::vector<bool> visited(n, false);
                        std::queue<size_t> q;
                        
                        q.push(start_node);
                        visited[start_node] = true;
                        
                        bool conflict = false;

                        while(!q.empty()) {
                            size_t curr = q.front();
                            q.pop();
                            chain.push_back(curr);

                            if (curr != start_node && color[curr] == c2) {
                                for(size_t adj_v : adj[v]) {
                                    if(curr == adj_v) { conflict = true; break; }
                                }
                            }
                            if(conflict) break;

                            for(size_t nxt : adj[curr]) {
                                if(nxt == v) continue;
                                if((color[nxt] == c1 || color[nxt] == c2) && !visited[nxt]) {
                                    visited[nxt] = true;
                                    q.push(nxt);
                                }
                            }
                        }

                        if(!conflict) {
                            for(size_t node : chain) {
                                if(color[node] == c1) color[node] = c2;
                                else color[node] = c1;
                            }
                            color[v] = c1;
                            v_resolved = true;
                            break;
                        }
                    }
                }
            }
            
            if (v_resolved) {
                any_resolved = true;
                resolved_count++;
            }
        }

        if (any_resolved) {
            improved = true;
            if (resolved_count == target_vertices.size()) {
                max_color--;
            }
        }
    }
}

bool apply_simulated_annealing(size_t n, const std::vector<std::vector<size_t>> &adj, size_t k, std::vector<ssize_t> &coloring) {
    std::mt19937 rng(42);
    
    std::vector<std::vector<int>> adj_conflicts(n, std::vector<int>(k, 0));
    
    for (size_t i = 0; i < n; ++i) {
        if (coloring[i] >= (ssize_t)k) {
            coloring[i] = rng() % k;
        }
    }

    for (size_t u = 0; u < n; ++u) {
        for (size_t v : adj[u]) {
            adj_conflicts[u][coloring[v]]++;
        }
    }

    int E = 0;
    for (size_t u = 0; u < n; ++u) {
        E += adj_conflicts[u][coloring[u]];
    }
    E /= 2;

    double T = 100;
    double alpha = 0.99995;
    size_t max_iters = 500000;
    size_t max_attempts = 100;

    for (int step = 0; step < max_iters; ++step) {
        if (E == 0) return true;

        size_t u = rng() % n;
        int attempts = 0;
        while (adj_conflicts[u][coloring[u]] == 0 && attempts < max_attempts) {
            u = rng() % n;
            attempts++;
        }
        if (adj_conflicts[u][coloring[u]] == 0) {
            for (size_t i = 0; i < n; ++i) {
                if (adj_conflicts[i][coloring[i]] > 0) { u = i; break; }
            }
        }

        ssize_t old_color = coloring[u];
        ssize_t new_color = rng() % k;
        while (new_color == old_color && k > 1) {
            new_color = rng() % k;
        }

        int delta = adj_conflicts[u][new_color] - adj_conflicts[u][old_color];

        bool accept = false;
        if (delta <= 0) {
            accept = true;
        } else {
            double p = std::exp(-delta / T);
            std::uniform_real_distribution<double> dist(0.0, 1.0);
            if (dist(rng) < p) {
                accept = true;
            }
        }

        if (accept) {
            for (size_t v : adj[u]) {
                adj_conflicts[v][old_color]--;
                adj_conflicts[v][new_color]++;
            }
            coloring[u] = new_color;
            E += delta;
        }

        T *= alpha;
    }

    return (E == 0);
}

std::pair<size_t, std::vector<ssize_t>> kempe(size_t n, const std::vector<std::vector<size_t>> &adj) {
    auto [best_max, best_color] = dsatur(n, adj);
    auto [max_rlf, color_rlf] = rlf(n, adj);

    if (max_rlf < best_max) {
        best_max = max_rlf;
        best_color = std::move(color_rlf);
    }

    apply_kempe_chains(n, adj, best_max, best_color);
    return {best_max, best_color};
}

std::pair<size_t, std::vector<ssize_t>> simulated_annealing(size_t n, const std::vector<std::vector<size_t>> &adj) {
    auto [best_max, best_color] = dsatur(n, adj);
    auto [max_rlf, color_rlf] = rlf(n, adj);

    if (max_rlf < best_max) {
        best_max = max_rlf;
        best_color = std::move(color_rlf);
    }

    apply_kempe_chains(n, adj, best_max, best_color);

    size_t current_k = best_max + 1;
    std::vector<ssize_t> current_coloring = best_color;

    while (current_k > 1) {
        size_t target_k = current_k - 1;
        std::vector<ssize_t> working_coloring = current_coloring;

        if (apply_simulated_annealing(n, adj, target_k, working_coloring)) {
            current_k = target_k;
            current_coloring = working_coloring;
        } else break;
    }

    ssize_t final_max = 0;
    for (size_t i = 0; i < n; ++i) {
        if (current_coloring[i] > final_max) final_max = current_coloring[i];
    }

    return {(size_t)final_max, current_coloring};
}

int main(int argc, char* argv[]) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [n, m, adj] = read_data(filename);
    auto [max, coloring] = simulated_annealing(n, adj);
    
    std::cout << max + 1 << '\n';
    for (const auto &color : coloring)
        std::cout << color << '\n';
}