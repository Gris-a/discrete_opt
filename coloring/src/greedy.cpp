#include <vector>
#include <queue>
#include <fstream>
#include <sstream>
#include <iostream>

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

std::pair<size_t, std::vector<ssize_t>> coloring_greedy(size_t n, size_t m, const std::vector<std::vector<size_t>> &adj) {
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

std::pair<size_t, std::vector<ssize_t>> run_dsatur(size_t n, const std::vector<std::vector<size_t>> &adj) {
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

std::pair<size_t, std::vector<ssize_t>> run_rlf(size_t n, const std::vector<std::vector<size_t>> &adj) {
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
        for(size_t i=0; i<n; ++i) {
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

// ==========================================
// Алгоритм 2: Перестановки Кемпе (Локальный поиск)
// ==========================================
void apply_kempe_chains(size_t n, const std::vector<std::vector<size_t>> &adj, size_t &max_color, std::vector<ssize_t> &color) {
    bool improved = true;
    while(improved) {
        improved = false;
        std::vector<size_t> target_vertices;
        
        // Ищем все вершины с максимальным цветом
        for(size_t i = 0; i < n; ++i) {
            if(color[i] == max_color) target_vertices.push_back(i);
        }

        bool all_eliminated = true;
        for(size_t v : target_vertices) {
            bool v_resolved = false;
            for(ssize_t c = 0; c < max_color; ++c) {
                std::vector<size_t> neighbors_c;
                for(size_t u : adj[v]) {
                    if(color[u] == c) neighbors_c.push_back(u);
                }

                if(neighbors_c.empty()) {
                    color[v] = c;
                    v_resolved = true;
                    break;
                }

                // Строим цепь Кемпе если ровно 1 сосед имеет цвет 'c'
                if(neighbors_c.size() == 1) {
                    size_t start_node = neighbors_c[0];
                    std::vector<size_t> chain;
                    std::vector<bool> visited(n, false);
                    std::queue<size_t> q;
                    
                    q.push(start_node);
                    visited[start_node] = true;
                    
                    while(!q.empty()) {
                        size_t curr = q.front();
                        q.pop();
                        chain.push_back(curr);

                        for(size_t nxt : adj[curr]) {
                            if(nxt == v) continue;
                            if((color[nxt] == c || color[nxt] == max_color) && !visited[nxt]) {
                                visited[nxt] = true;
                                q.push(nxt);
                            }
                        }
                    }

                    // Проверяем, не заденет ли цепь других соседей 'v'
                    bool conflict = false;
                    for(size_t node : chain) {
                        if(node != start_node) {
                            for(size_t adj_v : adj[v]) {
                                if(adj_v == node) { conflict = true; break; }
                            }
                        }
                        if(conflict) break;
                    }

                    if(!conflict) {
                        for(size_t node : chain) {
                            if(color[node] == c) color[node] = max_color;
                            else color[node] = c;
                        }
                        color[v] = c;
                        v_resolved = true;
                        break;
                    }
                }
            }
            if(!v_resolved) all_eliminated = false;
        }

        if(all_eliminated && !target_vertices.empty()) {
            max_color--;
            improved = true; // Продолжаем, если смогли уменьшить максимальный цвет
        }
    }
}

std::pair<size_t, std::vector<ssize_t>> coloring_final(size_t n, size_t m, const std::vector<std::vector<size_t>> &adj) {
    auto [best_max, best_color] = run_dsatur(n, adj);
    auto [max_rlf, color_rlf] = run_rlf(n, adj);

    if (max_rlf < best_max) {
        best_max = max_rlf;
        best_color = std::move(color_rlf);
    }

    apply_kempe_chains(n, adj, best_max, best_color);

    return {best_max, best_color};
}

int main(int argc, char* argv[]) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [n, m, adj] = read_data(filename);
    auto [max, coloring] = run_dsatur(n, adj);
    
    std::cout << max << '\n';
    for (const auto &color : coloring)
        std::cout << color << '\n';
}