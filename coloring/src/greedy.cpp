#include <vector>

#include "CLI/CLI.hpp"

CLI::App app{"compiler"};

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

int main(int argc, char* argv[]) {
    std::string filename;
    app.add_option("source", filename)->required()->check(CLI::ExistingFile);

    CLI11_PARSE(app, argc, argv);

    auto [n, m, adj] = read_data(filename);
    auto [max, coloring] = coloring_greedy(n, m, adj);
    
    std::cout << max << '\n';
    for (const auto &color : coloring)
        std::cout << color << '\n';
}