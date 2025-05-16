#ifndef TSPARSER_H
#define TSPARSER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <algorithm>

class TSPParser {
private:
    std::string filepath;
    int dimension;
    std::string edge_weight_type;
    std::map<int, std::pair<double, double>> nodes;

    std::string trim(const std::string& str) {
        size_t first = str.find_first_not_of(" \t\r\n");
        if (first == std::string::npos) return "";
        size_t last = str.find_last_not_of(" \t\r\n");
        return str.substr(first, (last - first + 1));
    }

public:
    TSPParser(const std::string& file) : filepath(file), dimension(0) {}

    void parse() {
        std::ifstream infile(filepath);
        if (!infile.is_open()) {
            std::cerr << "❌ Cannot open file: " << filepath << std::endl;
            return;
        }

        std::string line;
        bool reading_nodes = false;
        while (std::getline(infile, line)) {
            line = trim(line);
            if (line.empty()) continue;

            if (line.find("NODE_COORD_SECTION") != std::string::npos) {
                reading_nodes = true;
                std::cout << "🔵 Start reading nodes..." << std::endl;
                continue;
            }
            if (line.find("EOF") != std::string::npos) {
                std::cout << "🔴 End of file reached." << std::endl;
                break;
            }

            if (!reading_nodes) {
                if (line.find("DIMENSION") != std::string::npos) {
                    dimension = std::stoi(line.substr(line.find(":") + 1));
                } else if (line.find("EDGE_WEIGHT_TYPE") != std::string::npos) {
                    edge_weight_type = trim(line.substr(line.find(":") + 1));
                }
            } else {
                std::istringstream iss(line);
                int node_id;
                double x, y;
                if (iss >> node_id >> x >> y) {
                    nodes[node_id] = std::make_pair(x, y);
                    // 디버깅
                    if (node_id <= 3) {
                        std::cout << "Node parsed: " << node_id << " (" << x << ", " << y << ")\n";
                    }
                }
            }
        }

        infile.close();
    }

    void printInfo() const {
        std::cout << "📄 TSP Instance Info\n";
        std::cout << "Dimension: " << dimension << "\n";
        std::cout << "Edge Weight Type: " << edge_weight_type << "\n";
        std::cout << "Total nodes loaded: " << nodes.size() << "\n";
    }

    const std::map<int, std::pair<double, double>>& getNodes() const {
        return nodes;
    }
};

#endif