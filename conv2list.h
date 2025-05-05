#ifndef CONV2LIST_H
#define CONV2LIST_H

#include <string>
#include <vector>
#include <utility>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cmath>
#include <sstream>
#include "pugixml.hpp"

#define MAX 1e12

class XmlToTspConverter {
private:
    std::string xml_filename;
    std::string tsp_filename;
    int N;

public:
    XmlToTspConverter(const std::string& xml_file, const std::string& tsp_file, int num_vertices);
    void convert();
};

class TspParser {
private:
    std::string tsp_filename;
    int N;
    std::vector<std::vector<double>> dist;

public:
    TspParser(const std::string& filename, int num_vertices);
    void parse();
    const std::vector<std::vector<double>>& get_matrix() const;
    void print_matrix(int num) const;
};

class TspCoordParser {
private:
    std::string tsp_filename;
    int N;
    std::vector<std::pair<double, double>> coordinates;
    std::vector<std::vector<double>> dist;

public:
    TspCoordParser(const std::string& filename, int num_vertices);
    void parse();
    void compute_distance_matrix();
    const std::vector<std::pair<double, double>>& get_coords() const;
    const std::vector<std::vector<double>>& get_matrix() const;
    void print_matrix(int num) const;
};

#endif // CONV2LIST_H
