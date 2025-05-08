#include "conv2list.h"

using namespace std;

// XmlToTspConverter
XmlToTspConverter::XmlToTspConverter(const string& xml_file, const string& tsp_file, int num_vertices)
    : xml_filename(xml_file), tsp_filename(tsp_file), N(num_vertices) {}

void XmlToTspConverter::convert() {
    pugi::xml_document doc;
    if (!doc.load_file(xml_filename.c_str())) {
        cerr << "UNABLE to load xml file" << endl;
        exit(1);
    }

    pugi::xml_node graph = doc.child("travellingSalesmanProblemInstance").child("graph");

    ofstream out(tsp_filename);
    if (!out.is_open()) {
        cerr << "UNABLE to create tsp file" << endl;
        exit(1);
    }

    out << "NAME : ConvertedFromXML\n";
    out << "TYPE : TSP\n";
    out << "DIMENSION : " << N << "\n";
    out << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
    out << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
    out << "EDGE_WEIGHT_SECTION\n";

    for (pugi::xml_node vertex : graph.children("vertex")) {
        for (pugi::xml_node edge : vertex.children("edge")) {
            double cost = stod(edge.attribute("cost").value());
            out << cost << " ";
        }
        out << "\n";
    }

    out << "EOF\n";
    out.close();
}

// TspParser
TspParser::TspParser(const string& filename, int num_vertices)
    : tsp_filename(filename), N(num_vertices), dist(num_vertices, vector<double>(num_vertices, MAX)) {}

void TspParser::parse() {
    ifstream in(tsp_filename);
    if (!in.is_open()) {
        cerr << "UNABLE to load tsp file" << endl;
        exit(1);
    }

    string line;
    while (getline(in, line)) {
        if (line == "EDGE_WEIGHT_SECTION")
            break;
    }

    for (int i = 0; i < N; ++i)
        for (int j = 0; j < N; ++j)
            in >> dist[i][j];

    in.close();
}

const vector<vector<double>>& TspParser::get_matrix() const {
    return dist;
}

void TspParser::print_matrix(int num) const {
    if (num > N) {
        cout << "OUT OF RANGE! MAXIMUM length is :" << N << endl;
        return;
    }
    for (int i = 0; i < num; ++i) {
        cout << "Vertex " << i << " → ";
        for (double d : dist[i])
            cout << (d == MAX ? "INF" : to_string(d)) << " ";
        cout << endl;
    }
}

// TspCoordParser
TspCoordParser::TspCoordParser(const string& filename, int num_vertices)
    : tsp_filename(filename), N(num_vertices),
      coordinates(num_vertices),
      dist(num_vertices, vector<double>(num_vertices, MAX)) {}

void TspCoordParser::parse() {
    ifstream file(tsp_filename);
    if (!file.is_open()) {
        cerr << "FAIL to open tsp file: " << tsp_filename << endl;
        exit(1);
    }

    string line;
    while (getline(file, line)) {
        if (line.find("NODE_COORD_SECTION") != string::npos)
            break;
    }

    int count = 0;
    while (getline(file, line)) {
        if (line.find("EOF") != string::npos || count >= N) break;

        istringstream iss(line);
        int index;
        double x, y;
        if (!(iss >> index >> x >> y)) {
            cerr << "PARSING FAILURE: " << line << endl;
            continue;
        }

        coordinates[count] = {x, y};
        ++count;
    }

    file.close();
}

void TspCoordParser::compute_distance_matrix() {
    for (int i = 0; i < N; ++i) {
        for (int j = 0; j < N; ++j) {
            if (i == j) {
                dist[i][j] = MAX;
            } else {
                double dx = coordinates[i].first - coordinates[j].first;
                double dy = coordinates[i].second - coordinates[j].second;
                dist[i][j] = sqrt(dx * dx + dy * dy);
            }
        }
    }
}

const vector<pair<double, double>>& TspCoordParser::get_coords() const {
    return coordinates;
}

const vector<vector<double>>& TspCoordParser::get_matrix() const {
    return dist;
}

void TspCoordParser::print_matrix(int num) const {
    if (num > N) {
        cout << "OUT OF RANGE! MAXIMUM length is :" << N << endl;
        return;
    }
    for (int i = 0; i < num; ++i) {
        cout << "Vertex " << i << " → ";
        for (double d : dist[i])
            cout << (d == MAX ? "INF" : to_string(d)) << " ";
        cout << endl;
    }
}
