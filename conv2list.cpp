#include "conv2list.h"

using namespace std;

#define MAX 1e12

class XmlToTspConverter {
private:
    string xml_filename;
    string tsp_filename;
    int N;

public:
    XmlToTspConverter(const string& xml_file, const string& tsp_file, int num_vertices)
        : xml_filename(xml_file), tsp_filename(tsp_file), N(num_vertices) {}

    void convert() {
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
};

class TspParser {
private:
    string tsp_filename;
    int N;
    vector<vector<double>> dist;

public:
    TspParser(const string& filename, int num_vertices)
        : tsp_filename(filename), N(num_vertices), dist(num_vertices, vector<double>(num_vertices, MAX)) {}

    void parse() {
        ifstream in(tsp_filename);
        if (!in.is_open()) {
            cerr << "UNABLE to load tsp file" << endl;
            exit(1);
        }

        string line;
        // 헤더 무시
        while (getline(in, line)) {
            if (line == "EDGE_WEIGHT_SECTION")
                break;
        }

        // 거리 행렬 읽기
        for (int i = 0; i < N; ++i) {
            for (int j = 0; j < N; ++j) {
                in >> dist[i][j];
            }
        }

        in.close();
    }

    const vector<vector<double>>& get_matrix() const { return dist; }

    void print_matrix(int num) const {
        if (num > N) {
            cout << "OUT OF RANGE! MAXIMUM length is :" << N << endl;
            return;
        }
        for (int i = 0; i < num; ++i) {
            cout << "Vertex " << i << " → ";
            for (double d : dist[i]){
                if (d == MAX) cout << "INF" << " ";
                else          cout << d << " ";
            }
            cout << endl;
        }
    }
};

class TspCoordParser {
private:
    string tsp_filename;
    int N;
    vector<pair<double, double>> coordinates;
    vector<vector<double>> dist;

public:
    TspCoordParser(const string& filename, int num_vertices)
        : tsp_filename(filename), N(num_vertices),
            coordinates(num_vertices),
            dist(num_vertices, vector<double>(num_vertices, MAX)) {}

    void parse() {
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
            
    void compute_distance_matrix() {
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

    const vector<pair<double, double>>& get_coords() const { return coordinates; }
    const vector<vector<double>>& get_matrix() const { return dist; }

    void print_matrix(int num) const {
        if (num > N) {
            cout << "OUT OF RANGE! MAXIMUM length is :" << N << endl;
            return;
        }
        for (int i = 0; i < num; ++i) {
            cout << "Vertex " << i << " → ";
            for (double d : dist[i]){
                if (d == MAX) cout << "INF" << " ";
                else          cout << d << " ";
            }
            cout << endl;
        }
    }
};


// int main() {
//     // XmlToTspConverter a280T("a280.xml", "a280.tsp", 280);
//     // a280T.convert();
//     // TspParser a280("a280.tsp", 280);
//     // a280.parse();
//     // // parser.print_matrix();

//     // TspCoordParser kz9976("kz9976.tsp", 9976);
//     // kz9976.parse();

//     // TspCoordParser ml100k("mona-lisa100K.tsp", 100000);
//     // ml100k.parse();

//     // TspCoordParser xql662("xql662.tsp", 662);
//     // xql662.parse();
//     // vector<pair<double, double>> co = xql662.get_coords();
//     // for(pair<double, double> p: co) cout << "x: " << p.first << ", y: " << p.second << "\n";
//     // xql662.compute_distance_matrix();
//     // vector<vector<double>> list = xql662.get_matrix();
//     // for(double num: list[1]) cout << num << " ";
//     // xql662.print_matrix(5);

//     // TspCoordParser kz9976("kz9976.tsp", 9976);
//     //kz9976.parse();
//     //vector<pair<double, double>> co = kz9976.get_coords();
//     //for(pair<double, double> p: co) cout << "x: " << p.first << ", y: " << p.second << "\n";
//     // kz9976.compute_distance_matrix();
//     // vector<vector<double>> list = xql662.get_matrix();
//     // for(double num: list[1]) cout << num << " ";
//     // kz9976.print_matrix(5);

//     // TspCoordParser ml100k("mona-lisa100K.tsp", 100000);
//     // ml100k.parse();
//     // // vector<pair<double, double>> co = xql662.get_coords();
//     // // for(pair<double, double> p: co) cout << "x: " << p.first << ", y: " << p.second << "\n";
//     // ml100k.compute_distance_matrix();
//     // // vector<vector<double>> list = xql662.get_matrix();
//     // // for(double num: list[1]) cout << num << " ";
//     // ml100k.print_matrix(5);

//     return 0;
// }
