#include "conv2list.h"
#include "Held_Karp.h"
//#include "christofides.h"
#include <iostream>
#include <vector>

using namespace std;


int main() {

    // XmlToTspConverter a280T("a280.xml", "a280.tsp", 280);
    // a280T.convert();
    // TspParser a280TSP("a280.tsp", 280);
    // a280TSP.parse();

    // vector<vector<double>> weight = a280TSP.get_matrix();

    int num = 23;
    TspCoordParser xql662("xql662.tsp", num);
    xql662.parse();
    xql662.compute_distance_matrix();
    //xql662.print_matrix(5);
    vector<vector<double>> xql_matrix = xql662.get_matrix();

    HK_TSP xql(xql_matrix);
    //printf("the size of matrix: %d\n", xql.get_n());
    //printf("dp talbe size: %d\n", xql.get_dp_size());
    xql.solve();
    cout << "Min cost of xql_" << num << ": " << xql.min_cost() << endl;

    return 0;
}