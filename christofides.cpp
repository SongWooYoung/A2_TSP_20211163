#include <iostream>
#include "conv2list.h"

using namespace std;


int main() {

    XmlToTspConverter a280T("a280.xml", "a280.tsp", 280);
    a280T.convert();
    TspParser a280TSP("a280.tsp", 280);
    a280TSP.parse();
    //a280TSP.print_matrix(5);

    return 0;
}