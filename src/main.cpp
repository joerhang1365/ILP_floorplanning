#include "gurobi_c++.h"
#include "floorplanner.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv) 
{
    if (argc != 4) 
    {
        std::cerr << "Usage: " << argv[0] << " <inputFile> <specFile> <outputFile>" << std::endl;
        return 1;
    }

    Floorplanner fp_;
    fp_.initialize(argv[1]);
    fp_.setSpec(Spec(argv[2]));
    fp_.solve();
    fp_.validityCheck();    // you can comment out this function
    fp_.writeOutput(argv[3]);

    return 0;
}
