#ifndef _FLOORPLANNER_H_
#define _FLOORPLANNER_H_

#include "solver.h"
#include "util.h"
#include "module.h"
#include "spec.h"
#include "cluster.h"

class Floorplanner 
{
public:
    Floorplanner() {}
    ~Floorplanner() {}
    void solve();
    void initialize(std::string inputFile);
    void setSpec(Spec s) { spec = s; }
    void setSpec(std::string specFile) { spec = Spec(specFile); }
    void writeOutput(std::string outputFile);
    bool validityCheck();
    float category0Opt();
    float category1Opt();      
    
private:
    bool solveCluster(Cluster * c, float targetWidth, float targetHeight);
    
    std::vector<std::unique_ptr<Module>> modules;
    std::vector<std::unique_ptr<Cluster>> clusters;
    Spec spec;
    Solver solver_;
};

#endif