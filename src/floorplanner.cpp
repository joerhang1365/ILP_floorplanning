#include "floorplanner.h"

void Floorplanner::solve() 
{
    if (spec.problemType == 0) 
    {
        category0Opt();
    } 
    else 
    {
        category1Opt();
    }
}

bool Floorplanner::solveCluster(Cluster * c, float targetWidth, float targetHeight) 
{
    // Implement the ILP model to minimize height here
    // Use the solver_ object to add variables, constraints, and set the objective
    // After setting up the model, call solver_.optimize() to solve it
    // Update the positions and rotations of modules based on the solution

    std::vector<Module *> clusterModules = c->getSubModules();

    /*TODO: add ILP formulation to solve this module set*/
    /*
    ILP Usage Example (Gurobi)

    1) Add Variables
    solver_.addVariable("REAL", 0.0, 100.0, GRB_CONTINUOUS);  // REAL belongs to (0, 100)
    solver_.addVariable("REAL", 0.0, 2.0, GRB_INTEGER);  // REAL belongs to [0, 2] ( {0,1,2} ) 
    solver_.addVariable("BOOL", 0.0, 1.0, GRB_BINARY);        // BOOL belongs to {0, 1}

    2) Set Objective
    // Minimize: X + Y
    solver_.setObjective({{"X", 1.0}, {"Y", 1.0}}, 'M');      // 'M' = minimize (implementation dependent)

    3) Add Constraints
    // Y - X ≤ 100
    solver_.addConstraint("c1", {{"Y", 1.0}, {"X", -1.0}}, '<', 100.0);
    // Supported relations: '<' (≤), '>' (≥), '=' (==)

    4) Check if a Module is a Cluster
    // To determine whether a Module* is actually a Cluster*, use isCluster(...)
    if (isCluster(clusterModules[0])) {
        std::cout << "clusterModules[0] is a cluster" << std::endl;
    }

    5) Set Time Limit
    // If you are concerned that the solver might spend too much time on a single sub-problem,
    // set a time limit (in seconds) before calling optimize().
    double seconds = 600.0;                // e.g., 10 minutes
    solver_.setTimeLimit(seconds);
    solver_.optimize();

    Notes:
    - When the time limit is reached, Gurobi may still return a feasible solution (incumbent).
      Before accessing solution values, check if the solution count (SolCount) > 0.
    */

    int n = clusterModules.size();
    double M = std::max(targetWidth, targetHeight); // Big-M

    std::vector<std::string> x_vars(n); // left of module
    std::vector<std::string> y_vars(n); // bottom of module
    std::vector<std::string> r_vars(n); // rotation flag

    // create variables for each module

    for (int i = 0; i < n; i++)
    {
        x_vars[i] = "x_" + std::to_string(i);
        y_vars[i] = "y_" + std::to_string(i);
        r_vars[i] = "r_" + std::to_string(i);

        solver_.addVariable(x_vars[i], 0.0, targetWidth, GRB_CONTINUOUS);
        solver_.addVariable(y_vars[i], 0.0, targetHeight, GRB_CONTINUOUS);
        solver_.addVariable(r_vars[i], 0.0, 1.0, GRB_BINARY);
    }

    // create non-overlapping variables between modules

    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            std::string p_name = "p_" + std::to_string(i) + "_" + std::to_string(j);
            std::string q_name = "q_" + std::to_string(i) + "_" + std::to_string(j);
            solver_.addVariable(p_name, 0.0, 1.0, GRB_BINARY);
            solver_.addVariable(q_name, 0.0, 1.0, GRB_BINARY);
        }
    }
    
    // set objective to minimize height
    solver_.setObjective({{"Y", 1.0}}, 'M'); // 'M" for minimize

    // add constraints for each module

    for (int i = 0; i < n; i++)
    {
        Module * mod = clusterModules[i];

        // TODO: check if this is correct
        double w = mod->getOrgWidth();
        double h = mod->getOrgHeight();

        // inside outline constraints
    }
    
    // After all constraints and the objective are set, solve the model.
    // Do NOT remove the following lines.

    solver_.optimize();
    if (solver_.getStatus() == GRB_INFEASIBLE)
    {
        std::cout << "ILP unsat!" << std::endl;
        solver_.reset();
        return false;
    }

    const int st = solver_.getStatus();
    if (st == GRB_TIME_LIMIT || st == GRB_INTERRUPTED) 
    {
        if (solver_.getSolutionCount() == 0) 
        {
            std::cout << "Time limit reached with NO feasible solution.\n";
            solver_.reset();
            return false;
        }
    }

    /* TODO: Use solver_.getVariableValue(<name>) to fetch solution values
       and write them back to the corresponding modules/clusters.
       - Example: double x = solver_.getVariableValue("X");
    */

    solver_.reset();    // DO NOT delete or comment out this line
    return true;
}

float Floorplanner::category0Opt() 
{
    // You don't need to modify this function 
    // Do NOT remove or reorder the following three lines unless you understand the workflow.
    // It shows how to wrap all modules into a top-level Cluster, and solve it.

    clusters.clear();
    clusters.push_back(std::make_unique<Cluster>(modules));
    float finalHeight = solveCluster(clusters[0].get(), spec.targetWidth, spec.targetHeight);

    // you may try to uncomment the following 4 functions to verify if your cluster level 
    // rotate() works

    //clusters[0]->setPosition(Point(2025,1015));
    //clusters[0]->rotate();
    //clusters[0]->setPosition(Point(10,2));
    //clusters[0]->rotate();

    return finalHeight;
}

float Floorplanner::category1Opt()
{
    // TODO: Implement your own heuristic to solve this problem.

    return 0.0;
}