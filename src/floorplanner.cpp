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

// Implement the ILP model to minimize height here
// Use the solver_ object to add variables, constraints, and set the objective
// After setting up the model, call solver_.optimize() to solve it
// Update the positions and rotations of modules based on the solution

bool Floorplanner::solveCluster(Cluster * c, float targetWidth, float targetHeight) 
{
    std::cout << "Solving cluster with targetWidth: " << targetWidth << ", targetHeight: " << targetHeight << std::endl;
    std::vector<Module *> clusterModules = c->getSubModules();
    int n = clusterModules.size();
    double M = std::max(targetWidth, targetHeight); // Big-M constant

    // create variables for each module
    //

    std::vector<std::string> x_vars(n); // left of module
    std::vector<std::string> y_vars(n); // bottom of module
    std::vector<std::string> r_vars(n); // rotation flag

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
    //

    std::vector<std::vector<std::string>> p_vars(n, std::vector<std::string>(n)); // non-overlapping flag x
    std::vector<std::vector<std::string>> q_vars(n, std::vector<std::string>(n)); // non-overlapping flag y

    for (int i = 0; i < n; i++)
    {
        for (int j = i + 1; j < n; j++)
        {
            p_vars[i][j] = "p_" + std::to_string(i) + "_" + std::to_string(j);
            q_vars[i][j] = "q_" + std::to_string(i) + "_" + std::to_string(j);

            solver_.addVariable(p_vars[i][j], 0.0, 1.0, GRB_BINARY);
            solver_.addVariable(q_vars[i][j], 0.0, 1.0, GRB_BINARY);
        }
    }

    // create variable for overall height
    solver_.addVariable("Y", 0.0, targetHeight, GRB_CONTINUOUS);
    
    // set objective to minimize height
    solver_.setObjective({{"Y", 1.0}}, 'M');

    // add constraints for each module
    //

    // optional height constraint Y <= H
    //solver_.addConstraint("height_limit", {{"Y", 1.0}}, '<', targetHeight);

    // check if module is not a cluster
    if (!isCluster(clusterModules[0])) 
    {
        std::cout << "clusterModules[0] is not in a cluster" << std::endl;
    }

    for (int i = 0; i < n; i++)
    {
        Module * mod_i = clusterModules[i];
        double w_i = mod_i->getOrgWidth();
        double h_i = mod_i->getOrgHeight();

        // inside outline constraints
        //

        // x_i >= 0 and y_i >= 0 are already handled by variable domain

        // x_i + w_i' <= W
        // x_i + r_i * (h_i - w_i) <= targetWidth - w_i
        solver_.addConstraint("in_x" + std::to_string(i), {{x_vars[i], 1.0}, {r_vars[i], h_i - w_i}}, '<', targetWidth - w_i);

        // y_i + h_i' <= Y
        // y_i + r_i * (w_i - h_i) * r_i - Y <= -h_i
        solver_.addConstraint("in_y" + std::to_string(i), {{y_vars[i], 1.0}, {r_vars[i], w_i - h_i}, {"Y", -1.0}}, '<', -h_i);

        // non-overlapping constraints
        for (int j = i + 1; j < n; j++)
        {
            Module * mod_j = clusterModules[j];
            double w_j = mod_j->getOrgWidth();
            double h_j = mod_j->getOrgHeight();

            // x_i + w_i' <= x_j + M * (1 - p_ij)
            // x_i - x_j + r_i * (h_i - w_i) - M * (p_ij) - M * (q_ij) <= -w_i
            solver_.addConstraint("nonoverlap1_" + std::to_string(i) + "_" + std::to_string(j), 
                {{x_vars[i], 1.0}, {x_vars[j], -1.0}, {r_vars[i], h_i - w_i}, {p_vars[i][j], -M}, {q_vars[i][j], -M}}, '<', -w_i);

            // y_i + h_i' <= y_j + M * (1 + p_ij - q_ij)
            // y_i - y_j + r_i * (w_i - h_i) - M * (p_ij) + M * (q_ij) <= M - h_i
            solver_.addConstraint("nonoverlap2_" + std::to_string(i) + "_" + std::to_string(j), 
                {{y_vars[i], 1.0}, {y_vars[j], -1.0}, {r_vars[i], w_i - h_i}, {p_vars[i][j], -M}, {q_vars[i][j], M}}, '<', M - h_i);

            // x_i >= x_j + w_j' - M * (1 - p_ij + q_ij)
            // x_i - x_j - r_j * (h_j - w_j) - M * (p_ij) + M * (q_ij) >= w_j - M
            solver_.addConstraint("nonoverlap3_" + std::to_string(i) + "_" + std::to_string(j), 
                {{x_vars[i], 1.0}, {x_vars[j], -1.0}, {r_vars[j], -(h_j - w_j)}, {p_vars[i][j], -M}, {q_vars[i][j], M}}, '>', w_j - M);

            // y_i >= y_j + h_j' - M * (2 - p_ij - q_ij)
            // y_i - y_j - r_j * (w_j - h_j) - M * (p_ij) - M * (q_ij) >= h_j - 2 * M
            solver_.addConstraint("nonoverlap4_" + std::to_string(i) + "_" + std::to_string(j), 
                {{y_vars[i], 1.0}, {y_vars[j], -1.0}, {r_vars[j], -(w_j - h_j)}, {p_vars[i][j], -M}, {q_vars[i][j], -M}}, '>', h_j - 2 * M);
        }
    }
    
    // after all constraints and the objective are set solve the model
    double seconds = 600.0;
    solver_.setTimeLimit(seconds);
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

    // extract solution from solver
    for (int i = 0; i < n; i++) 
    {
        double x = solver_.getVariableValue(x_vars[i]);
        double y = solver_.getVariableValue(y_vars[i]);
        double r = solver_.getVariableValue(r_vars[i]);

        std::cout << "Module " << clusterModules[i]->getId() << ": x=" << x << ", y=" << y << ", r=" << r << std::endl;
    
        // change the modules position and rotation according to solution
        clusterModules[i]->setPosition(Point(x, y));
        clusterModules[i]->setRotate(r > 0.5);
    }

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
    std::cout << "work fucking" << std::endl;
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