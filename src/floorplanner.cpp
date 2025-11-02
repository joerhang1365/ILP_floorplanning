#include "floorplanner.h"
#include <algorithm>
#include <cmath>
#include <limits>

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
    std::vector<Module *> clusterModules = c->getSubModules();

    // reset modules because weird shit happening
    for (auto m : clusterModules)
    {
        m->setRotate(false);
        m->setPosition(Point(0, 0));
    }

    int n = clusterModules.size();
    double M = std::max(targetWidth, targetHeight);

    // create variables for each module
    //

    // x left coordinate
    // y bottom coordinate
    // r rotation flag
    std::vector<std::string> x_vars(n), y_vars(n), r_vars(n);

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

    for (int i = 0; i < n; i++)
    {
        Module * mod_i = clusterModules[i];
        double w_i = mod_i->getRotatedWidth();
        double h_i = mod_i->getRotatedHeight();

        // inside outline constraints
        //

        // x_i >= 0 and y_i >= 0 are already handled by variable domain

        solver_.addConstraint("inside_outline_x" + std::to_string(i), {{x_vars[i], 1.0}, {r_vars[i], h_i - w_i}}, '<', targetWidth - w_i);
        solver_.addConstraint("inside_outline_y" + std::to_string(i), {{y_vars[i], 1.0}, {r_vars[i], w_i - h_i}, {"Y", -1.0}}, '<', -h_i);

        // non-overlapping constraints
        for (int j = i + 1; j < n; j++)
        {
            Module * mod_j = clusterModules[j];
            double w_j = mod_j->getRotatedWidth();
            double h_j = mod_j->getRotatedHeight();

            solver_.addConstraint("left_" + std::to_string(i) + "_" + std::to_string(j), 
                {{x_vars[i], 1.0}, {x_vars[j], -1.0}, {r_vars[i], h_i - w_i}, {p_vars[i][j], -M}, {q_vars[i][j], -M}}, '<', -w_i);

            solver_.addConstraint("below_" + std::to_string(i) + "_" + std::to_string(j), 
                {{y_vars[i], 1.0}, {y_vars[j], -1.0}, {r_vars[i], w_i - h_i}, {p_vars[i][j], -M}, {q_vars[i][j], M}}, '<', M - h_i);

            solver_.addConstraint("right_" + std::to_string(i) + "_" + std::to_string(j), 
                {{x_vars[i], 1.0}, {x_vars[j], -1.0}, {r_vars[j], -(h_j - w_j)}, {p_vars[i][j], -M}, {q_vars[i][j], M}}, '>', w_j - M);

            solver_.addConstraint("above_" + std::to_string(i) + "_" + std::to_string(j), 
                {{y_vars[i], 1.0}, {y_vars[j], -1.0}, {r_vars[j], -(w_j - h_j)}, {p_vars[i][j], -M}, {q_vars[i][j], -M}}, '>', h_j - 2 * M);
        }
    }

    // optional height constraint Y <= H
    //solver_.addConstraint("height_limit", {{"Y", 1.0}}, '<', targetHeight);
    
    // after all constraints and the objective are set solve the model
    // add a time limit if it takes too long
    double seconds = 600.0;
    solver_.setTimeLimit(seconds);
    solver_.optimize();

    const int status = solver_.getStatus();
    if (status == GRB_INFEASIBLE)
    {
        std::cout << "ILP unsat!" << std::endl;
        solver_.reset();
        return false;
    }

    if (status == GRB_TIME_LIMIT || status == GRB_INTERRUPTED) 
    {
        if (solver_.getSolutionCount() == 0) 
        {
            std::cout << "Time limit reached with NO feasible solution.\n";
            solver_.reset();
            return false;
        }
    }

    // extract solution from solver

    double Y = solver_.getVariableValue("Y");

    for (int i = 0; i < n; i++) 
    {
        double x = solver_.getVariableValue(x_vars[i]);
        double y = solver_.getVariableValue(y_vars[i]);
        double r = solver_.getVariableValue(r_vars[i]);

        // convert positions to integers because python drawer freaks out
        int x_int = static_cast<int>(std::round(x));
        int y_int = static_cast<int>(std::round(y));
        
        // change the modules position and rotation according to solution
        clusterModules[i]->setPosition(Point(x_int, y_int));

        // convert r to boolean
        bool b = r > 0.5 ? true : false;
        clusterModules[i]->setRotate(b);
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

    float finalHeight = solveCluster(clusters[0].get(), spec.targetWidth, spec.targetHeight);

    // you may try to uncomment the following 4 functions to verify if your cluster level 
    // rotate() works

    //clusters[0]->setPosition(Point(2025,1015));
    //clusters[0]->rotate();
    //clusters[0]->setPosition(Point(6.5,0.5));
    //clusters[0]->rotate();

    return finalHeight;
}

// current implementation is a simple greedy algorithm called shelf packing
// it works but can be better
// if I have time another solution is to group modules into clusters and solve each cluster with ILP
// then arrange the clusters using their width and heights using my shelf packing algorithm
float Floorplanner::category1Opt()
{
    const int clusterSize = 5;

    clusters.clear();

    // group modules into clusters
    for (size_t i = 0; i < modules.size(); i += clusterSize)
    {
        std::vector<Module *> moduleCluster;
        size_t end = std::min(i + clusterSize, modules.size());

        for (size_t j = i; j < end; j++)
        {
            moduleCluster.push_back(modules[j].get());
        }

        if (!moduleCluster.empty())
        {
            clusters.push_back(std::make_unique<Cluster>(moduleCluster));
        }
    }

    // solve each cluster
    for (auto &cluster : clusters)
    {
        // try to find good constraints for solver
        double totalArea = 0.0;
        double maxModWidth = 0.0;
        double maxModHeight = 0.0;

        for (auto &module : cluster->getSubModules())
        {
            double w = module->getRotatedWidth();
            double h = module->getRotatedHeight();
            totalArea += w * h;
            maxModWidth = std::max(maxModWidth, w);
            maxModHeight = std::max(maxModHeight, h);
        }
        
        // Try progressively larger bounds with better scaling
        for (int attempt = 0; attempt < 3; attempt++)
        {
            double width = std::min(static_cast<double>(spec.targetWidth), maxModWidth * (1.0 + attempt * 0.5));
            double height = std::min(static_cast<double>(spec.targetHeight), maxModHeight * (1.0 + attempt * 0.5));
            
            if (solveCluster(cluster.get(), width, height))
            {
                break;
            }
        }
    }

    // arrange clusters using shelf packing

    std::vector<Cluster *> sortedClusters;

    for (auto &cluster : clusters)
    {
        sortedClusters.push_back(cluster.get());
    }

    for (auto &cluster : sortedClusters)
    {
        if (cluster->getRotatedHeight() < cluster->getRotatedWidth())
        {
            //cluster->rotate();
        }
    }

    // sort tallest height
    std::sort(sortedClusters.begin(), sortedClusters.end(), [](Cluster * a, Cluster * b) {
        return a->getRotatedHeight() > b->getRotatedHeight();
    });

    double currentX = 0.0;
    double currentY = 0.0;
    double shelfHeight = 0.0;

    for (auto &cluster : sortedClusters)
    {
        double width = cluster->getRotatedWidth();
        double height = cluster->getRotatedHeight();

        // check if fits on current shelf
        if (currentX + width <= spec.targetWidth)
        {
            cluster->setPosition(Point(currentX, currentY));
            currentX += width;
            shelfHeight = std::max(shelfHeight, height);
        }
        // else make new shelf
        else
        {
            currentX = 0.0;
            currentY += shelfHeight;
            shelfHeight = height;
            cluster->setPosition(Point(currentX, currentY));
            currentX += width;
        }
    }

    return currentY + shelfHeight;
/*
    std::vector<Module *> sortedModules;

    for (auto &module : modules)
    {
        sortedModules.push_back(module.get());
    }

    // rotate so height >= width
    for (auto &module : sortedModules)
    {
        if (module->getRotatedHeight() < module->getRotatedWidth())
        {
            module->setRotate(true);
        }
    }

    // sort tallest height
    std::sort(sortedModules.begin(), sortedModules.end(), [](Module * a, Module * b) {
        return a->getRotatedHeight() > b->getRotatedHeight();
    });
    
    double currentX = 0.0;
    double currentY = 0.0;
    double shelfHeight = 0.0;

    for (auto &module : sortedModules)
    {
        double width = module->getRotatedWidth();
        double height = module->getRotatedHeight();

        // check if fits on current shelf
        if (currentX + width <= spec.targetWidth)
        {
            module->setPosition(Point(currentX, currentY));
            currentX += width;
            shelfHeight = std::max(shelfHeight, height);
        }
        // else make new shelf
        else
        {
            currentX = 0.0;
            currentY += shelfHeight;
            shelfHeight = height;
            module->setPosition(Point(currentX, currentY));
            currentX += width;
        }
    }

    return currentY + shelfHeight;*/
    //clusters[0]->setPosition(Point(100, 100));
    //clusters[2]->setPosition(Point(200, 200));
    return 0.0;
}