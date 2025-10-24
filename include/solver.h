#ifndef _SOLVER_H_
#define _SOLVER_H_

#include "gurobi_c++.h"
#include "util.h"

class Solver 
{
public:
    Solver();
    void addVariable(const std::string &name, double lowerbound, double upperbound, char type);
    void addConstraint(const std::string &name, const std::vector<std::pair<std::string, double>> &vars, char sense, double rhs);
    void setObjective(const std::vector<std::pair<std::string, double>> &vars, char sense);
    void optimize();
    void reset();
    double getObjectiveValue() const;
    double getVariableValue(const std::string &name) const;
    void setTimeLimit(double seconds);
    int getSolutionCount();
    int getStatus() { return model_->get(GRB_IntAttr_Status); } 

private:
    static int objSense(std::string s);  // 'MIN' for minimization, 'MAX' for maximization

    std::unique_ptr<GRBEnv> env_;
    std::unique_ptr<GRBModel> model_;
    std::unordered_map<std::string, GRBVar> varmap_;
    int status_; 
};

#endif