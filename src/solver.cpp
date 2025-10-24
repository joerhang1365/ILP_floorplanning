#include "solver.h"

Solver::Solver() : env_(std::make_unique<GRBEnv>(true)), model_(nullptr), status_(GRB_LOADED) 
{
    try 
    {
        std::cout << "Initializing Gurobi model...\n";
    
        if (!std::getenv("GRB_LICENSE_FILE") && std::filesystem::exists("./gurobi.lic")) 
        {
            setenv("GRB_LICENSE_FILE", "./gurobi.lic", 1);
        }

        env_->set("LogToConsole", "1");
        env_->set("LogFile", "gurobi.log"); 
        env_->start();                      

        model_ = std::make_unique<GRBModel>(*env_);
        std::cout << "Gurobi model initialized.\n";
    }
    catch (const GRBException& e) 
    {
        std::cerr << "[Gurobi Init Failed] code=" << e.getErrorCode() << " msg=" << e.getMessage() << "\n";
        throw; 
    }
}

int Solver::objSense(std::string s)
{
    if (s == "MIN") 
    {
        return GRB_MINIMIZE;
    }
    else if (s == "MAX") 
    {
        return GRB_MAXIMIZE;
    }
    else 
    {
        std::cerr << "Error: unknown objective sense, set to maximize " << s << std::endl;
        return GRB_MAXIMIZE;
    }
}

void Solver::addVariable(const std::string &name, double lb, double ub, char type) 
{
    if (varmap_.count(name))
    {
        throw std::runtime_error("Variable already exists: " + name);
    }

    GRBVar v = model_->addVar(lb, ub, 0.0, type, name);
    varmap_.emplace(name, v);
}

void Solver::addConstraint(const std::string &name, const std::vector<std::pair<std::string, double>> &vars, char sense, double rhs) 
{
    GRBLinExpr expr = 0.0;
    for (const auto& [vname, coef] : vars) 
    {
        auto it = varmap_.find(vname);
        if (it == varmap_.end())
        {
            throw std::runtime_error("Unknown variable in constraint '" + name + "': " + vname);
        }
        expr += coef * it->second;
    }
    model_->addConstr(expr, sense, rhs, name);
}

void Solver::setObjective(const std::vector<std::pair<std::string, double>> &vars, char sense) 
{
    GRBLinExpr expr = 0.0;
    for (const auto& [vname, coef] : vars) 
    {
        auto it = varmap_.find(vname);
        if (it == varmap_.end()) 
        {
            throw std::runtime_error("Unknown variable in objective: " + vname);
        }
        expr += coef * it->second;
    }
    model_->setObjective(expr, sense);
}

void Solver::optimize() 
{
    model_->optimize();
    status_ = model_->get(GRB_IntAttr_Status);
}

void Solver::reset() 
{
    std::cout << "Resetting the solver..." << std::endl;

    model_.reset();                               
    varmap_.clear();                              
    model_ = std::make_unique<GRBModel>(*env_);   

    status_ = GRB_LOADED;
}

double Solver::getObjectiveValue() const 
{
    return model_->get(GRB_DoubleAttr_ObjVal);
}

void Solver::setTimeLimit(double seconds) 
{
    if (!model_) 
    {
        return;
    }
    model_->set(GRB_DoubleParam_TimeLimit, seconds);
}

double Solver::getVariableValue(const std::string &name) const 
{
    auto it = varmap_.find(name);
    if (it == varmap_.end())
    {
        throw std::runtime_error("Unknown variable: " + name);
    }
    return it->second.get(GRB_DoubleAttr_X);
}

int Solver::getSolutionCount() 
{
    if (!model_) return 0;
    return model_->get(GRB_IntAttr_SolCount);
}