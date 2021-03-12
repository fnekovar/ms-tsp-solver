#ifndef SOLVER_H
#define SOLVER_H

#include <iostream>
#include "solution.h"

// Magic tricks to have CPLEX behave well:
#ifndef IL_STD
#define IL_STD
#endif
#include <cstring>
#include <ilcplex/ilocplex.h>
ILOSTLBEGIN
// End magic tricks

  class Solver_Ilp {
    const std::vector<std::vector<double>> cost_matrix;
    const uint32_t max_cost;
    const uint32_t m;
    const uint32_t max_time;
    std::vector<std::vector<std::vector<int>>> init_x;
    std::vector<std::vector<int>> init_t;
    std::vector<std::vector<int>> getRoutes(const IloCplex& cplex, const IloArray<IloArray<IloNumVarArray>>& x, const IloArray<IloNumVarArray>& t) const;
  public:
    void setInitialSolution(Solution init);
    explicit Solver_Ilp(const std::vector<std::vector<double>> cost_matrix, const uint32_t max_cost, const uint32_t m, const uint32_t max_time) : cost_matrix{cost_matrix}, max_cost{max_cost}, m{m}, max_time{max_time} {}
    Solution solve() const;
  };

#endif