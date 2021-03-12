#include "solver_ilp.h"
#include <cmath>
#include <limits>


Solution Solver_Ilp::solve() const {
    const auto n = cost_matrix.size();
    // CPLEX environment. Takes care of everything, including memory management for CPLEX objects.
    IloEnv env;



    // CPLEX model. We put variables and constraints in it!
    IloModel model(env);

    // Model:
    //
    // BINARY VARIABLE x[i][j]    For all i,j = 0, ..., n - 1
    //    x[i][j] == 1            If arc (i,j) is selected
    //    x[i][j] == 0            Otherwise
    //
    // INTEGER VARIABLE t[i]      For all i = 0, ..., n - 1
    //    t[i] == k               Iff node i is the k-th node in the tour
    //    t[0] == 1
    //    t[i] in [2, ..., n]     For all i = 1, ... n - 1
    //
    // OBJECTIVE FUNCTION
    //    MIN sum((i,j), c[i][j] * x[i][j])
    //
    // CONSTRAINTS
    //    1) sum(j, x[j][i]) == 1                    For all i
    //    2) sum(j, x[i][j]) == 1                    For all i
    //    3) t[i] - t[j] + 1 <= n * (1 - x[i][j])    For all i,j = 1, ..., n - 1
    //       Can be written as:
    //       t[i] - t[j] + n * x[i][j] <= n - 1

    // We use this stringstream to create variable and constraint names
    std::stringstream name;
    // Constraints
    // IloRangeArray inbound_arcs(env, n);  // Constraints 1)
    //IloRangeArray outbound_arcs(env, n); // Constraints 2)

    //IloRangeArray mandatory_arcs(env, n*(n-1)/2);
    IloArray<IloArray<IloRangeArray>> mtz(env, m); // Constraints 3)
    // Variables
    IloArray<IloArray<IloNumVarArray>> x(env, m);
    IloArray<IloNumVarArray> t(env, m);

    IloExpr expr(env);
    IloExpr in_expr(env);
    IloExpr out_expr(env);

    // Create variables x
    for (auto k = 0u; k < m; ++k) {
        IloArray<IloNumVarArray> x_k(env, n);
        for (auto i = 0u; i < n; ++i) {
            x_k[i] = IloNumVarArray(env, n);
            for (auto j = 0u; j < n; ++j) {
                name << "x_" << k << "_" << i << "_" << j;
                x_k[i][j] = IloNumVar(env, 0, 1, IloNumVar::Bool, name.str().c_str());
                name.str(""); // Clean name
            }
        }
        x[k] = x_k;
    }

    for (auto i = 0u; i < m; ++i) {
        IloNumVarArray t_i(env, n);
        // Create variable t[0] and fix it to value 1
        // This breaks symmetry, because it fixes node 0 as the starting node of the tour
        name << "t_" << i << "_0";
        t_i[0] = IloNumVar(env, 1, 1, IloNumVar::Int, name.str().c_str());
        name.str("");
        // Create variables t[1], ..., t[n]
        for (auto j = 1u; j < n; ++j) {
            name << "t_" << i << "_" << j;
            t_i[j] = IloNumVar(env, 2, n, IloNumVar::Int, name.str().c_str());
            name.str(""); // Clean name
        }
        t[i] = t_i;
    }

// Fix number of inbound and outbound arcs coming to and from start node respectively to 1 for each group.
    for (auto k = 0u; k < m; k++) {
        for (auto i = 2u; i < n; ++i) {
            in_expr += x[k][i][1];
        }

        name << "inbound_0_" << k;
        IloRange inbound_arcs(env, 1, in_expr, 1, name.str().c_str());
        name.str(""); // Clean name
        in_expr.clear(); // Clean expr
        model.add(inbound_arcs);

        for (auto i = 2u; i < n; ++i) {
            out_expr += x[k][0][i];
        }
        name << "outbound_0_" << k;
        IloRange outbound_arcs(env, 1, out_expr, 1, name.str().c_str());
        name.str(""); // Clean name
        out_expr.clear(); // Clean expr
        model.add(outbound_arcs);
    }


    for (auto k = 0u; k < m; k++) {
        expr += x[k][0][0] + x[k][1][0] + x[k][0][1] + x[k][1][1];
        for (auto i = 2u; i < n; ++i) {
            expr += x[k][1][i] + x[k][i][0];
        }
    }
    IloRange forbidden0nodes(env, 0, expr, 0, "forbidden_0_nodes");
    model.add(forbidden0nodes);
    expr.clear();

    // inbount and outbound constrains on entss
    out_expr.clear(); // Clean expr
    in_expr.clear(); // Clean expr
    expr.clear();
    for (auto k = 0u; k < m; k++) {
        for (auto i = 2u; i < n; ++i) {
            for (auto j = 0u; j < n; ++j) {
                if (i != j) {
                    out_expr += x[k][i][j];
                    in_expr += x[k][j][i];
                }
            }
            expr += out_expr;
            expr -= in_expr;
            name << "out_" << k << "_" << i;
            IloRange out(env, 0, out_expr, 1, name.str().c_str());
            name.str(""); // Clean name
            model.add(out);
            name << "in_" << k << "_" << i;
            IloRange in(env, 0, in_expr, 1, name.str().c_str());
            name.str(""); // Clean name
            model.add(in);
            name << "inout_" << k << "_" << i;
            IloRange inout(env, 0, expr, 0, name.str().c_str());
            name.str(""); // Clean name
            model.add(inout);
            expr.clear(); // Clean expr
            out_expr.clear(); // Clean expr
            in_expr.clear(); // Clean expr
        }
    }
    for (auto i = 2u; i < n; i += 2) {
        for (auto k = 0u; k < m; ++k) {
            expr += x[k][i][i] + x[k][i + 1][i] + x[k][i][i + 1] + x[k][i + 1][i + 1];
        }
    }
    name << "forbidden_segments";
    IloRange forb(env, 0, expr, 0, name.str().c_str());
    name.str(""); // Clean name
    model.add(forb);
    expr.clear();

    // fix entries and exists of each segment to one
    for (auto i = 2u; i < n; i += 2) {
        for (auto j = 0u; j < n; ++j) {
            for (auto k = 0u; k < m; ++k) {
                out_expr += x[k][j][i] + x[k][j][i + 1];
                in_expr += x[k][i][j] + x[k][i + 1][j];
            }
        }

        auto seg_number = i / 2;

        name << "out_segment_" << seg_number;
        IloRange out(env, 1, out_expr, 1, name.str().c_str());
        name.str(""); // Clean name
        model.add(out);
        name << "in_segment_" << seg_number;
        IloRange in(env, 1, in_expr, 1, name.str().c_str());
        name.str(""); // Clean name
        model.add(in);
        in_expr.clear();
        out_expr.clear();
    }

    // Create constraints to avoid nested loops.
    for (auto m_it = 0u; m_it < m; ++m_it) {
        // The constraint is for i = 1,...,n and therefore we add empty constraints for i == 0
        IloArray<IloRangeArray> mtz_i(env, n);
        mtz_i[0] = IloRangeArray(env);
        // We then continue normally for all other i > 0
        for (auto i = 1u; i < n; ++i) {
            mtz_i[i] = IloRangeArray(env, n);
            for (auto j = 1u; j < n; ++j) {
                expr = t[m_it][i] - t[m_it][j] + static_cast<int>(n) * x[m_it][i][j];

                name << "mtz_" << m_it << "_" << i << "_" << j;
                mtz_i[i][j] = IloRange(env, -IloInfinity, expr, n - 1, name.str().c_str());
                name.str(""); // Clean name
                expr.clear(); // Clean expr
            }
            model.add(mtz_i[i]);
        }
        mtz[m_it] = mtz_i;
    }


    // Create objective function
    for (auto k = 0u; k < m; k++) {
        for (auto i = 0u; i < n; ++i) {
            for (auto j = 0u; j < n; ++j) {
                expr += cost_matrix[i][j] * x[k][i][j];
            }
        }
    }
    IloObjective obj(env, expr, IloObjective::Minimize);

    // Add the objective function to the model
    model.add(obj);

    // Create constraint on maximum budget
    expr.clear();
    for (auto k = 0u; k < m; k++) {
        for (auto i = 0u; i < n; ++i) {
            for (auto j = 0u; j < n; ++j) {
                expr += cost_matrix[i][j] * x[k][i][j];
            }
        }
        name << "budget_" << k;
        IloRange budget(env, 0, expr, max_cost, name.str().c_str());
        name.str("");
        expr.clear();
        model.add(budget);
    }
    // Free the memory used by expr
    expr.end();



    // Create the solver object
    IloCplex cplex(model);

    // Initial solution

    IloNumVarArray init_var(env);
    IloNumArray init_val(env);
    if (init_t.size() > 0) {
        for (auto k = 0u; k < m; k++) {
            for (auto i = 1u; i < n; i++) { // t[k][0] is fixed at 1
                init_var.add(t[k][i]);
                init_val.add(init_t[k][i]);
            }
        }
    }
    if (init_x.size() > 0) {
        for (auto k = 0u; k < m; k++) {
            for (auto i = 0u; i < n; i++) {
                for (auto j = 0u; j < n; j++) {
                    init_var.add(x[k][i][j]);
                    init_val.add(init_x[k][i][j]);
                }
            }
        }
    }
    cplex.addMIPStart(init_var, init_val);
    init_var.end();
    init_val.end();

    cplex.setParam(IloCplex::Param::TimeLimit, max_time);
//    cplex.setParam(IloCplex::Param::Threads,cores);

    // Export model to file (useful for debugging!)
    cplex.exportModel("model.lp");

    bool solved = false;

    IloNum sTime;
    IloNum eTime;

    try {
        // Try to solve with CPLEX (and hope it does not raise an exception!)
        sTime = cplex.getCplexTime();
        solved = cplex.solve();
        eTime = cplex.getCplexTime();
    } catch (const IloException &e) {
        std::cerr << "\n\nCPLEX Raised an exception:\n";
        std::cerr << e << "\n";
        env.end();
        throw;
    }
    Solution solution;

    if (solved) {
        // If CPLEX successfully solved the model, print the results
        std::cout.precision(5);
        std::cout << "\n\nCplex success!\n";
        std::cout << "\tStatus: " << cplex.getStatus() << "\n";
        std::cout << "\tObjective value: " << cplex.getObjValue() << "\n";

        solution.routes = getRoutes(cplex, x, t);
        solution.solution_cost = cplex.getObjValue();
        solution.time_to_solve = eTime - sTime;
        solution.solver = "ILP";
    } else {
        std::cerr << "\n\nCplex error!\n";
        std::cerr << "\tStatus: " << cplex.getStatus() << "\n";
        std::cerr << "\tSolver status: " << cplex.getCplexStatus() << "\n";
    }

    env.end();
    return solution;
}

std::vector<std::vector<int>> Solver_Ilp::getRoutes(const IloCplex &cplex, const IloArray<IloArray<IloNumVarArray>> &x,
                                                    const IloArray<IloNumVarArray> &t) const {
    const auto n = cost_matrix.size();
    std::vector<std::vector<int>> lines;
    for (auto k = 0u; k < m; k++) {
        std::vector<int> line;
        auto starting_vertex = 0u;
        auto current_vertex = starting_vertex;
        line.push_back(starting_vertex);
        do {
            for (auto i = 0; i < n; ++i) {
                if (cplex.getValue(x[k][current_vertex][i]) > .5) {
                    current_vertex = i;
//                      std::cout << unsigned(current_vertex) << " ";
                    line.push_back(current_vertex);
                    break;
                }
            }
        } while (current_vertex != 1);
        lines.push_back(line);
    }
    return lines;
}

void Solver_Ilp::setInitialSolution(Solution init) {
    auto n = cost_matrix.size();
    for (auto ids : init.routes) {
        std::vector<std::vector<int>> x_i(n, std::vector<int>(n, 0));
        std::vector<int> t_i(n, n);
        if (!ids.empty()) {
            for (auto i = 0; i < ids.size(); i++) {
                t_i[ids[i]] = i + 1;
            }
            for (auto i = 0u; i < ids.size() - 1; i++) {
                x_i[ids[i]][ids[i + 1]] = 1;
            }
        }
        init_t.push_back(t_i);
        init_x.push_back(x_i);
    }
}
