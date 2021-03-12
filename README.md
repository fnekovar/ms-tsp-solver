# ms-tsp-solver

Code repository for the paper:\
Multi-tour Set Traveling Salesman Problem in Planning Power Transmission Line Inspection.

Solver for multi-tour set TSP in power line inspection.
Executable argument should be path to the configuration file.

Configuration file is explained below:\
problem_format: "yaml" is currently supported, see directory problems,\
problem_file: path to the problem,\
result_dir: path to output directory,\
max_distance: maximum distance of pylon from depot used in inspection,\
max_cost: maximum flight time,\
n_agents: number of tours,\
grasp_iterations: number of GRASP solver iterations,\
max_ilp_time: maximum wall time limit for Cplex solver.

Valid solver fields are:
  * "grasp": solves using GRASP solver
  * "ilp": solves using Cplex solver
  * "both": solves using Cplex solver initialized from best GRASP solution.\
Fields describing UAV dynamics contraints are yaw_max, a_max, v_max and v_insp.\
Common value for all routes is used at the moment.

See config_grasp.cfg for GRASP solver specific parameters.
