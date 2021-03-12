//
// Created by mrs on 14.12.20.
//

#ifndef MSTSP_H
#define MSTSP_H

#include "solver_grasp.h"
#include <iostream>
#include <experimental/filesystem>
#include <libconfig.h++>
#include "uav_agent.h"
#include <Eigen/Eigen>
#include <fstream>
#include "solver_ilp.h"

using namespace std;
using namespace libconfig;
using namespace Eigen;

class Mstsp {
private:
    string problem_format;
    int n_agents;
    int max_load_distance;
    int max_flight_time;
    int grasp_runs;
    int max_ilp_time;
    double v_max_global;
    double v_insp_global;
    double a_max_global;
    double yaw_max_global;
    string problem_file;
    string solver;
    Solution solution;
    std::vector<std::vector<Vector2d>> segments;
    Config cfg;
    void process_data();
    void read_yaml_data();
    void write_csv(string filename, Solution sol);
public:
    Mstsp(char* config_file) {
        cfg.readFile(config_file);
        cfg.lookupValue("problem_format", problem_format);
        cfg.lookupValue("solver", solver);
        cfg.lookupValue("n_agents", n_agents);
        cfg.lookupValue("max_distance", max_load_distance);
        cfg.lookupValue("max_cost", max_flight_time);
        cfg.lookupValue("problem_file", problem_file);
        cfg.lookupValue("grasp_iterations", grasp_runs);
        cfg.lookupValue("max_ilp_time", max_ilp_time);
        cfg.lookupValue("yaw_max", yaw_max_global);
        cfg.lookupValue("a_max", a_max_global);
        cfg.lookupValue("v_max", v_max_global);
        cfg.lookupValue("v_insp", v_insp_global);
    }
    int run();
private:
};

#endif
