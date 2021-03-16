//
// Created by mrs on 14.12.20.
//

#include "mstsp.h"

int Mstsp::run() {
    if (problem_format == "yaml") {
        read_yaml_data();
    } else {
        cout << "Invalid problem format specified.";
        return 1;
    }
    process_data();
    return 0;
}

void Mstsp::process_data(){
    Vector2d depot_pos(0,0);
    UavVector uavs;
    for(auto i = 0u; i < n_agents; i++) {
        Uav_Agent agent(v_max_global, v_insp_global, a_max_global, 999, yaw_max_global, max_flight_time, depot_pos, segments);
        uavs.push_back(agent);
    }
    if (solver.compare("grasp") == 0) {
        Solver_Grasp solver_grasp(uavs, segments);
        solution = solver_grasp.solve();
        string result_filename = "grasp_"+ to_string(n_agents)+"_"+to_string(max_load_distance)+"_"+to_string(max_flight_time)+".csv";
        write_csv(result_filename, solution);
    } else if (solver.compare("ilp") == 0) {
        Solver_Ilp solver_ilp(uavs[0].cost_matrix, max_flight_time, n_agents, max_ilp_time);
        solution = solver_ilp.solve();
        string result_filename = "ilp_"+ to_string(n_agents)+"_"+to_string(max_load_distance)+"_"+to_string(max_flight_time)+".csv";
        write_csv(result_filename, solution);
    } else if (solver.compare("both") == 0) {
        Solution best_solution;
        double best_cost = std::numeric_limits<double>::max();
        for (int i = 0; i < grasp_runs; i++)
        {
            Solver_Grasp solver_grasp(uavs,segments);
            solution = solver_grasp.solve();
//            string result_filename = "grasp_"+ to_string(n_agents)+"_"+to_string(max_load_distance)+"_"+to_string(max_flight_time)+"_"+to_string(i)+".csv";
//            write_csv(result_filename, solution);
            if (solution.solution_cost < best_cost){
                best_solution = solution;
                best_cost = solution.solution_cost;
            }
        }

        Solver_Ilp solver_ilp(uavs[0].cost_matrix, max_flight_time, n_agents, max_ilp_time);
        solver_ilp.setInitialSolution(best_solution);
        solution = solver_ilp.solve();
        string result_filename = "both_"+ to_string(n_agents)+"_"+to_string(max_load_distance)+"_"+to_string(max_flight_time)+".csv";
        write_csv(result_filename, solution);
    }
}

void Mstsp::read_yaml_data() {
    Config cfg_graph;
    cfg_graph.readFile(problem_file.c_str());

    string pylon_positions = "";
    string pylon_connections = "";

    cfg_graph.lookupValue("pylons_position", pylon_positions);
    cfg_graph.lookupValue("connections_indexes", pylon_connections);

    vector<Vector2d> pylons;
    Vector2d depot(0,0);

    stringstream pylons_stream(pylon_positions);
    string line;
    while (getline(pylons_stream, line)) {
        stringstream line_stream(line);
        double x, y;
        line_stream >> x;
        line_stream >> y;
        pylons.push_back(Vector2d(x, y));
    }
    auto n = pylons.size();
    vector<vector<bool>> connection_graph(n, vector<bool>(n, false));
    stringstream connections_stream(pylon_connections);
    auto i = 0u;
    while (getline(connections_stream, line)) {
        stringstream line_stream(line);
        string temp;
        int connection_index;
        while (line_stream >> temp) {
            if (stringstream(temp) >> connection_index) {
                connection_graph[i][connection_index - 1] = true;
            }
        }
        i++;
    }
    for (i = 0u; i < n; i++) {
        for (auto j = i + 1; j < n; j++) {
            if (connection_graph[i][j]) {
                vector<Vector2d> segment;
                segment.push_back(pylons.at(i));
                segment.push_back(pylons.at(j));
                if((pylons.at(i)-depot).norm() < max_load_distance and (pylons.at(j)-depot).norm() < max_load_distance) {
                    segments.push_back(segment);
                }
            }
        }
    }
}

void Mstsp::write_csv(string filename, Solution sol){
    // Make a CSV file with one column of integer values
    std::ofstream myFile(filename);
    // Send data to the stream
    myFile << sol.solution_cost << std::endl << sol.time_to_solve;
    for(auto route : sol.routes)
    {
        myFile << std::endl;
        for(auto i = 1u; i < route.size()-1; i++) {
            auto segment_id = route[i];
            Vector2d pylon0 = segments.at((segment_id/2)-1 ).at(0);
            Vector2d pylon1 = segments.at((segment_id/2)-1 ).at(1);
            if(segment_id % 2) {
                myFile << pylon1.x() << " " << pylon1.y() << ", ";
                myFile << pylon0.x() << " " << pylon0.y() << ", ";
            } else {
                myFile << pylon0.x() << " " << pylon0.y() << ", ";
                myFile << pylon1.x() << " " << pylon1.y() << ", ";
            }
        }
    }
    myFile.close();
}
