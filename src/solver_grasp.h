//
// Created by mrs on 16.09.20.
//

#ifndef MSCTSP_SOLVER_GRASP_H
#define MSCTSP_SOLVER_GRASP_H

#include <iostream>
#include <cstring>
#include <memory>
#include "solution.h"
#include "target.h"
#include "target_shift.h"
#include "insertion.h"
#include "uav_agent.h"
#include <set>
#include <list>
#include <algorithm>
#include <ctime>

    class Solver_Grasp {
        UavVector agents;
        TargetSetVector targets;
        TargetSetVectorVector finalSolution;
        double g1Score;
        double g2Score;
        double g3Score;
        double g4Score;
        double R_T_iterator;
        int sizeRCL;
        const double p1 = 1; // best group solution (p for prize)
        const double p2 = 5; // best tabu solution
        //const int R_T = 5;
        const int w0 = 5;

        void targets_from_segments(vector<vector<Vector2d>> segments);

    public:
        Solver_Grasp(const UavVector agents, const vector<vector<Vector2d>> segments)
                : agents{agents} {
            targets_from_segments(segments);
        }
        TargetSetVectorVector greedy_random();
        Solution solve();
        TargetSetVectorVector getTSSolution(TargetSetVectorVector& prevSol);
        void getG1Solution(TargetSetVectorVector& prevSol);
        void getG2Solution(TargetSetVectorVector& prevSol);
        void getG3Solution(TargetSetVectorVector& prevSol);
        void getG4Solution(TargetSetVectorVector& prevSol);
        double get_path_cost(TargetSetVector &pts, const int agent_index);
        double get_solution_cost(TargetSetVectorVector &paths);
    };

#endif //MSCTSP_SOLVER_GRASP_H