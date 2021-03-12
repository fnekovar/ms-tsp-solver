//
// Created by mrs on 15.09.20.
//

#ifndef msctsp_SOLUTION_H
#define msctsp_SOLUTION_H

#include <vector>
#include <iostream>
#include <cmath>
#include "coords.h"
#include "target.h"

    struct Solution{
        std::vector<std::vector<int>> routes;
        std::string solver;
        double solution_cost = -1;
        double time_to_solve = -1;
    };

#endif //msctsp_SOLUTION_H
