//
// Created by mrs on 16.09.20.
//

#include "solver_grasp.h"

    void Solver_Grasp::load_config() {

        libconfig::Config cfg;
        cfg.readFile("config_grasp.cfg");
        cfg.lookupValue("w0", w0);
        cfg.lookupValue("p1", p1);
        cfg.lookupValue("p2", p2);
        cfg.lookupValue("R_T", R_T);

}

    void Solver_Grasp::targets_from_segments(vector<vector<Vector2d>> lines) {
        int cost_index = 2;
        for (auto line : lines)
            for (auto i = 0u; i < line.size() - 1; i++) {
                targets.push_back(TargetSet(cost_index, 0));
                cost_index += 2;
            }
    }


TargetSetVectorVector Solver_Grasp::optimize_directions(TargetSetVectorVector solution){
    TargetSetVectorVector sol_best;
    for (TargetSetVector route : solution){
        std::cout << "Cost before opt " << get_path_cost(route,0) << std::endl;
        TargetSetVector route_best;
        const std::vector<std::vector<double>> &cost_matrix = agents.at(0).cost_matrix;
        std::vector<int> indices;
        std::vector<TopoNode> nodes;
        for (TargetSet set : route){
            indices.push_back(set.index);
        }
        double cost_0 = cost_matrix[0][indices[0]];
        double cost_1 = cost_matrix[0][indices[0]+1];
        double cost_0_0, cost_0_1, cost_1_0, cost_1_1;
        nodes.push_back(TopoNode(indices[0],-1,cost_0));
        nodes.push_back(TopoNode(indices[0]+1,-1,cost_1));
        for (int i = 0; i < indices.size()-1; i++){
            cost_0_0 = nodes[i*2].cost + cost_matrix[indices[i]][indices[i+1]];
            cost_1_0 = nodes[i*2+1].cost + cost_matrix[indices[i]+1][indices[i+1]];
            cost_0_1 = nodes[i*2].cost + cost_matrix[indices[i]][indices[i+1]+1];
            cost_1_1 = nodes[i*2+1].cost + cost_matrix[indices[i]+1][indices[i+1]+1];
            if (cost_0_0 < cost_1_0) {
                nodes.push_back(TopoNode(indices[i+1],i*2, cost_0_0));
            }else{
                nodes.push_back(TopoNode(indices[i+1],i*2+1, cost_1_0));
            }
            if (cost_0_1 < cost_1_1) {
                nodes.push_back(TopoNode(indices[i+1]+1,i*2, cost_0_1));
            }else{
                nodes.push_back(TopoNode(indices[i+1]+1,i*2+1, cost_1_1));
            }
        }
        cost_0 = nodes[(indices.size()-1)*2].cost + cost_matrix[indices[indices.size()-1]][1];
        cost_1 = nodes[(indices.size()-1)*2+1].cost + cost_matrix[indices[indices.size()-1]+1][1];
        TopoNode currentNode;
        if (cost_0 < cost_1) {
            currentNode = nodes[(indices.size()-1)*2];
        }else{
            currentNode = nodes[(indices.size()-1)*2+1];
        }
        route_best.push_back(TargetSet((currentNode.index>>1)<<1,currentNode.index%2));
        //currentNode = nodes[currentNode.parent];
        while (currentNode.parent >= 0) {
            currentNode = nodes[currentNode.parent];
            route_best.push_back(TargetSet((currentNode.index>>1)<<1,currentNode.index%2));
        }
        std::reverse(route_best.begin(), route_best.end());
        std::cout << "Cost after opt " << get_path_cost(route_best,0) << std::endl;
        sol_best.push_back(route_best);
    }
    return sol_best;
}

TargetSetVectorVector Solver_Grasp::greedy_random() {

        int step = 1;
        double cost;
        int bestAgent;
        int bestTarget;
        int bestInsertIndex;
        TargetSetVector currentRoute;
        int random;
        std::set<Insertion, InsComp> possibleInsertions;

        TargetSetVectorVector currentSolution(agents.size(),TargetSetVector());

        // initial search in close neighborhood
        while (!targets.empty()) {
            for (int i = 0; i < targets.size(); i++) {
                TargetSet currentCoords = targets[i]; // hard
                for (int j = 0; j < agents.size(); j++) {
                    for (int k = 0; k <= currentSolution.at(j).size(); k++) {
                        currentRoute = currentSolution.at(j); // hard
                        currentRoute.insert(currentRoute.begin() + k, currentCoords);
                        cost = get_path_cost(currentRoute,j);
                        possibleInsertions.insert(Insertion(cost, i, 0, j, k));
                        currentRoute[k].reverse();
                        cost = get_path_cost(currentRoute,j);
                        possibleInsertions.insert(Insertion(cost, i, 1, j, k));
                    }
                }
            }

            sizeRCL = std::floor((double) possibleInsertions.size() / 4.);

            random = std::rand() % (sizeRCL + 1);
            while (random >= possibleInsertions.size()) { random--; }
            Insertion chosenInsertion = *std::next(possibleInsertions.begin(), random);

            bestTarget = chosenInsertion.TargetsIndex;
            bestAgent = chosenInsertion.Agent;
            bestInsertIndex = chosenInsertion.InsertIndex;
            possibleInsertions.clear();
            if (chosenInsertion.Direction == 1)
                targets[bestTarget].reverse();
            currentSolution[bestAgent].emplace(currentSolution[bestAgent].begin() + bestInsertIndex,
                                               targets[bestTarget]);
            targets.erase(targets.begin() + bestTarget);
            step++;
        }
        return currentSolution;
    }

    Solution Solver_Grasp::solve() {

        std::clock_t c_start = std::clock();
        std::srand(std::time(nullptr));
        Solution solution;

        TargetSetVectorVector currentSolution = greedy_random();

        bool validSolution = true;

        currentSolution = getTSSolution(currentSolution);
        currentSolution = optimize_directions(currentSolution);

        std::clock_t c_end = std::clock();

        solution.solution_cost = get_solution_cost(currentSolution);

        double time_elapsed = 1.0 * (c_end-c_start) / CLOCKS_PER_SEC;
        if (validSolution) {
            solution.time_to_solve = time_elapsed;
            for (auto route : currentSolution) {
                std::vector<int> route_indices;
                route_indices.push_back(0);
                for (auto segment : route) {
                    route_indices.push_back(segment.cost_index());
                }
                route_indices.push_back(1);
                solution.routes.push_back(route_indices);
            }
        }

        return solution;

    }



Solution Solver_Grasp::solve_gr() {

    std::clock_t c_start = std::clock();
    std::srand(std::time(nullptr));
    Solution solution;

    TargetSetVectorVector currentSolution = greedy_random();

    bool validSolution = true;

    std::clock_t c_end = std::clock();

    solution.solution_cost = get_solution_cost(currentSolution);

    double time_elapsed = 1.0 * (c_end-c_start) / CLOCKS_PER_SEC;
    if (validSolution) {
        solution.time_to_solve = time_elapsed;
        for (auto route : currentSolution) {
            std::vector<int> route_indices;
            route_indices.push_back(0);
            for (auto segment : route) {
                route_indices.push_back(segment.cost_index());
            }
            route_indices.push_back(1);
            solution.routes.push_back(route_indices);
        }
    }
    return solution;

}

TargetSetVectorVector Solver_Grasp::getTSSolution(TargetSetVectorVector &prevSol) {
    int totalScore;
        int random;

        int nodes = 0;
        for (auto line : prevSol)
            nodes += line.size();

        std::list<TargetSetVectorVector> tabuList;
        double bestSolutionLength = get_solution_cost(prevSol);

        int tabuIterationsDynamic = nodes*10;
        int bestGroup = 0;
        double bestNeighborhoodLength;
        double tabuSolutionLength;
        TargetSetVectorVector bestNeigborhoodSolution = prevSol;
        tabuList.push_back(prevSol);

        R_T_iterator = R_T;

        bool stop_crit = false;
        int iteration = 0;
        int no_improvement_iterator = 0;

        while (!stop_crit) {
            std::cout << "Iteration " << iteration << " out of " << tabuIterationsDynamic << std::endl;
            std::cout << "Best solution cost: " << bestSolutionLength << std::endl;
            std::cout << "Scores: " << g1Score << " " << g2Score << " " << g3Score << " " << g4Score << std::endl;
            iteration++;

            bestNeighborhoodLength = std::numeric_limits<double>::max();

// reinitialize score each R_t iterations
            if (++R_T_iterator >= R_T) {
                g1Score = w0;
                g2Score = w0;
                g3Score = w0;
                g4Score = w0;
                R_T_iterator = 0;
            }

// search solution neigborhood
            for (int j = 0; j < nodes; j++) {
                TargetSetVectorVector tabuSolution = bestNeigborhoodSolution;
                totalScore = g1Score + g2Score + g3Score + g4Score;
                random = (std::rand() % totalScore);
                if (random < g1Score) {
                    getG1Solution(tabuSolution);
                } else if (random < (g1Score + g2Score)) {
                    getG2Solution(tabuSolution);
                }else if (random < (g1Score + g2Score + g3Score)) {
                    getG3Solution(tabuSolution);
                } else {
                    getG4Solution(tabuSolution);
                }
                tabuSolutionLength = get_solution_cost(tabuSolution);
                if (tabuSolutionLength < bestNeighborhoodLength && std::find(tabuList.begin(), tabuList.end(), tabuSolution) == tabuList.end()) {
                    bestNeighborhoodLength = tabuSolutionLength;
                    bestNeigborhoodSolution = tabuSolution;
                    bestGroup = random;
                }
            }
            if (bestNeighborhoodLength < std::numeric_limits<double>::max()) {
                if (tabuList.size() >= nodes/4) {
                    tabuList.pop_front();
                }
                tabuList.push_back(bestNeigborhoodSolution);
                if (bestGroup < g1Score) {
                    g1Score = g1Score + p1;
                } else if (bestGroup < (g1Score + g2Score)) {
                    g2Score = g2Score + p1;
                } else if (bestGroup < (g1Score + g2Score + g3Score)) {
                    g3Score = g3Score + p1;
                } else {
                    g4Score = g4Score + p1;
                }

                if (bestNeighborhoodLength < bestSolutionLength) {
                    finalSolution = bestNeigborhoodSolution;
                    bestSolutionLength = bestNeighborhoodLength;

                    if (random < g1Score) {
                        g1Score = g1Score + p2;
                    } else if (random < (g1Score + g2Score)) {
                        g2Score = g2Score + p2;
                    } else if (random < (g1Score + g2Score + g3Score)) {
                        g3Score = g3Score + p2;
                    } else {
                        g4Score = g4Score + p2;
                    }
                    no_improvement_iterator = 0;
                }else {
                    no_improvement_iterator++;
                }
            }
            g1Score = g1Score + p1;
            if(no_improvement_iterator >= 200) {
                stop_crit = true;
            }
        }
        return finalSolution;
    }

    void Solver_Grasp::getG1Solution(TargetSetVectorVector &prevSol) // random shift intra-inter route
    {
        int routes = prevSol.size();
        int indexA1 = std::rand() % routes;
        int indexA2;
        while (prevSol.at(indexA1).size() < 2) {
            indexA1 = std::rand() % routes;
        }
        int indexC1 = std::rand() % prevSol.at(indexA1).size();
        int indexC2;
        TargetSet movedCoord = prevSol.at(indexA1).at(indexC1);
        prevSol.at(indexA1).erase(prevSol.at(indexA1).begin() + indexC1);

        if (std::rand() % 2 < 1) { // shift intra route
            indexA2 = indexA1;
            do {
                indexC2 = std::rand() % (prevSol.at(indexA1).size() + 1);
            } while (indexC1 == indexC2);
            prevSol[indexA2].emplace(prevSol.at(indexA1).begin() + indexC2, movedCoord);
        } else { // shift inter route
            do {
                indexA2 = std::rand() % routes;
            } while (indexA1 == indexA2);
            indexC2 = std::rand() % (prevSol.at(indexA2).size() + 1);
            prevSol.at(indexA2).insert(prevSol.at(indexA2).begin() + indexC2, movedCoord);
        }
        //double routeCost = get_solution_cost(prevSol);
        double route_cost1 = get_path_cost(prevSol[indexA1],indexA1) + get_path_cost(prevSol[indexA2],indexA2);
        prevSol[indexA2][indexC2].reverse();
        double route_cost2 = get_path_cost(prevSol[indexA1],indexA1) + get_path_cost(prevSol[indexA2],indexA2);
        if (route_cost1 < route_cost2)
            prevSol[indexA2][indexC2].reverse();
    }

    void Solver_Grasp::getG4Solution(TargetSetVectorVector &prevSol) {
        int A = std::rand() % prevSol.size();
        while(prevSol.at(A).size() <= 0) {
            A = std::rand() % prevSol.size();
        }
        int C = std::rand() % prevSol.at(A).size();
        prevSol.at(A).at(C).reverse();
    }


    void Solver_Grasp::getG2Solution(
            TargetSetVectorVector &prevSol) // best shift intra-inter route based on exhaustive search
    {
        //DEBUG("G2 invoked");
        int routes = prevSol.size();
        int indexA1 = std::rand() % routes;
        while (prevSol.at(indexA1).size() < 2) {
            indexA1 = std::rand() % routes;
        }
        int indexC1 = std::rand() % prevSol.at(indexA1).size();
        TargetSet movedCoord = prevSol.at(indexA1).at(indexC1);
        prevSol.at(indexA1).erase(prevSol.at(indexA1).begin() + indexC1);

        double bestSolutionLength = std::numeric_limits<double>::max();
        double solutionLength1;
        double solutionLength2;
        bool reverse1 = false;

        TargetSetVectorVector intermediateSolution = prevSol;

        for (int i = 0; i < agents.size(); i++) {
            for (int j = 1; j <= prevSol.at(i).size(); j++) {
                if (!((i == indexA1) && (j == indexC1))) {
                    intermediateSolution.at(i).insert(intermediateSolution.at(i).begin() + j, movedCoord);
                    //solutionLength1 = get_solution_cost(intermediateSolution);
                    solutionLength1 = get_path_cost(intermediateSolution[indexA1],indexA1) + get_path_cost(intermediateSolution[i],i);
                    intermediateSolution.at(i).at(j).reverse();
                    //solutionLength2 = get_solution_cost(intermediateSolution);
                    solutionLength2 = get_path_cost(intermediateSolution[indexA1],indexA1) + get_path_cost(intermediateSolution[i],i);
                    intermediateSolution.at(i).at(j).reverse();
                    if (solutionLength1 < bestSolutionLength) {
                        indexA1 = i;
                        indexC1 = j;
                        bestSolutionLength = solutionLength1;
                        reverse1 = false;
                    } else if (solutionLength2 < bestSolutionLength) {
                        indexA1 = i;
                        indexC1 = j;
                        bestSolutionLength = solutionLength2;
                        reverse1 = true;
                    }
                    intermediateSolution = prevSol;
                }
            }
        }

        if (reverse1)
            movedCoord.reverse();
        prevSol.at(indexA1).insert(prevSol.at(indexA1).begin() + indexC1, movedCoord);
    }

    void Solver_Grasp::getG3Solution(
            TargetSetVectorVector &prevSol) // best swapt intra-inter route based on exhaustive search
    {
        int routes = prevSol.size();
        int indexA1 = std::rand() % routes;
        while (prevSol.at(indexA1).size() < 2) {
            indexA1 = std::rand() % routes;
        }

        int indexC1 = std::rand() % prevSol.at(indexA1).size();
        int indexA2;
        int indexC2;
        double reverse1 = false;
        double reverse2 = false;
//        std::shared_ptr<TargetSet> movedTargetSet = prevSol.at(indexA1).at(indexC1);

        double bestSolutionLength = std::numeric_limits<double>::max();
        double solutionLength1;
        double solutionLength2;
        double solutionLength3;
        double solutionLength4;

        TargetSetVectorVector intermediateSolution = prevSol;

        for (int i = 0; i < routes; i++) {
            for (int j = 0; j < prevSol.at(i).size(); j++) {
                if (!((i == indexA1) && (j == indexC1))) {
                    std::swap(intermediateSolution[indexA1][indexC1], intermediateSolution[i][j]);
                    //solutionLength1 = get_solution_cost(intermediateSolution);
                    solutionLength1 = get_path_cost(intermediateSolution[indexA1],indexA1) + get_path_cost(intermediateSolution[i],i);
                    intermediateSolution[indexA1][indexC1].reverse();
                    //solutionLength2 = get_solution_cost(intermediateSolution);
                    solutionLength2 = get_path_cost(intermediateSolution[indexA1],indexA1) + get_path_cost(intermediateSolution[i],i);
                    intermediateSolution[i][j].reverse();
                    //solutionLength3 = get_solution_cost(intermediateSolution);
                    solutionLength3 = get_path_cost(intermediateSolution[indexA1],indexA1) + get_path_cost(intermediateSolution[i],i);
                    intermediateSolution[indexA1][indexC1].reverse();
                    //solutionLength4 = get_solution_cost(intermediateSolution);
                    solutionLength4 = get_path_cost(intermediateSolution[indexA1],indexA1) + get_path_cost(intermediateSolution[i],i);
                    intermediateSolution[i][j].reverse();

                    double solutionLength = std::min(
                            std::min(std::min(solutionLength1, solutionLength2), solutionLength3),
                            solutionLength4);

                    if (solutionLength < bestSolutionLength) {
                        if (solutionLength1 <= solutionLength) {
                            reverse1 = false;
                            reverse2 = false;
                        } else if (solutionLength2 <= solutionLength) {
                            reverse1 = true;
                            reverse2 = false;
                        } else if (solutionLength3 <= solutionLength) {
                            reverse1 = true;
                            reverse2 = true;
                        } else {
                            reverse1 = false;
                            reverse2 = true;
                        }
                        indexA2 = i;
                        indexC2 = j;
                        bestSolutionLength = solutionLength;
                    }
                }
            }
        }
        std::swap(prevSol[indexA1][indexC1], prevSol[indexA2][indexC2]);
        if (reverse1)
            prevSol[indexA1][indexC1].reverse();
        if (reverse2)
            prevSol[indexA2][indexC2].reverse();
    }

    double Solver_Grasp::get_path_cost(TargetSetVector &targetsets, const int agent_index) {
        if (targetsets.size() < 1) {
            return 0;
        }
        const std::vector<std::vector<double>> &cost_matrix = agents.at(agent_index).cost_matrix;
        double max_cost = agents.at(agent_index).max_t;
        double len = 0;
        len += cost_matrix[0][targetsets.front().cost_index()]; // leaving depot
        len += cost_matrix[targetsets.back().cost_index()][1]; // entering depot

        for (unsigned long i = 0; i < targetsets.size()-1; i++) {
            len += cost_matrix[targetsets[i].cost_index()][targetsets[i+1].cost_index()];
        }
        if (len > max_cost) {
            len += (len - max_cost) * 1000;
        }
        return len;
    }

    double Solver_Grasp::get_solution_cost(TargetSetVectorVector &paths) {
        double totalCost = 0;
        for (unsigned long i = 0; i < paths.size(); i++) {
            totalCost = totalCost + get_path_cost(paths.at(i),i);
        }
        return totalCost;
    }

