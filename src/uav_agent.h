//
// Created by mrs on 03.12.20.
//

#ifndef MSCTSP_UAV_AGENT_H
#define MSCTSP_UAV_AGENT_H

#include <Eigen/Eigen>

using namespace std;
using namespace Eigen;

class Uav_Agent {
public:
    Uav_Agent(double v_max, double v_inspection, double a_max, double j_max, double yaw_max, int max_t, Vector2d coord_0, vector<vector<Vector2d>> lines) : v_max(v_max), v_inspection(v_inspection), a_max(a_max), j_max(j_max), yaw_max(yaw_max), max_t(max_t) {
        generate_cost_matrix_straightline(coord_0, lines);
    }

    std::vector<std::vector<double>> cost_matrix; // cost matrix for segments
    double yaw_max;
    double v_inspection;
    double v_max; // maximum velocity
    double a_max; // acceleration
    double j_max; // jerk
    int max_t;
    size_t graphSize;


    double estimateTimeLines(const vector<Vector2d>& vertices) { // 4 vertices
        Vector2d origin = vertices[1];
        Vector2d start = vertices[2];
        Vector2d end = vertices[3];
        Vector2d vec0 = vertices[1] - vertices[0];
        Vector2d vec1 = vertices[2] - vertices[1];
        Vector2d vec2 = vertices[3] - vertices[2];
        vec0.normalize();
        vec1.normalize();
        vec2.normalize();
        double angle1 = atan2(vec1[1], vec1[0]) - atan2(vec0[1], vec0[0]);
        double angle2 = atan2(vec2[1], vec2[0]) - atan2(vec1[1], vec1[0]);
        if (angle1 > M_PI)        { angle1 -= 2 * M_PI; }
        else if (angle1 <= -M_PI) { angle1 += 2 * M_PI; }
        angle1 = abs(angle1);
        if (angle2 > M_PI)        { angle2 -= 2 * M_PI; }
        else if (angle2 <= -M_PI) { angle2 += 2 * M_PI; }
        angle2 = abs(angle2);
        double turnTime1 = angle1 / yaw_max;
        double turnTime2 = angle2 / yaw_max;
        double distAccTraverse = v_max*v_max/a_max;
        double distAccInspection = v_inspection*v_inspection/a_max;
        double dist_traverse = (start - origin).norm();
        double dist_inspect = (end - start).norm();
        double timeTraverse = (dist_traverse - distAccTraverse)/v_max;
        double timeInspection = (dist_inspect - distAccInspection)/v_inspection;
        return turnTime1 + timeTraverse + turnTime2 + timeInspection + 2*v_max/a_max + 2*v_inspection/a_max;
//        return timeTraverse + timeInspection + 2*v_max/a_max + 2*v_inspection/a_max;
    }

    double estimateTimeEnd(const vector<Vector2d>& vertices) {
        Vector2d origin = vertices[1];
        Vector2d depot = vertices[2];
        Vector2d vec0 = vertices[1] - vertices[0];
        Vector2d vec1 = vertices[2] - vertices[1];
        vec0.normalize();
        vec1.normalize();
        double angle1 = atan2(vec1[1], vec1[0]) - atan2(vec0[1], vec0[0]);
        if (angle1 > M_PI)        { angle1 -= 2 * M_PI; }
        else if (angle1 <= -M_PI) { angle1 += 2 * M_PI; }
        angle1 = abs(angle1);
        double turnTime1 = angle1 / yaw_max;
        double distAccTraverse = v_max*v_max/a_max;
        double dist_traverse = (depot - origin).norm();
        double timeTraverse = (dist_traverse - distAccTraverse)/v_max;
        return turnTime1 + timeTraverse + 2*v_max/a_max;
//        return timeTraverse + 2*v_max/a_max;
    }

    void generate_cost_matrix_straightline(Vector2d coord_0, vector<vector<Vector2d>> lines) {
        graphSize = 2 + lines.size()*2;
        cost_matrix = vector<vector<double>>(graphSize,vector<double>(graphSize));
        // fill non-traversable edges with, well ... anything, except float inf, that breaks cplex
        // auto inf = std::numeric_limits<float>::infinity();
        auto inf = 0;
        // total number of nodes is 2 for start/end node and n*l for n nodes in l lines
        // start node 0 and end node 1
        // end
        auto offset = 2u;
        cost_matrix[0][0] = inf;
        cost_matrix[1][0] = inf;
        cost_matrix[1][1] = inf;
        cost_matrix[0][1] = inf;
        // edges ending at depot
        for (auto i = 0u; i < lines.size(); ++i) {
            vector<Vector2d> vec01 {lines[i][1], lines[i][0], coord_0}; // B-A-0
            vector<Vector2d> vec02 {lines[i][0], lines[i][1], coord_0}; // A-B-0
            auto dist01 = estimateTimeEnd(vec01);
            auto dist02 = estimateTimeEnd(vec02);
            cost_matrix[1][offset] = inf;
            cost_matrix[1][offset+1] = inf;
            cost_matrix[offset][1] = dist02; // reverse order as in incoming
            cost_matrix[offset+1][1] = dist01;
            offset += 2;
        }
        // edges starting at depot
        offset = 2u;
        for (auto i = 0u; i < lines.size(); ++i) {
            vector<Vector2d> vec01 {Vector2d(-1,0), coord_0, lines[i][0], lines[i][1]}; // 0-A-B
            vector<Vector2d> vec02 {Vector2d(-1,0), coord_0, lines[i][1], lines[i][0]}; // 0-B-A
            auto dist01 = estimateTimeLines(vec01);
            auto dist02 = estimateTimeLines(vec02);
            cost_matrix[0][offset] = dist01;
            cost_matrix[0][offset+1] = dist02;
            cost_matrix[offset][0] = inf;
            cost_matrix[offset+1][0] = inf;
            offset += 2;
        }

        offset = 2u;

        for (auto l = 0u; l < lines.size(); ++l) {
            cost_matrix[offset][offset] = inf;
            cost_matrix[offset + 1][offset] = inf;
            cost_matrix[offset + 1][offset + 1] = inf;
            cost_matrix[offset][offset + 1] = inf;
            auto off_y = offset;
            for(auto k = l+1; k < lines.size(); ++k){
                off_y += (lines[k-1].size()-1)*2;
                auto off_x = offset;
                vector<Vector2d> vec1 {lines[l][0], lines[l][1], lines[k][0], lines[k][1]}; // 1234
                vector<Vector2d> vec2 {lines[l][0], lines[l][1], lines[k][1], lines[k][0]}; // 1243
                vector<Vector2d> vec3 {lines[l][1], lines[l][0], lines[k][0], lines[k][1]}; // 2134
                vector<Vector2d> vec4 {lines[l][1], lines[l][0], lines[k][1], lines[k][0]}; // 2143
                vector<Vector2d> vec5 {lines[k][0], lines[k][1], lines[l][0], lines[l][1]}; // 3412
                vector<Vector2d> vec6 {lines[k][0], lines[k][1], lines[l][1], lines[l][0]}; // 3421
                vector<Vector2d> vec7 {lines[k][1], lines[k][0], lines[l][0], lines[l][1]}; // 4312
                vector<Vector2d> vec8 {lines[k][1], lines[k][0], lines[l][1], lines[l][0]}; // 4321
                auto dist1 = estimateTimeLines(vec1);
                auto dist2 = estimateTimeLines(vec2);
                auto dist3 = estimateTimeLines(vec3);
                auto dist4 = estimateTimeLines(vec4);
                auto dist5 = estimateTimeLines(vec5);
                auto dist6 = estimateTimeLines(vec6);
                auto dist7 = estimateTimeLines(vec7);
                auto dist8 = estimateTimeLines(vec8);
                cost_matrix[off_x][off_y] = dist1;
                cost_matrix[off_x][off_y + 1] = dist2;
                cost_matrix[off_x + 1][off_y + 1] = dist4;
                cost_matrix[off_x + 1][off_y] = dist3;
                cost_matrix[off_y][off_x] = dist5;
                cost_matrix[off_y][off_x + 1] = dist6;
                cost_matrix[off_y + 1][off_x + 1] = dist8;
                cost_matrix[off_y + 1][off_x] = dist7;
            }
            offset += 2;
        }
    }
};

typedef std::vector<Uav_Agent> UavVector;

#endif //MSCTSP_UAV_AGENT_H
