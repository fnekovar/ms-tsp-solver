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

    void generate_cost_matrix_bacastyle(Vector2d coord_0, vector<vector<Vector2d>> lines) {
        graphSize = 2 + lines.size()*2;
        cost_matrix = vector<vector<double>>(graphSize,vector<double>(graphSize));
        // fill non-traversable edges with, well ... anything, except float inf, that breaks cplex
        // auto inf = std::numeric_limits<float>::infinity();
        auto inf = 0;
        // total number of nodes is 1 for start node and n*l for n nodes in l lines
        // start node 0 and end node 0
        // end
        auto offset = 2u;
        cost_matrix[0][0] = inf;
        cost_matrix[1][0] = inf;
        cost_matrix[1][1] = inf;
        cost_matrix[0][1] = inf;
        // edges ending at depot
        for (auto i = 0u; i < lines.size(); ++i) {
            vector<Vector2d> vec01 {2*lines[i][1]-lines[i][0], lines[i][1], lines[i][0], coord_0};
            vector<Vector2d> vec02 {2*lines[i][0]-lines[i][1], lines[i][0], lines[i][1], coord_0};
            auto dist01 = estimateTimes(vec01);
            auto dist02 = estimateTimes(vec02);
            cost_matrix[1][offset] = inf;
            cost_matrix[1][offset+1] = inf;
            cost_matrix[offset][1] = dist02[2]; // reverse order as in incoming
            cost_matrix[offset+1][1] = dist01[2];
            offset += 2;
        }
        // edges starting at depot
        offset = 2u;
        for (auto i = 0u; i < lines.size(); ++i) {
            vector<Vector2d> vec01 {coord_0, lines[i][0], lines[i][1], 2*lines[i][1]-lines[i][0]};
            vector<Vector2d> vec02 {coord_0, lines[i][1], lines[i][0], 2*lines[i][0]-lines[i][1]};
            auto dist01 = estimateTimes(vec01);
            auto dist02 = estimateTimes(vec02);
            cost_matrix[0][offset] = dist01[0]+dist01[1];
            cost_matrix[0][offset+1] = dist02[0]+dist02[1];
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
                vector<Vector2d> vec1 {2*lines[l][0]-lines[l][1], lines[l][0], lines[l][1], lines[k][0], lines[k][1], 2*lines[k][1]-lines[k][0] }; // 1234
                vector<Vector2d> vec2 {2*lines[l][0]-lines[l][1], lines[l][0], lines[l][1], lines[k][1], lines[k][0], 2*lines[k][0]-lines[k][1] }; // 1243
                vector<Vector2d> vec3 {2*lines[l][1]-lines[l][0], lines[l][1], lines[l][0], lines[k][0], lines[k][1], 2*lines[k][1]-lines[k][0] }; // 2134
                vector<Vector2d> vec4 {2*lines[l][1]-lines[l][0], lines[l][1], lines[l][0], lines[k][1], lines[k][0], 2*lines[k][0]-lines[k][1] };// 2143
                auto dist1 = estimateTimes(vec1);
                auto dist2 = estimateTimes(vec2);
                auto dist3 = estimateTimes(vec3);
                auto dist4 = estimateTimes(vec4);
                cost_matrix[off_x][off_y] = dist1[2] + dist1[3];
                cost_matrix[off_x][off_y + 1] = dist2[2] + dist2[3];
                cost_matrix[off_x + 1][off_y + 1] = dist4[2] + dist4[3];
                cost_matrix[off_x + 1][off_y] = dist3[2] + dist3[3];
                cost_matrix[off_y][off_x] = dist4[2] + dist4[1];
                cost_matrix[off_y][off_x + 1] = dist2[2] + dist2[1];
                cost_matrix[off_y + 1][off_x + 1] = dist1[2] + dist1[1];
                cost_matrix[off_y + 1][off_x] = dist3[2] + dist3[1];
            }
            offset += 2;
        }
    }
    vector<double> estimateTimes(const vector<Vector2d> &vertices) {

        vector<double> segment_times;
        segment_times.reserve(vertices.size() - 1);

        // for each vertex in the path
        for (size_t i = 0; i < vertices.size() - 1; ++i) {

            Vector2d start = vertices[i];
            Vector2d end = vertices[i+1];

            double acceleration_time_1 = 0;
            double acceleration_time_2 = 0;

            double jerk_time_1 = 0;
            double jerk_time_2 = 0;

            double acc_1_coeff = 0;
            double acc_2_coeff = 0;

            double distance = (end - start).norm();

            if (i >= 1) {

                Vector2d pre = vertices[i-1];
                Vector2d vec1 = start - pre;
                Vector2d vec2 = end - start;

                vec1.normalize();
                vec2.normalize();

                double scalar = vec1.dot(vec2) < 0 ? 0.0 : vec1.dot(vec2);

                acc_1_coeff = (1 - scalar);

                acceleration_time_1 = acc_1_coeff * ((v_max / a_max) + (a_max / j_max));

                jerk_time_1 = acc_1_coeff * (2 * (a_max / j_max));

            }

            // the first vertex
            if (i == 0) {
                acc_1_coeff = 1.0;
                acceleration_time_1 = (v_max / a_max) + (a_max / j_max);
                jerk_time_1 = (2 * (a_max / j_max));
            }

            // last vertex
            if (i == vertices.size() - 2) {
                acc_2_coeff = 1.0;
                acceleration_time_2 = (v_max / a_max) + (a_max / j_max);
                jerk_time_2 = (2 * (a_max / j_max));
            }

            // a vertex
            if (i < vertices.size() - 2) {

                Eigen::Vector2d post =vertices[i+2];
                Eigen::Vector2d vec1 = end - start;
                Eigen::Vector2d vec2 = post - end;

                vec1.normalize();
                vec2.normalize();

                double scalar = vec1.dot(vec2) < 0 ? 0.0 : vec1.dot(vec2);

                acc_2_coeff = (1 - scalar);

                acceleration_time_2 = acc_2_coeff * ((v_max / a_max) + (a_max / j_max));

                jerk_time_2 = acc_2_coeff * (2 * (a_max / j_max));
            }

            if (acceleration_time_1 > sqrt(distance / a_max)) {
                acceleration_time_1 = sqrt(distance / a_max);
            }

            if (jerk_time_1 > sqrt(v_max / j_max)) {
                jerk_time_1 = sqrt(v_max / j_max);
            }

            if (acceleration_time_2 > sqrt(distance / a_max)) {
                acceleration_time_2 = sqrt(distance / a_max);
            }

            if (jerk_time_2 > sqrt(v_max / j_max)) {
                jerk_time_2 = sqrt(v_max / j_max);
            }

            double max_velocity_time;

            if (((distance - ((v_max * v_max) / a_max)) / v_max) < 0) {
                max_velocity_time = ((distance) / v_max);
            } else {
                max_velocity_time = ((distance - ((v_max * v_max) / a_max)) / v_max);
            }

            /* double t = max_velocity_time + acceleration_time_1 + acceleration_time_2 + jerk_time_1 + jerk_time_2; */
            double t = max_velocity_time + acceleration_time_1 + acceleration_time_2;

            if (t < 0.01) {
                t = 0.01;
            }

            segment_times.push_back(t);
        }
        return segment_times;
    }

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
    }

    void generate_cost_matrix_straightline(Vector2d coord_0, vector<vector<Vector2d>> lines) {
        graphSize = 2 + lines.size()*2;
        cost_matrix = vector<vector<double>>(graphSize,vector<double>(graphSize));
        // fill non-traversable edges with, well ... anything, except float inf, that breaks cplex
        // auto inf = std::numeric_limits<float>::infinity();
        auto inf = 0;
        // total number of nodes is 1 for start node and n*l for n nodes in l lines
        // start node 0 and end node 0
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
                vector<Vector2d> vec5 {lines[l][0], lines[l][1], lines[k][0], lines[k][1]}; // 3412
                vector<Vector2d> vec6 {lines[l][0], lines[l][1], lines[k][1], lines[k][0]}; // 3421
                vector<Vector2d> vec7 {lines[l][1], lines[l][0], lines[k][0], lines[k][1]}; // 4312
                vector<Vector2d> vec8 {lines[l][1], lines[l][0], lines[k][1], lines[k][0]}; // 4321
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
