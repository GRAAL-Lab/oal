#ifndef OAL_PATH_HPP
#define OAL_PATH_HPP

#include <eigen3/Eigen/Eigen>
//#include <utility>
//#include <memory>
//#include <iostream> //debug
#include <stack>
//#include <iomanip> //tables
//#include "oal/data_structs/obstacle.hpp"
#include "oal/node.hpp"

struct Metrics {
    //double heading = 0; //used only for first node to compute heading changes
    double maxHeadingChange = 0;  // this should not start from zero if initial OS heading is known
    double totHeadingChange = 0;  // this should not start from zero if initial OS heading is known
    double totDistance = 0;
    double estimatedTime = 0;
};

struct Path {
    std::stack<Node> waypoints;
    Metrics metrics;
    std::vector<std::string> overtakingObsList;
    std::vector<std::string> overtakenObsList;

    bool debug_flag = false;

    [[nodiscard]] size_t size() const {
        return waypoints.size();
    }

    [[nodiscard]] bool empty() const {
        return waypoints.empty();
    }

    Node top() {
        return waypoints.top();
    }

    void pop() {
        // When reached a node, delete it from Path and set the obstacle as overtaken to avoid future crossing
        Node nd = waypoints.top();
        if (nd.obs_ptr != nullptr) {
            auto pos = std::find(overtakingObsList.begin(), overtakingObsList.end(), nd.obs_ptr->id);
            if (pos != overtakingObsList.end()) overtakenObsList.push_back(nd.obs_ptr->id);
        }
        waypoints.pop();
    }

    void UpdateMetrics(const Eigen::Vector2d &vh_pos, double starting_heading, double rot_speed) {
        // DO NOT REMOVE STARTING WAYPOINT FROM this PATH
        // TODO if I only use start heading here and not for path computing, maybe unnecessary in node def
        Path temp = *this;
        temp.pop(); // removing path starting point
        Eigen::Vector2d vh_to_first = temp.top().position - vh_pos;
        double dist = vh_to_first.norm();

        Eigen::Rotation2D<double> rotation(starting_heading);
        Eigen::Vector2d headVector = rotation * Eigen::Vector2d::UnitX();
        double course_change = abs(
                std::atan2(headVector.y(), headVector.x()) - std::atan2(vh_to_first.y(), vh_to_first.x()));
        if (course_change > M_PI) course_change = abs(course_change - 2 * M_PI);

        double max_course_change = course_change;
        double est_time = dist / temp.top().speed_to_it;
        if (rot_speed != 0) {
            est_time += course_change / rot_speed;
        }
        temp.pop();
        while (!temp.empty()) {
            Node next = temp.top();
            dist += next.distFromParent;
            course_change += next.courseChangeFromParent;
            max_course_change = std::max(max_course_change, next.courseChangeFromParent);
            est_time = next.distFromParent / temp.top().speed_to_it;
            if (rot_speed != 0) {
                est_time += next.courseChangeFromParent / rot_speed;
            }
            temp.pop();
        }
        metrics.totDistance = dist;
        metrics.maxHeadingChange = max_course_change;
        metrics.totHeadingChange = course_change;
        metrics.estimatedTime = est_time;
    }

    std::string print(bool local_wp = false) const {
        std::ostringstream oss;

        oss << "Path:\n"
            << " Length: " << metrics.totDistance << " meters\n"
            << " Total course change: " << metrics.totHeadingChange << " radians\n"
            << " Max course change: " << metrics.maxHeadingChange << " radians\n";

        if (local_wp) {
            Path temp = *this;
            oss << " Waypoint list:\n";

            while (!temp.empty()) {
                auto node = temp.top();
                oss << "   - time: " << node.time << "  Pos: " << node.position.x() << " " << node.position.y();

                if (node.obs_ptr != nullptr) {
                    switch (node.vx) {
                        case 0:
                            oss << "   Obs: " << node.obs_ptr->id << "/FR reaching speed: " << node.speed_to_it;
                            break;
                        case 1:
                            oss << "   Obs: " << node.obs_ptr->id << "/FL reaching speed: " << node.speed_to_it;
                            break;
                        case 2:
                            oss << "   Obs: " << node.obs_ptr->id << "/RR reaching speed: " << node.speed_to_it;
                            break;
                        case 3:
                            oss << "   Obs: " << node.obs_ptr->id << "/RL reaching speed: " << node.speed_to_it;
                            break;
                        case 5:
                            oss << "   Obs: " << node.obs_ptr->id << "/W reaching speed: " << node.speed_to_it;
                            break;
                        default:
                            oss << " <Obs has undefined vx ?!?!>\n";
                    }
                }
                oss << "\n";
                temp.pop();
            }
        }

        return oss.str();
    }


};

#endif //OAL_PATH_HPP
