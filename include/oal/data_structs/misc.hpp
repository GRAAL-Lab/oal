#ifndef DATA_STRUCTS_HPP
#define DATA_STRUCTS_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include <stack>
#include <iomanip> //tables
#include "oal/data_structs/obstacle.hpp"
#include "oal/data_structs/node.hpp"

typedef std::shared_ptr<Obstacle> obs_ptr;

struct VehicleInfo {
    Eigen::Vector2d position;
    std::vector<double> velocities;
    double heading;
    double rot_speed = 0;
};

struct ObstaclesInfo {
    std::vector<std::shared_ptr<Obstacle>> obstacles;
};

struct Metrics{
    //double heading = 0; //used only for first node to compute heading changes
    double maxHeadingChange = 0;  // this should not start from zero if initial OS heading is known
    double totHeadingChange = 0;  // this should not start from zero if initial OS heading is known
    double totDistance = 0;
};
struct Path {
    bool debug_flag = false;
    std::stack<Node> waypoints;
    Metrics metrics;
    std::vector<std::string> overtakingObsList;
    std::vector<std::string> overtakenObsList;

    [[nodiscard]] size_t size() const{
      return waypoints.size();
    }

    [[nodiscard]] bool empty() const{
      return waypoints.empty();
    }

    Node top(){
      return waypoints.top();
    }

    void pop(){
      // When reached a node, delete it from Path and set the obstacle as overtaken to avoid future crossing
      Node nd = waypoints.top();
      if(nd.obs_ptr != nullptr){
        auto pos = std::find(overtakingObsList.begin(), overtakingObsList.end(), nd.obs_ptr->id);
        if (pos != overtakingObsList.end())  overtakenObsList.push_back(nd.obs_ptr->id);
      }
      waypoints.pop();
    }

    void UpdateMetrics(const Eigen::Vector2d& vh_pos, double starting_heading){
      // DO NOT REMOVE STARTING WAYPOINT FROM this PATH
      // TODO if I only use start heading here and not for path computing, maybe unnecessary in node def
      Path temp = *this;
      temp.pop(); // removing path starting point
      Eigen::Vector2d vh_to_first = temp.top().position - vh_pos;
      double dist = vh_to_first.norm();
      // TODO check starting heading is proper
      double course_change = abs(starting_heading- std::acos(vh_to_first.normalized().dot(Eigen::Vector2d(1,0))));
      double max_course_change = course_change;
      temp.pop();
      while(!temp.empty()){
        Node next = temp.top();
        dist += next.distFromParent;
        course_change += next.courseChangeFromParent;
        max_course_change = std::max(max_course_change, next.courseChangeFromParent);
        temp.pop();
      }
      metrics.totDistance = dist;
      metrics.maxHeadingChange = max_course_change;
      metrics.totHeadingChange = course_change;
    }

    void print(bool local_wp = false){
      std::cout<<"Path:\n"
               <<" Length: "<<metrics.totDistance<<" meters\n"
               <<" Total course change: "<<metrics.totHeadingChange<<" radians\n"
               <<" Max course change: "<<metrics.maxHeadingChange<<" radians"<<std::endl;

      if(local_wp){
        Path temp = *this;
        std::cout<<" Waypoint list:"<<std::endl;

        while(!temp.empty()){
          auto node = temp.top();
          std::cout << "   - time: " << temp.top().time << "  Pos: " << node.position.x() << " " << node.position.y();
          //std::cout << temp.top().time << std::setw(5) << node.position.x() << std::setw(5) << node.position.y() << std::setw(5);
          if(node.obs_ptr != nullptr){
            switch (node.vx){
              case 0:
                std::cout << "   Obs: " << node.obs_ptr->id << "/FR reaching speed: " << node.speed_to_it;
                //std::cout << node.obs_ptr->id << std::setw(4) << "FR" << std::setw(4) << node.speed_to_it;
                break;
              case 1:
                std::cout << "   Obs: " << node.obs_ptr->id << "/FL reaching speed: " << node.speed_to_it;
                //std::cout << node.obs_ptr->id << std::setw(4) << "FL" << std::setw(4) << node.speed_to_it;
                break;
              case 2:
                std::cout << "   Obs: " << node.obs_ptr->id << "/RR reaching speed: " << node.speed_to_it;
                //std::cout << node.obs_ptr->id << std::setw(4) << "RR" << std::setw(4) << node.speed_to_it;
                break;
              case 3:
                std::cout << "   Obs: " << node.obs_ptr->id << "/RL reaching speed: " << node.speed_to_it;
                //std::cout << node.obs_ptr->id << std::setw(4) << "RL" << std::setw(4) << node.speed_to_it;
                break;
              case 5:
                std::cout << "   Obs: " << node.obs_ptr->id << "/W reaching speed: " << node.speed_to_it;
                //std::cout << node.obs_ptr->id << std::setw(4) << "within" << std::setw(4) << node.speed_to_it;
                break;
              default:
                std::cout << " <Obs has undefined vx ?!?!> "<<std::endl;
            }
          }
          std::cout << std::endl;
          temp.pop();
        }
      }
    }

};


#endif
