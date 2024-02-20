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





#endif
