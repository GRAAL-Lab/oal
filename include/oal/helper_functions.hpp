#ifndef HELPER_HPP
#define HELPER_HPP

#include "oal/misc.hpp"
#include "oal/obstacle.hpp"
#include <set>
#include <chrono>

template<typename T>
Eigen::Vector2d ComputePosition(T &element, std::chrono::seconds time) {
    Eigen::Vector2d shift(element.velocity.speed * time.count() * cos(element.velocity.angle), element.velocity.speed *time.count() * sin(element.velocity.angle));
    return element.pose.position + shift;
}

template<typename T>
Eigen::Vector3d Get3dPos(T &element) {
    return {element.position.x(), element.position.y(), 0};
}

Eigen::Vector2d GetProjectionInObsFrame(const Eigen::Vector2d &point, const Obstacle &obs, std::chrono::seconds time);

// Wrt TS, from which angle OS approach (given OS direction and TS heading)
double GetBearing(Eigen::Vector2d direction, double obs_heading);

#endif