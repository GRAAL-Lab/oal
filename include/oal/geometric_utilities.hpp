#ifndef GEOMETRIC_UTILITIES_HPP
#define GEOMETRIC_UTILITIES_HPP

#include <eigen3/Eigen/Eigen>
#include <iostream>

#define ZERO_NUMERICAL 1e-5

double GetBearing(const Eigen::Vector2d& direction, double observer_heading);

void FindInterceptPoints(const Eigen::Vector2d& p1,
    double p1_speed,
    const Eigen::Vector2d& p2,
    const Eigen::Vector2d& p2_vel,
    std::vector<Eigen::Vector3d>& points);

bool FindLinePlaneIntersection(const Eigen::Vector3d& p1,
    const Eigen::Vector3d& p2,
    const Eigen::Vector3d& planePoint,
    const Eigen::Vector3d& planeNormal,
    Eigen::Vector3d& intersection);

// Function to check if a point is inside a quadrilateral using the winding number algorithm
bool IsPointInQuadrilateral(const Eigen::Vector2d& point,
    const std::vector<Eigen::Vector2d>& vertices);

std::vector<bool> ComputeVisibleVertices(const Eigen::Vector2d& observer,
    const Eigen::Vector2d& target,
    std::vector<Eigen::Vector2d>& vertexes);

#endif // GEOMETRIC_UTILITIES_HPP