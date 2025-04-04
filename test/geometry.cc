
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <vector>

#include "oal/geometric_utilities.hpp"

bool Test_LinePlaneIntersection()
{
    Eigen::Vector3d p1(0, 0, 0); // Line start
    Eigen::Vector3d p2(7, 1.5, 0.71589105316381763); // Line end
    Eigen::Vector3d planePoint(77.0, 1.5, 0.0); // A point on the plane
    Eigen::Vector3d planeNormal(0.0, 14.0, 0.0); // Plane normal

    Eigen::Vector3d intersection;
    bool test = FindLinePlaneIntersection(p1, p2, planePoint, planeNormal.normalized(), intersection);

    if (test)
        return false;
    // Eigen::Vector3d p1(0, 0, 0);      // Line start
    // Eigen::Vector3d p2(10, 0, 1);      // Line end
    // Eigen::Vector3d planePoint(5, 0, 0); // A point on the plane
    // Eigen::Vector3d planeNormal(-1, 0, 0.1); // Plane normal

    // Eigen::Vector3d intersection;
    // return FindLinePlaneIntersection(p1, p2, planePoint, planeNormal.normalized(), intersection);
    return true;
}

bool Test_InBB()
{
    std::vector<Eigen::Vector2d> vertexes = {
        { 0, 0 }, { 4, 0 }, { 4, 3 }, { 0, 3 }
    };

    // Test point
    Eigen::Vector2d inside(2, 1); // Inside the quadrilateral
    Eigen::Vector2d putside(5, 1); // Outside the quadrilateral

    if (IsPointInQuadrilateral(inside, vertexes) && !IsPointInQuadrilateral(putside, vertexes)) {
        return true;
    }
    return false;
}

bool Test_ObserverAligned()
{

    Eigen::Vector2d observer(-10, 0);
    Eigen::Vector2d target(0, 0);

    std::vector<Eigen::Vector2d> vertexes = {
        { -1, 1 }, { 1, 1 }, { 1, -1 }, { -1, -1 }
    };

    std::vector<bool> visibility = ComputeVisibleVertices(observer, target, vertexes);

    if (visibility == std::vector<bool> { 1, 0, 0, 1 }) {
        return true;
    }
    return false;
}

bool Test_ObserverFarAway()
{

    Eigen::Vector2d observer(-10, 10);
    Eigen::Vector2d target(0, 0);

    std::vector<Eigen::Vector2d> vertexes = {
        { -1, 1 }, { 1, 1 }, { 1, -1 }, { -1, -1 }
    };

    std::vector<bool> visibility = ComputeVisibleVertices(observer, target, vertexes);

    if (visibility == std::vector<bool> { 1, 1, 0, 1 }) {
        return true;
    }
    return false;
}

// Test Case 1: No intercept
bool Test_NoIntercept()
{
    Eigen::Vector2d vehicle_position(0, 0); // Vehicle starting position
    double vehicle_speed = 1.0; // Vehicle speed
    Eigen::Vector2d obstacle_position(10, 10); // Obstacle position
    Eigen::Vector2d obstacle_velocity(1, 1); // Obstacle velocity

    std::vector<Eigen::Vector3d> points;
    FindInterceptPoints(vehicle_position, vehicle_speed, obstacle_position, obstacle_velocity, points);

    for (const auto& point : points) {
        std::cout << "Intercept point: " << point.transpose() << std::endl;
    }

    // If no intercept, points should be empty
    return points.empty();
}

// Test Case 2: Single intercept point
bool Test_SingleIntercept()
{
    Eigen::Vector2d vehicle_position(0, 0); // Vehicle starting position
    double vehicle_speed = 5.0; // Vehicle speed
    Eigen::Vector2d obstacle_position(3, 3); // Obstacle position
    Eigen::Vector2d obstacle_velocity(1, 1); // Obstacle velocity

    std::vector<Eigen::Vector3d> points;
    FindInterceptPoints(vehicle_position, vehicle_speed, obstacle_position, obstacle_velocity, points);

    // If there is one intercept, we expect points to contain exactly one element
    return points.size() == 1;
}

// Test Case 3: Two intercept points
bool Test_TwoIntercepts()
{
    Eigen::Vector2d vehicle_position(0, 0); // Vehicle starting position
    double vehicle_speed = 8.0; // Vehicle speed
    Eigen::Vector2d obstacle_position(50, 50); // Obstacle position
    Eigen::Vector2d obstacle_velocity(-10, 0); // Obstacle velocity

    std::vector<Eigen::Vector3d> points;
    FindInterceptPoints(vehicle_position, vehicle_speed, obstacle_position, obstacle_velocity, points);

    // If there are two intercept points, we expect points to contain exactly two elements
    return points.size() == 2;
}

// Test Case 4: Vehicle intercepts directly in line with obstacle (single point)
bool Test_DirectIntercept()
{
    Eigen::Vector2d vehicle_position(0, 0); // Vehicle starting position
    double vehicle_speed = 5.0; // Vehicle speed
    Eigen::Vector2d obstacle_position(10, 0); // Obstacle position (directly in line with vehicle)
    Eigen::Vector2d obstacle_velocity(0, 0); // Obstacle is stationary

    std::vector<Eigen::Vector3d> points;
    FindInterceptPoints(vehicle_position, vehicle_speed, obstacle_position, obstacle_velocity, points);

    // If there's a direct intercept, points should contain exactly one element
    return points.size() == 1;
}

int main()
{

    bool test_results = true;

    test_results &= Test_NoIntercept();
    test_results &= Test_SingleIntercept();
    test_results &= Test_TwoIntercepts();
    test_results &= Test_DirectIntercept();
    test_results &= Test_LinePlaneIntersection();

    if (Test_ObserverFarAway && Test_ObserverAligned && Test_InBB && Test_LinePlaneIntersection() && test_results) {
        std::cout << "- GEOMETRY: OK" << std::endl;
    } else {
        std::cout << "- GEOMETRY: FAIL" << std::endl;
    }
    return 0;
}
