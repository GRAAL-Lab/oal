#include "oal/devel/geometric_utilities.hpp"


/**
 * @brief Computes the relative bearing between a direction vector and an observer's heading.
 * 
 * This function calculates the angle difference between a given direction vector 
 * and an observer's heading, returning a signed angle in the range [-π, π].
 * 
 * @param direction The 2D direction vector.
 * @param observer_heading The heading angle of the observer in radians.
 * @return The relative bearing in radians, constrained to [-π, π].
 * @throws std::invalid_argument If the direction vector is (0,0).
 */
double GetBearing(const Eigen::Vector2d& direction, double observer_heading) {
    if (direction.isZero(1e-10)) {  // Handle zero vector case with a small tolerance
        throw std::invalid_argument("Direction vector passed to GetBearing cannot be (0,0)");
    }
    
    double direction_angle = std::atan2(direction.y(), direction.x());
    double theta = M_PI + observer_heading - direction_angle;  // Interception angle
    double result = std::remainder(theta, 2 * M_PI);

    return result;
}


void FindInterceptPoints(const Eigen::Vector2d &p1, double p1_speed, const Eigen::Vector2d& p2, const Eigen::Vector2d& p2_vel, std::vector<Eigen::Vector3d>& points) {
    std::vector<double> t_instants;
    Eigen::Vector3d p2_velocity(p2_vel.x(), p2_vel.y(), 1);
    auto distance = p2 - p1;
    if(p2_vel.norm() < ZERO) { 
        t_instants.push_back(distance.norm() / p1_speed);
    } else {
        double det = pow(p1_speed, 2) + 1;
        double c2 = 1 - (pow(p2_velocity.x(), 2) + pow(p2_velocity.y(), 2) + 1) / det;
        double c1 = -(p2_velocity.x() * distance.x() + p2_velocity.y() * distance.y()) / det;
        double c0 = -(pow(distance.x(), 2) + pow(distance.y(), 2)) / det;
        double delta = pow(c1, 2) - c0 * c2;
        if (abs(c2) > ZERO && delta >= 0) {
            // 2 points
            double t1 = (-c1 + sqrt(delta)) / c2;
            double t2 = (-c1 - sqrt(delta)) / c2;
            if (t1 > 0) t_instants.push_back(t1);
            if (t2 > 0 && abs(t1 - t2) > ZERO) t_instants.push_back(t2);
        } else {
            if (abs(c2) <= ZERO && abs(c1) > ZERO) {
                // 1 point
                double t = -c0 / (2 * c1);
                if (t > 0) t_instants.push_back(t);
            } else {
                // no points
                return;
            }
        }
    }

    points.clear();
    for(double t: t_instants) {
        Eigen::Vector3d point_3d = Eigen::Vector3d(p2.x(), p2.y(), 0) + p2_velocity * t;
        // TODO check outside if in bb
        points.push_back(point_3d);
    }
    //             std::shared_ptr<std::vector<obs_ptr>> surrounding_obs(new std::vector<obs_ptr>);
    //             bool isInAnyBB = IsInAnyBB(intercept_point, surrounding_obs);
    //             if (isInAnyBB && surrounding_obs->size() == 1 && surrounding_obs->at(0)->id == obstacle.id) {
    //                 // if vx is in its own bb, ignore
    //                 isInAnyBB = false;
    //             }
    //             // Save the point
    //             if (!isInAnyBB) {
    //                 Vertex vx = visible_vxs[i];
    //                 vx.intercept_point = intercept_point;
    //                 vx.intercept_speed = vh_speed;
    //                 reachable_vxs.push_back(vx);
    //             }
    //         }
    //     }
    // }
}

bool FindLinePlaneIntersection(const Eigen::Vector3d &p1, 
                               const Eigen::Vector3d &p2, 
                               const Eigen::Vector3d &planePoint, 
                               const Eigen::Vector3d &planeNormal, 
                               Eigen::Vector3d &intersection) 
{
    //Algorithm from http://paulbourke.net/geometry/pointlineplane/
    Eigen::Vector3d lineDir = p2 - p1;   // Direction of the line
    double denom = planeNormal.dot(lineDir);

    // If denominator is near ZERO, the line is parallel to the plane (no intersection)
    if (std::abs(denom) < ZERO) return false;

    double t = planeNormal.dot(planePoint - p1) / denom;

    // If t is in [0,1], intersection occurs within the segment
    if (t >= 0.0 && t <= 1.0) {
        intersection = p1 + t * lineDir;
        return true;
        // TODO ACTUAL COLLISION WITH:
         // Get vertexes at time t'
//         Eigen::Vector3d vertex1_position = vx1_pos + bb_direction * collision_time;
//         Eigen::Vector3d vertex2_position = vx2_pos + bb_direction * collision_time;
//         // Check if point is inside those two vertexes
//         Eigen::Vector3d P1 = collision_point - vertex1_position;
//         Eigen::Vector3d P2 = collision_point - vertex2_position;
//         if (P1.dot(P2) <= 0) return true;
    }
    
    return false;  // Intersection is outside the segment
}

// Function to check if a point is inside a quadrilateral using the winding number algorithm
bool IsPointInQuadrilateral(const Eigen::Vector2d& point, const std::vector<Eigen::Vector2d>& vertices) {

    double windingNumber = 0.0;

    auto crossProduct = [](const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) {
        return v1.x() * v2.y() - v1.y() * v2.x();
    };
    
    for (int i = 0; i < 4; i++) {
        Eigen::Vector2d edgeStart = vertices[i];
        Eigen::Vector2d edgeEnd = vertices[(i + 1) % 4];

        Eigen::Vector2d edgeVector = edgeEnd - edgeStart;
        Eigen::Vector2d toPointStart = point - edgeStart;
        Eigen::Vector2d toPointEnd = point - edgeEnd;

        double cross = crossProduct(edgeVector, toPointStart);
        double dot = toPointStart.dot(toPointEnd);

        double angle = atan2(cross, dot);
        windingNumber += angle;
    }

    // If the absolute winding number is close to 2π, the point is inside
    return fabs(fabs(windingNumber) - 2 * M_PI) < 1e-5;
}

std::vector<bool> ComputeVisibleVertices(const Eigen::Vector2d& observer, const Eigen::Vector2d& target, std::vector<Eigen::Vector2d>& vertexes) {
    Eigen::Vector2d ref_to_target = target - observer;
    std::vector<double> angles;
    std::vector<double> distances;

    std::vector<bool> visibility(vertexes.size(), false);
    
    for (const auto& vertex : vertexes) {
        Eigen::Vector2d ref_to_vertex = vertex - observer;
        if (ref_to_vertex.norm() <= ZERO) continue;
        
        double cross = ref_to_target.x() * ref_to_vertex.y() - ref_to_target.y() * ref_to_vertex.x();
        double sign = (cross >= 0) ? 1.0 : -1.0;
        double angle = sign * acos(ref_to_target.normalized().dot(ref_to_vertex.normalized()));
        angles.push_back(angle);
        distances.push_back(ref_to_vertex.norm());
    }
    
    auto min_it = std::min_element(angles.begin(), angles.end());
    auto max_it = std::max_element(angles.begin(), angles.end());
    
    if (min_it != angles.end() && max_it != angles.end()) {
        size_t min_idx = std::distance(angles.begin(), min_it);
        size_t max_idx = std::distance(angles.begin(), max_it);

        visibility[min_idx] = true;
        visibility[max_idx] = true;
        
        double min_distance = distances[min_idx];
        double max_distance = distances[max_idx];
        
        for (size_t i = 0; i < vertexes.size(); ++i) {
            if (i == min_idx || i == max_idx) continue;
            if (distances[i] < min_distance && distances[i] < max_distance) {
                visibility[i] = true;
            }
        }
    }

    return visibility;

}