#ifndef OAL_OBSTACLE_HPP
#define OAL_OBSTACLE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug


#include "oal/vertex.hpp"

#include "oal/data_structs.hpp"



class Obstacle {
//private:
public:
    // Defining attributes
    std::string id;
    std::string obsClass;
    Pose pose;
    Velocity velocity;
    BoundingBoxData bb_data;
    bool higher_priority = false;

    std::vector<Vertex> vxs; // local position (wrt obs)

    bool uncertainty = false;

    void ComputeLocalVxsBasedOnVhDist(const Eigen::Vector2d &bodyObs_vhPos, bool compensate_localization_error);

    // Set bb size according to own ship distance
    // void SetSize(double dist_x, double dist_y, double theta, double &bb_dim_x_bow, double &bb_dim_x_stern,
    //              double &bb_dim_y_starboard, double &bb_dim_y_port) const;

    // Project vxs position in world frame
    void FindAbsVxs(double time, std::vector<Vertex> &vxs_abs);

    // Project absolute position in obstacle frame (depends on time-instant)
    Eigen::Vector2d GetProjectionInLocalFrame(TPoint &time_point);

    // Check if point is in obs bb (depends on time-instant)
    bool IsInBB(TPoint &time_point);

    std::string plotStuff(double time);

    void print() const {
        //std::cout<<id<<std::endl<< position.x()<<" "<<position.y()<<std::endl<<head<<std::endl<<speed<<std::endl;
        std::cout << "obstacles.push_back(Obstacle(\"" << id << "\", {" << pose.position.x() << ", " << pose.position.y() << "}, "
                  << pose.heading << ", " << velocity.speed << ", " << velocity.angle << ", bb_dimension));" << std::endl;
    }

//public:
    Obstacle() = default;

    Obstacle(std::string name, Eigen::Vector2d position, double heading, double speed, double vel_dir, BoundingBoxData bb,
             bool high_priority = false)
            : id(std::move(name)),
              bb_data(bb), higher_priority(high_priority) {

                pose.position = position;
                pose.heading = heading;
                velocity.speed = speed;
                velocity.angle = vel_dir;
              }

    // Compute local position of bb vxs
    //void FindLocalVxs(const Eigen::Vector2d &vhPos);

};

#endif //OAL_OBSTACLE_HPP
