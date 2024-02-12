#ifndef OAL_OBSTACLE_HPP
#define OAL_OBSTACLE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug


#include "oal/data_structs/vertex.hpp"
#include "oal/data_structs/tpoint.hpp"
#include "oal/data_structs/bounding_box.hpp"

#define BB_GAP 1.5

class Obstacle {
//private:
public:
    std::string id;
    Eigen::Vector2d position;  //absolute position at time 0
    double head{};
    double vel_dir{};
    double speed{};
    bb_data bb;
    std::vector<Vertex> vxs; // local position (wrt obs)

    bool uncertainty = false;
    bool higher_priority = false;

    // Set bb size according to own ship distance
    void SetSize(double dist_x, double dist_y, double theta, double &bb_dim_x_bow, double &bb_dim_x_stern,
                 double &bb_dim_y_starboard, double &bb_dim_y_port) const;

    // Compute local position of bb vxs
    void FindLocalVxs(const Eigen::Vector2d &vhPos);

    // Project vxs position in world frame
    void FindAbsVxs(double time, std::vector<Vertex> &vxs_abs);

    // Project absolute position in obstacle frame (depends on time-instant)
    Eigen::Vector2d GetProjectionInLocalFrame(TPoint &time_point);

    // Check if point is in obs bb (depends on time-instant)
    bool IsInBB(TPoint &time_point);

    std::string plotStuff(double time);

    void print() const {
      //std::cout<<id<<std::endl<< position.x()<<" "<<position.y()<<std::endl<<head<<std::endl<<speed<<std::endl;
      std::cout<<"obstacles.push_back(Obstacle(\""<<id<<"\", {"<<position.x()<<", "<<position.y()<<"}, "<< head << ", "<< speed<<", "<<vel_dir<<", bb_dimension));"<<std::endl;
    }

//public:
    Obstacle() = default;

    Obstacle(std::string name, Eigen::Vector2d position, double heading, double speed, double vel_dir, bb_data bb,
             bool high_priority = false)
            : id(std::move(name)), position(std::move(position)), head(heading), speed(speed), vel_dir(vel_dir),
              bb(bb), higher_priority(high_priority) {}

};

#endif //OAL_OBSTACLE_HPP
