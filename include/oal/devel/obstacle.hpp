#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug

#include "oal/devel/data_structs.hpp"

namespace oal{

class Obstacle{
            
    std::string id;
    std::string obsClass;
    Pose initial_pose;
    Eigen::Vector2d velocity;
    BoundingBoxData bb_data;

    Eigen::Vector2d obs_P_vehicle;

    std::vector<Vx> vxs; 
    
    void SetLocalVxs();
    
public: 

    Obstacle(std::string id, std::string obsClass, Pose initial_pose, Eigen::Vector2d velocity, BoundingBoxData bb_data, Eigen::Vector2d obs_P_vehicle)
    : id(id), obsClass(obsClass), initial_pose(initial_pose), velocity(velocity), bb_data(bb_data), obs_P_vehicle(obs_P_vehicle) {
        SetLocalVxs();
    }

    auto Id() const -> const std::string& { return id; }
    auto ObsClass() const -> const std::string& { return obsClass; }
    auto InitialPose() const -> const Pose& { return initial_pose; }
    auto Velocity() const -> const Eigen::Vector2d& { return velocity; }



    // All of them
    std::vector<Vx> GetVxs(TimeDouble time, bool compensate_localization_error = false) const;

    Eigen::Vector2d GetPosition(TimeDouble time) const;

    void Print() const {
        std::cerr << "Obstacle: " << std::endl;
        std::cerr << "  - ID: " << id << "\n";
        std::cerr << "  - Class: " << obsClass << "\n";
        std::cerr << "  - Initial Pose: (" << initial_pose.Position().x() << ", " << initial_pose.Position().y() << ") "<<initial_pose.Heading()/M_PI*180<<"°\n";
        std::cerr << "  - Velocity: (" << velocity.x() << ", " << velocity.y() << ")\n";
        std::cerr << "  - Vehicle Position in Obs Frame: (" << obs_P_vehicle.x() << ", " << obs_P_vehicle.y() << ")\n";
        std::cerr << "  - Bounding Box Data: \n";
        std::cerr << "      - Dimensions: " << bb_data.dim_x << " x " << bb_data.dim_y << "\n";
        std::cerr << "      - Min Distance From Obstacle: " << bb_data.minDistFromObs << "\n";
        std::cerr << "      - Reduction While Checking Path: " << bb_data.reductionWhileCheckingPath << "\n";
        std::cerr << "      - Safe Max Gap: " << bb_data.safeMaxGap << "\n";
        std::cerr << "      - Look Ahead Safety Span: " << bb_data.lookAheadSafetySpan << " seconds\n";
        std::cerr << "      - Max Size: (Bow: " << bb_data.max_x_bow << ", Stern: " << bb_data.max_x_stern
                << ", Starboard: " << bb_data.max_y_starboard << ", Port: " << bb_data.max_y_port << ")\n";
        std::cerr << "      - Safety Size: (Bow: " << bb_data.safety_x_bow << ", Stern: " << bb_data.safety_x_stern
                << ", Starboard: " << bb_data.safety_y_starboard << ", Port: " << bb_data.safety_y_port << ")\n";
        
        std::cerr << "  - Vxs: \n";
        for (const auto& vx : vxs) {
            std::cerr << "      - " <<static_cast<VxId>(vx.first)<< ": "<<vx.second.transpose()<<"\n";
        }
        std::cerr << "\n";
    }


};

}
#endif //OAL_OBSTACLE_HPP
