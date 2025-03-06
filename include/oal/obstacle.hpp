#ifndef OBSTACLE_HPP
#define OBSTACLE_HPP

#include <eigen3/Eigen/Eigen>
#include <utility>
#include <memory>
#include <iostream> //debug
#include <fstream>
#include <iomanip>  

#include "oal/data_structs.hpp"

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

    Obstacle(std::string id, std::string obsClass, Pose initial_pose, Eigen::Vector2d velocity, BoundingBoxData bb_data, Eigen::Vector2d vehicle)
    : id(id), obsClass(obsClass), initial_pose(initial_pose), velocity(velocity), bb_data(bb_data){
        obs_P_vehicle = initial_pose.FromWorld2ThisFrame(vehicle);
        SetLocalVxs();
    }
    
    Obstacle(const ObsPtr& other, TimeDouble timeDiff, Eigen::Vector2d vehicle)
    : id(other->Id()), obsClass(other->ObsClass()), velocity(other->Velocity()), bb_data(other->bb_data)
    {
        // Same obstacle but after/before timeDiff, with different vehicle pos
        initial_pose = Pose(other->GetPosition(timeDiff), other->InitialPose().Heading());
        obs_P_vehicle = initial_pose.FromWorld2ThisFrame(vehicle);
        SetLocalVxs();
    }

    auto Id() const -> const std::string& { return id; }
    auto ObsClass() const -> const std::string& { return obsClass; }
    auto InitialPose() const -> const Pose& { return initial_pose; }
    auto Velocity() const -> const Eigen::Vector2d& { return velocity; }
    auto BBData() const -> const BoundingBoxData& {return bb_data; }



    // All of them
    std::vector<Vx> GetVxs(TimeDouble time, bool compensate_localization_error = false) const;

    Eigen::Vector2d GetPosition(TimeDouble time) const;

    void Print(TimeDouble time = std::chrono::duration<double>(0)) const {
        std::cerr << "Obstacle: " << std::endl;
        std::cerr << "  - ID: " << id << "\n";
        std::cerr << "  - Class: " << obsClass << "\n";
        std::cerr << "  - Initial Pose: (" << initial_pose.Position().x() << ", " 
                                        << initial_pose.Position().y() << ") " 
                                        << initial_pose.Heading() / M_PI * 180 << "°\n";
        std::cerr << "  - Velocity: (" << velocity.x() << ", " << velocity.y() << ")\n";
        std::cerr << "  - Vehicle Position in Obs Frame: (" << obs_P_vehicle.x() 
                << ", " << obs_P_vehicle.y() << ")\n";
        
        std::cerr << "  - Bounding Box Data: \n";
        std::cerr << "      - Dimensions: " << bb_data.dim_x << " x " << bb_data.dim_y << "\n";
        std::cerr << "      - Min Distance From Obstacle: " << bb_data.minDistFromObs << "\n";
        std::cerr << "      - Reduction While Checking Path: " << bb_data.reductionWhileCheckingPath << "\n";
        std::cerr << "      - Safe Max Gap: " << bb_data.safeMaxGap << "\n";
        std::cerr << "      - Look Ahead Safety Span: " << bb_data.lookAheadSafetySpan << " seconds\n";
        std::cerr << "      - Max Size: (Bow: " << bb_data.max_x_bow 
                << ", Stern: " << bb_data.max_x_stern
                << ", Starboard: " << bb_data.max_y_starboard 
                << ", Port: " << bb_data.max_y_port << ")\n";
        std::cerr << "      - Safety Size: (Bow: " << bb_data.safety_x_bow 
                << ", Stern: " << bb_data.safety_x_stern
                << ", Starboard: " << bb_data.safety_y_starboard 
                << ", Port: " << bb_data.safety_y_port << ")\n";

        std::cerr << "  - Vxs (wrt obs pose / wrt world at time " << time.count() << "): \n";
        
        auto abs_vxs = GetVxs(time);
        for (size_t i = 0; i < 4; i++) {
            auto local = vxs.at(i);
            auto abs = abs_vxs.at(i);
            
            // Formatting numbers for better alignment
            std::cerr << std::fixed << std::setprecision(1); // Set decimal precision

            std::cerr << "      - " << std::setw(2) << static_cast<VxId>(local.first) << ":  " 
                    << std::setw(6) << local.second.x() << "  " 
                    << std::setw(6) << local.second.y() << "  /  " 
                    << std::setw(6) << abs.second.x() << "  " 
                    << std::setw(6) << abs.second.y() << "\n";
        }
        
        std::cerr << "\n";
    }  


    void Log(std::ofstream& logFile) const{
        logFile << "---" << std::endl;
        logFile << "Obstacle_"<<id<<
        "_Position_"<<initial_pose.Position().x()<<"_"<<initial_pose.Position().y()<<
        "_Heading_"<<initial_pose.Heading()<<
        "_Velocity_"<<velocity.x()<<"_"<<velocity.y();

        auto abs_vxs = GetVxs(TimeDouble(0));
        for(size_t i = 0;i<4;i++){
            logFile<<"_Vx"<<abs_vxs[i].first<<"_"<<abs_vxs[i].second.x()<<"_"<<abs_vxs[i].second.y();
        }
        logFile<< std::endl;
    }


};

}
#endif //OAL_OBSTACLE_HPP
