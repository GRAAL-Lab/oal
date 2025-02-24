#ifndef ASTAR_NODE_HPP
#define ASTAR_NODE_HPP

#include <memory>
#include <fstream>

#include "oal/devel/data_structs.hpp"
#include "oal/devel/obstacle.hpp"
#include "oal/devel/geometric_utilities.hpp"

namespace oal {



class AStarNode {

    std::shared_ptr<AStarNode> parent_ = nullptr;

    double costToReach = -1; //cost to reach the Node
    double costToGoal = -1; //estimated cost to reach Goal

public:

    EncounterData data;

    auto Parent() const -> const std::shared_ptr<AStarNode>& { return parent_; }

    AStarNode(EncounterData data,  std::shared_ptr<AStarNode> parent = nullptr);

    void SetCosts(VehicleData vh_data, Eigen::Vector2d goal);

    double GetTotalCost() const { return costToReach + costToGoal; }

    void GetExitVxs(std::vector<Vx> &allowedVxs) const;
    // void FindExitVxs(std::vector<vx_id> &allowedVxs) const;

    void Print(std::string id = "") const {
        if(parent_!= nullptr) {
            std::cerr << "Node"<<id<<": " << std::endl;
        }else{
            std::cerr << "(start) Node"<<id<<": " << std::endl;
        }
        std::cerr << "  - Time: " << data.time.count() << "\n  - Pos: " << data.position.transpose()<<"\n";
        std::cerr << "  - Heading: " << data.heading<<"\n";
        if(parent_!= nullptr) std::cerr << "  - Reaching speed: " << data.approachingSpeed <<"\n";
        std::cerr << "  - Costs: \n";
        std::cerr << "      - Reach: "<<costToReach<<"\n";
        std::cerr << "      - 2Goal: "<<costToGoal<<"\n";
        std::cerr << "      - Total: "<<GetTotalCost()<<"\n";       
        if (data.obs_ptr != nullptr) {
            std::cerr << "  - Obs: " << data.obs_ptr->Id() << "/" << (VxId) data.vx <<"\n";
        }
    }

    void Log(std::ofstream& logFile, std::string id = "") const{
        logFile << "---" << std::endl;
        logFile << "Trace_"<<id<<std::endl;
        const AStarNode* current = this;
        int count = 1;
        while(current != nullptr){
            logFile <<      "_Node_"<<count<<
                            "_Position_"<<current->data.position.x()<<"_"<<current->data.position.y()<<
                            "_Heading_"<<current->data.heading<<
                            "_Time_"<<current->data.time.count()<<
                            "_Speed_"<<current->data.approachingSpeed;
            if(current->data.obs_ptr != nullptr)
                logFile<<   "_Obstacle_"<<current->data.obs_ptr->Id()<<"_Vx_"<<current->data.vx;
            logFile<< std::endl;
            current = current->parent_.get();  
            count++;          
        }
        



    //         // Describes the interception of the ASV with an obstacle vertex
    // Eigen::Vector2d position;
    // double heading; //this is the heading when on the node (used for heading change computation)
    // std::chrono::duration<double> time =  std::chrono::seconds(0);
    // double approachingSpeed{};
    // std::shared_ptr<Obstacle> obs_ptr = nullptr;
    // VxId vx = NA;

        

    }

};

}
#endif //ASTAR_NODE_HPP