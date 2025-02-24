#ifndef ASTAR_GENERATOR_HPP 
#define ASTAR_GENERATOR_HPP


#include <fstream>
#include <memory>
#include <unordered_map>


#include "oal/devel/node.hpp"
#include "oal/devel/path.hpp"
#include "oal/devel/obstacle.hpp"
#include "oal/devel/geometric_utilities.hpp"
#include "oal/devel/path_evaluator.hpp"

namespace oal{

struct DebugSettings{
    bool printCurrentNode = false;
    bool printPath = false;
    bool printComputedVxs = false;

    bool logNodes = false;
    std::string logNodesPathFile = "WARNING_LOG_NODES_PATH_FILE_NOT_SET.txt";

    bool printNodesStats = false;
};

class Generator
{

    VehicleData vh_data_; // vehicle capabilities, not time dependent
    PruningParams pruning_params_; // exploration settings

    DebugSettings ds_;
    std::ofstream logNodesFile;

    bool GetFirstNodes(const Pose& start_, const Eigen::Vector2d& goal_, const std::vector<ObsPtr>& obstacles_, NodeSet& output, PathReport& response);

    bool IsNodeInSet(const NodeSet& nodes, const NodePtr& node);

    bool IsInBB(TimeDouble time, Eigen::Vector2d point, 
                const std::vector<ObsPtr>& obstacles,
                std::vector<ObsPtr>& surrounding_obs);

    void KeepVisibleVxsFromVehicle(Eigen::Vector2d vehicle, Eigen::Vector2d obstacle, std::vector<Vx>& vxs);

    void PrintNodeStats(const NodeSet& set);

public:

    Generator(VehicleData vh_data, PruningParams pruning_params, DebugSettings ds = {}) 
        : vh_data_(vh_data), pruning_params_(pruning_params), ds_(ds) {

            //Debug settings
            if(ds_.logNodes){
                if (logNodesFile.is_open()) logNodesFile.close();
                logNodesFile.open(ds_.logNodesPathFile, std::ofstream::trunc);
            } 
        }

    //auto Obstacles() -> std::vector<ObsPtr>& { return obstacles_; }
    
    PathReport FindPath(Pose start_, Eigen::Vector2d goal_, std::vector<ObsPtr> obstacles_, AstarPath& path_);

    void CheckPath();

    void GetNodesToGoal(const NodePtr& current, const Eigen::Vector2d& goal_, const std::vector<ObsPtr>& obstacles_, NodeSet& successors){
        // Nodes to goal
        for(const auto& speed : vh_data_.velocities){
            auto reachingTime = std::chrono::duration<double>((goal_ - current->data.position).norm() / speed);
            std::vector<ObsPtr> so;
            if(IsInBB(reachingTime, goal_, obstacles_, so)) continue;
            EncounterData data;
            data.approachingSpeed = speed;
            data.position = goal_;
            data.time = current->data.time + reachingTime;
            data.discoverySource = NodeSearch::DiscoverySource::AIM_TO_GOAL;

            AStarNode goal_node(data, current);
            goal_node.SetCosts(vh_data_, goal_);
            successors.push_back(std::make_shared<AStarNode>(goal_node));
        }
    }

    void GetNodesToObstacles(const NodePtr& current, const std::vector<ObsPtr>& obstacles_, NodeSet& successors){
        for(const auto& obs : obstacles_){
            EncounterData e_data;
            e_data.obs_ptr = obs;
            auto vxs = obs->GetVxs(current->data.time);
            KeepVisibleVxsFromVehicle(current->data.position, obs->GetPosition(current->data.time), vxs);
            
            for(const auto& vx : vxs){
                e_data.vx = vx.first;
                if(pruning_params_.onlyOnceOnSameVx){
                    bool alreadyBeen = false;
                    auto node_it = current;
                    while(node_it != nullptr){
                        if(node_it->data.obs_ptr != nullptr){
                            if(node_it->data.obs_ptr->Id() == obs->Id() && node_it->data.vx == vx.first){
                                alreadyBeen = true;
                                break;
                            }
                        }
                        node_it = node_it->Parent();
                    }
                    if(alreadyBeen) continue;
                }
                for(const auto& speed : vh_data_.velocities){
                    e_data.approachingSpeed = speed;
                    std::vector<Eigen::Vector3d> ip;
                    FindInterceptPoints(current->data.position, speed, vx.second, obs->Velocity(), ip);         
                    // TODO check outside if in bb   
                    for(const auto& point : ip){
                        e_data.position = point.head(2);
                        e_data.time = current->data.time + std::chrono::duration<double>(point.z());
                        e_data.discoverySource = NodeSearch::DiscoverySource::EXPLORATION;
                        auto new_node = std::make_shared<AStarNode>(e_data, current);
                        successors.push_back(new_node);
                    }    
                }
            }
        }
    }
    
    NodeSet EvaluateNodes(NodeSet& successors, const NodePtr& current, const Eigen::Vector2d& goal_, const std::vector<ObsPtr>& obstacles_, NodeSet& closedSet, const NodeSet& openSet){
        NodeSet output;
        for(NodePtr& tentative_node : successors){
            // It is not good if it is in the closed set, or if its motion is not colregs compliant or if it collides with an obstacle
            bool isGood = true;
            if(IsNodeInSet(closedSet, tentative_node)) {
                tentative_node->data.discardReason = NodeSearch::DiscardReason::PREVIOUSLY_DISCARDED;
                isGood = false;
            }

            if(pruning_params_.colregsCompliant && tentative_node->data.obs_ptr != nullptr) {
                if(!PathEvaluator::RuleCompliantMotion(current, tentative_node)){
                    tentative_node->data.discardReason = NodeSearch::DiscardReason::NON_COMPLIANT;
                    isGood = false;
                }                
            }
            
            if(!std::none_of(obstacles_.begin(), obstacles_.end(), 
                        [&](const auto& obs) { 
                            return PathEvaluator::CollisionWithObs(current, tentative_node, obs, pruning_params_.colregsCompliant); 
                        })){
                tentative_node->data.discardReason = NodeSearch::DiscardReason::COLLISION;
                isGood = false;
                        }

            if(!isGood){
                //In standard A* we would just discard it for the nature of the environment is grid based
                //In this case we need to keep it in a list to check it later to avoid ripetitions
                closedSet.push_back(tentative_node);
                continue;
            }
            if(IsNodeInSet(openSet, tentative_node)){
                //If it is already in the open set, we need to check if it is better than the one already in the open set
                std::cerr << "Already in open set" << std::endl;
            }else{
                tentative_node->SetCosts(vh_data_, goal_);
                output.push_back(tentative_node);
            }
        }
        return output;
    }
};


}
#endif //ASTAR_GENERATOR_HPP