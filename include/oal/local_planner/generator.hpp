#ifndef ASTAR_GENERATOR_HPP 
#define ASTAR_GENERATOR_HPP


#include <fstream>
#include <memory>
#include <unordered_map>

#include "oal/obstacle.hpp"
#include "oal/geometric_utilities.hpp"
#include "oal/local_planner/node.hpp"
#include "oal/local_planner/path.hpp"
#include "oal/local_planner/path_evaluator.hpp"


namespace oal{

struct DebugSettings{
    bool printCurrentNode = false;
    bool printPath = false;
    bool printComputedVxs = false;
    bool printNodeEvolutionStats = false;
    bool printPathUnsafetyReason = false;

    bool logNodes = false;
    std::string logNodesPathFile = "WARNING_LOG_NODES_PATH_FILE_NOT_SET.txt";

    bool logObstacles = false;
    std::string logObstaclesPathFile = "WARNING_LOG_OBSTACLES_PATH_FILE_NOT_SET.txt";

    bool printNodesStats = false;
};

class Generator
{

    VehicleData vh_data_; // vehicle capabilities, not time dependent
    PruningParams pruning_params_; // exploration settings

    DebugSettings ds_;
    std::ofstream logNodesFile;
    std::ofstream logObstaclesFile;

    bool GetFirstNodes(const Pose& start_, const Eigen::Vector2d& goal_, const std::vector<ObsPtr>& obstacles_, NodeSet& output, PathReport& response);

    bool IsNodeInSet(const NodeSet& nodes, const NodePtr& node);

    std::vector<oal::NodePtr>::iterator GetBestFromSet(NodeSet& set) const;

    bool IsInBB(TimeDouble time, const Eigen::Vector2d& point, 
                const std::vector<ObsPtr>& obstacles,
                std::vector<ObsPtr>& surrounding_obs);

    void GetVisibleVxsFromVehicle(const NodePtr& current, const ObsPtr& obstacle, std::vector<Vx>& vxs);
        
        //Eigen::Vector2d vehicle, Eigen::Vector2d obstacle, std::vector<Vx>& vxs);

    void PrintNodeStats(const NodeSet& set);
    void LogObstacles(const std::vector<ObsPtr>& obstacles);

    bool GetNodesToGoal(const NodePtr& current, const Eigen::Vector2d& goal_, const std::vector<ObsPtr>& obstacles_, NodeSet& successors);
    void GetNodesToObstacles(const NodePtr& current, const std::vector<ObsPtr>& obstacles_, NodeSet& successors);
    NodeSet EvaluateNodes(NodeSet& successors, const NodePtr& current, const Eigen::Vector2d& goal_, const std::vector<ObsPtr>& obstacles_, NodeSet& closedSet, const NodeSet& openSet);

    void ReconstructPath(const NodePtr& goal, AstarPath& path_);


public:

    Generator(VehicleData vh_data, PruningParams pruning_params, DebugSettings ds = {}) 
        : vh_data_(vh_data), pruning_params_(pruning_params), ds_(ds) {

            //Debug settings
            if(ds_.logNodes){
                if (logNodesFile.is_open()) logNodesFile.close();
                logNodesFile.open(ds_.logNodesPathFile, std::ofstream::trunc);
            } 
            if(ds_.logObstacles){
                if (logObstaclesFile.is_open()) logObstaclesFile.close();
                logObstaclesFile.open(ds_.logObstaclesPathFile, std::ofstream::trunc);
            } 
        }

    //auto Obstacles() -> std::vector<ObsPtr>& { return obstacles_; }
    
    PathReport FindPath(const Pose& start_, Eigen::Vector2d goal_, const std::vector<ObsPtr>& obstacles_, AstarPath& path_);

    bool IsPathValid(AstarPath path, const Pose& vehicle, const std::vector<ObsPtr>& obstacles_, Eigen::Vector2d &unreachable_wp);

};


}
#endif //ASTAR_GENERATOR_HPP