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

struct NodeLogger{
    bool log = false;
    std::string dirPath = "NULL";
    std::string fileName = "NULL";

    void Init(std::ofstream& logNodeFile) {
        if (log) {
            if (logNodeFile.is_open()) logNodeFile.close();
            
            if (dirPath == "NULL" || fileName == "NULL") {
                throw std::runtime_error("[oal] LOG NODES PATH OR FILE NAME NOT SET");
            }
            
            std::string logFilePath = dirPath + "/" + fileName;
            logNodeFile.open(logFilePath, std::ofstream::trunc);
            
            if (!logNodeFile.is_open()) {
                throw std::runtime_error("[oal] FAILED TO OPEN LOG FILE: " + logFilePath);
            }
        }
    }
};

struct DebugSettings{
    bool printCurrentNode = false;
    bool printPath = false;
    bool printComputedVxs = false;
    bool printNodeEvolutionStats = false;
    bool printPathUnsafetyReason = false;

    NodeLogger completePath;
    NodeLogger failedPath;
    NodeLogger validPath;
    NodeLogger notValidPath;

    NodeLogger freeLogger;

    bool logObstacles = false;
    std::string logObstaclesPathFile = "NULL";

    bool printNodesStats = false;
};

class Generator
{

    VehicleData vh_data_; // vehicle capabilities, not time dependent
    PruningParams pruning_params_; // exploration settings

    std::shared_ptr<DebugSettings> ds_;
    std::ofstream logNodesCPFile;
    std::ofstream logNodesFPFile;
    std::ofstream logNodesVPFile;
    std::ofstream logNodesNVPFile;
    std::ofstream logNodesFreeFile;

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

    Generator(VehicleData vh_data, PruningParams pruning_params, std::shared_ptr<DebugSettings> ds = nullptr) 
        : vh_data_(vh_data), pruning_params_(pruning_params), ds_(ds) {
            
            if(ds_ == nullptr) ds_ = std::make_shared<DebugSettings>();
            //Debug settings
            ds_->completePath.Init(logNodesCPFile);
            ds_->failedPath.Init(logNodesFPFile);
            ds_->validPath.Init(logNodesVPFile);
            ds_->notValidPath.Init(logNodesNVPFile);
            ds_->freeLogger.Init(logNodesFreeFile);

        }

    //auto Obstacles() -> std::vector<ObsPtr>& { return obstacles_; }
    
    PathReport FindPath(const Pose& start_, Eigen::Vector2d goal_, const std::vector<ObsPtr>& obstacles_, AstarPath& path_);

    bool IsPathValid(AstarPath path, const Pose& vehicle, const std::vector<ObsPtr>& obstacles_, Eigen::Vector2d &unreachable_wp);

};


}
#endif //ASTAR_GENERATOR_HPP