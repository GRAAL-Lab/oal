#ifndef ASTAR_NODE_HPP
#define ASTAR_NODE_HPP

#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
using json = nlohmann::ordered_json;

#include "oal/data_structs.hpp"
#include "oal/geometric_utilities.hpp"
#include "oal/obstacle.hpp"

namespace oal {

class AStarNode {

    std::shared_ptr<AStarNode> parent_ = nullptr;

    double costToReach = -1; // cost to reach the Node
    double costToGoal = -1; // estimated cost to reach Goal

public:
    AStarNode(const AStarNode&) = delete;
    AStarNode& operator=(const AStarNode&) = delete;
    EncounterData data;

    bool isGoal = false;
    bool reached = false;

    auto Parent() -> std::shared_ptr<AStarNode>& { return parent_; }

    AStarNode(EncounterData data, std::shared_ptr<AStarNode> parent = nullptr, bool isGoal = false);

    void SetCosts(const VehicleData& vh_data, Eigen::Vector2d goal);

    double GetTotalCost() const { return costToReach + costToGoal; }

    void GetExitVxs(std::vector<Vx>& allowedVxs) const;
    // void FindExitVxs(std::vector<vx_id> &allowedVxs) const;

    int NumberOfParents() const
    {
        if (parent_ != nullptr)
            return 1 + parent_->NumberOfParents();
        return 0;
    }

    void Print(std::string id = "") const
    {
        if (parent_ != nullptr) {
            std::cerr << "Node" << id << ": " << std::endl;
        } else {
            std::cerr << "(First) Node" << id << ": " << std::endl;
        }
        std::cerr << "  - Time: " << data.time.count() << "\n  - Pos: " << data.position.transpose() << "\n";
        std::cerr << "  - Heading: " << data.heading << "\n";
        
        std::cerr << "  - Reaching speed: " << data.approachingSpeed << "\n";

        if (data.obs_ptr != nullptr) {
            std::cerr << "  - Obs: " << data.obs_ptr->Id() << "/" << VxIdToString(data.vx) << "\n";
        }
    }

    void Log(std::ofstream& logFile, std::string id = "") const
    {
        logFile << "---" << std::endl;
        logFile << "Trace_" << id << std::endl;
        const AStarNode* current = this;
        int count = current->NumberOfParents();
        while (current != nullptr) {
            logFile << "_Node_" << count << "_Position_" << current->data.position.x() << "_" << current->data.position.y() << "_Heading_" << current->data.heading << "_Time_" << current->data.time.count() << "_Speed_" << current->data.approachingSpeed;
            if (current->data.obs_ptr != nullptr)
                logFile << "_Obstacle_" << current->data.obs_ptr->Id() << "_Vx_" << VxIdToString(data.vx);
            logFile << std::endl;
            current = current->parent_.get();
            count--;
        }
    }

    json LogTrace() const
    {
        json trace = json::array(); // Use a JSON array to collect nodes

        const AStarNode* current = this;
        int count = current->NumberOfParents();

        while (current != nullptr) {
            json nodeEntry;
            nodeEntry["NodeNumber"] = count;
            nodeEntry["Position"] = {
                { "x", current->data.position.x() },
                { "y", current->data.position.y() }
            };
            nodeEntry["Heading"] = current->data.heading;
            nodeEntry["Time"] = current->data.time.count();
            nodeEntry["Speed"] = current->data.approachingSpeed;

            if (current->data.obs_ptr != nullptr) {
                nodeEntry["Obstacle"] = current->data.obs_ptr->Id();
                nodeEntry["Vx"] = VxIdToString(current->data.vx);
            } else {
                nodeEntry["Obstacle"] = "";
                nodeEntry["Vx"] = "";
            }

            // Push this node entry to the trace array
            trace.push_back(nodeEntry);

            current = current->parent_.get(); // Move to parent
            count--;
        }

        return trace;
    }
};

}
#endif // ASTAR_NODE_HPP