#include "oal/devel/generator.hpp"


oal::PathReport oal::Generator::FindPath(Pose start_, Eigen::Vector2d goal_, std::vector<ObsPtr> obstacles_, AstarPath& path_)
{
    PathReport response; // search output
    std::shared_ptr<AStarNode> current; // under exploration node
    NodeSet openSet; //Nodes yet to be explored
    NodeSet closedSet; //Explored nodes
    NodeSet reachableSet; //Every safe/possible node to reach 


    if(!GetFirstNodes(start_, goal_, obstacles_, openSet, response)) return response;
    reachableSet = openSet;
    
    while (!openSet.empty()) {
        auto current_it = openSet.begin();
        current = *current_it;
        for (auto it = openSet.begin(); it != openSet.end(); it++) {
            auto node = *it;
            if (node->GetTotalCost() <= current->GetTotalCost()) {
                current = node;
                current_it = it;
            }
        }

        auto temp = *current;
        if(ds_.logNodes) current->Log(logNodesFile);
        if(ds_.printCurrentNode) current->Print(" Current");

        // check final
        if ((current->data.position - goal_).norm() < pruning_params_.samePositionThreshold) {
            break;
        }

        closedSet.push_back(current);
        openSet.erase(current_it);

        //Exploration
        NodeSet successors;

        //Nodes to Goal
        GetNodesToGoal(current, goal_, obstacles_, successors);
        std::cerr<<successors.size()<<"\n";

        //Nodes to obstacles
        GetNodesToObstacles(current, obstacles_, successors);
        std::cerr<<successors.size()<<"\n";

        //Evaluate nodes (TODO: split evaluation, do nodes to goal first and cut short the search)
        auto newNodes = EvaluateNodes(successors, current, goal_, obstacles_, closedSet, openSet);
        std::cerr<<newNodes.size()<<"\n";
        
        openSet.insert(openSet.end(), newNodes.begin(), newNodes.end());
        reachableSet.insert(reachableSet.end(), newNodes.begin(), newNodes.end());
 
    }

    if ((current->data.position - goal_).norm() < pruning_params_.samePositionThreshold) {
        if(ds_.logNodes) current->Log(logNodesFile, "FinalPath");
        if(ds_.printPath) std::cerr <<"Here is the path: \n";
        while (current != nullptr)
        {   
            
            if(ds_.printPath)  current->Print();
            path_.Data().push_back(*current);
            current = current->Parent();
        }

        response.result = SearchResult::FOUND;
        return response;
    }else{
        response.result = SearchResult::PARTIAL;
        response.failMsg = "take best from reachable";
        //take best from reachable
    }

    if(ds_.printNodesStats) PrintNodeStats(closedSet);

    // return path;
    response.result = SearchResult::FAIL;
    response.failMsg = "Reached end of code";
    return response;
}

bool oal::Generator::GetFirstNodes(const Pose& start_, const Eigen::Vector2d& goal_, const std::vector<ObsPtr>& obstacles_, NodeSet& output, PathReport& response){
        //Check if starting in multiple bbs
        std::vector<ObsPtr> surrounding_obs;
        IsInBB(TimeDouble(0), start_.Position(), obstacles_, surrounding_obs);
        if(surrounding_obs.size() > 2){
            response.failMsg = "Start position is in more than one BB";
            response.result = SearchResult::FAIL;
            return false;
        }
        //If in one bb, look for nodes leading it out
        EncounterData data;
        data.position = start_.Position();
        data.heading = start_.Heading();
        auto start_node = std::shared_ptr<AStarNode>();
        if (surrounding_obs.size() == 1){
            auto obs = surrounding_obs[0];
            data.obs_ptr = obs;
            data.vx = NA;
            start_node = std::make_shared<AStarNode>(data);
            start_node->SetCosts(vh_data_, goal_);
            std::vector<Vx> allowedVxs;
            start_node->GetExitVxs(allowedVxs);
            for (const auto& vx : allowedVxs){
                for(const auto& speed : vh_data_.velocities){
                    std::vector<Eigen::Vector3d> ip;
                    FindInterceptPoints(start_.Position(), speed, vx.second, obs->Velocity(), ip);         
                    for(const auto& point : ip){
                        EncounterData data_exit_vx;
                        data_exit_vx.position = point.head(2);
                        data_exit_vx.approachingSpeed = speed;
                        data_exit_vx.obs_ptr = obs;
                        data_exit_vx.vx = vx.first;
                        data_exit_vx.time += std::chrono::duration<double>(point.z());
                        data_exit_vx.discoverySource = NodeSearch::DiscoverySource::EXIT_VX;
                        auto data_exit_node = std::make_shared<AStarNode>(data_exit_vx, start_node);
                        output.push_back(data_exit_node);
                    }    
                }
            }
        }else{
            data.discoverySource = NodeSearch::DiscoverySource::START;
            start_node = std::make_shared<AStarNode>(data);
            start_node->SetCosts(vh_data_, goal_);
            output.push_back(start_node);
        }
        if(output.empty()){
            response.failMsg = "Starting in one bb but no exit found";
            response.result = SearchResult::FAIL;
            return false;
        }   
        return true;
    }

void oal::Generator::KeepVisibleVxsFromVehicle(Eigen::Vector2d vehicle, Eigen::Vector2d obstacle, std::vector<Vx>& vxs){
    // Remove vxs that are not visible from position
    std::vector<Eigen::Vector2d> vxs_positions = {vxs[0].second, vxs[1].second, vxs[2].second, vxs[3].second};
    auto visible = ComputeVisibleVertices(vehicle, obstacle, vxs_positions);
    std::vector<Vx> new_vxs;
    for(int i = 0; i < 4; i++){
        if(visible[i]){
            new_vxs.push_back(vxs[i]);
        }
    }
    vxs = new_vxs;
}

bool oal::Generator::IsInBB(TimeDouble time, Eigen::Vector2d point, const std::vector<ObsPtr>& obstacles,
                std::vector<ObsPtr>& surrounding_obs){
    bool isInBB = false;
    for(const auto& obs : obstacles){
        auto vxs = obs->GetVxs(time);
        std::vector<Eigen::Vector2d> vxs_positions = {vxs[0].second, vxs[1].second, vxs[2].second, vxs[3].second};
        if(IsPointInQuadrilateral(point, vxs_positions)){
            isInBB = true;
            surrounding_obs.push_back(obs);
        }
    }
    return isInBB;
}

bool oal::Generator::IsNodeInSet(const NodeSet& nodes, const NodePtr& node) {
    for (auto& n : nodes) {
        bool isSimilar = ((n->data.position - node->data.position).norm() < pruning_params_.samePositionThreshold && 
                          std::abs((n->data.time - node->data.time).count()) < pruning_params_.sameTimeThreshold );

        if(n->data.obs_ptr != nullptr && node->data.obs_ptr != nullptr) {
            isSimilar &=  n->data.obs_ptr->Id() == node->data.obs_ptr->Id() &&
                          n->data.vx == node->data.vx;
        }

        if (isSimilar) return true;
    }
    return false;
}

void oal::Generator::PrintNodeStats(const NodeSet& set){
    std::unordered_map<std::string, uint> discovery_source_counts;
    std::unordered_map<std::string, uint> discard_reason_counts;

    for (const NodePtr& node : set) {
        // Track discovery source
        std::string discovery_source = node->data.discoverySource;  // Assuming this is a string
        discovery_source_counts[discovery_source]++;

        // Track discard reason if applicable
        std::string discard_reason = node->data.discardReason;  // Assuming discardReason exists
        discard_reason_counts[discard_reason]++;
    }

    // Print results
    std::cout << "Discovery Sources:\n";
    for (const auto& [source, count] : discovery_source_counts) {
        std::cout << "  " << source << ": " << count << "\n";
    }

    std::cout << "\nDiscard Reasons:\n";
    for (const auto& [reason, count] : discard_reason_counts) {
        std::cout << "  " << reason << ": " << count << "\n";
    }
     std::cout << "\n";
}