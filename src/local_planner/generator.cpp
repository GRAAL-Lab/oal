#include "oal/local_planner/generator.hpp"


oal::PathReport oal::Generator::FindPath(const Pose& start_, Eigen::Vector2d goal_, const std::vector<ObsPtr>& obstacles_, AstarPath& path_)
{
    PathReport response; // search output
    std::shared_ptr<AStarNode> current; // under exploration node
    NodeSet openSet; //Nodes yet to be explored
    NodeSet closedSet; //Explored nodes
    NodeSet reachableSet; //Every safe/possible node to reach 
    if(ds_.logObstacles) LogObstacles(obstacles_);


    if(!GetFirstNodes(start_, goal_, obstacles_, openSet, response)) return response;
    reachableSet = openSet;

    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    while (!openSet.empty()) {
        clock_gettime(CLOCK_MONOTONIC, &end);
        double time_result = end.tv_sec - start.tv_sec + (end.tv_nsec - start.tv_nsec)*1e-9;
        if (time_result > 1.0) { //time failure limit
            response.result = SearchResult::FAIL;
            response.failMsg = "OUT OF TIME";
            break;
        }

        auto current_it = GetBestFromSet(openSet);
        current = *current_it;

        //remove aimtogoal first: if(ds_.logNodes) current->Log(logNodesFile);
        if(ds_.printCurrentNode) current->Print(" Current");

        // check final
        if ((current->data.position - goal_).norm() < pruning_params_.samePositionThreshold) {
            break;
        }

        current->data.discardReason = NodeSearch::DiscardReason::EXPLORED;
        closedSet.push_back(current);
        openSet.erase(current_it);

        //Exploration
        //Nodes to Goal
        NodeSet nodesToGoal, nodesToObstacles, nodesEligible;
        GetNodesToGoal(current, goal_, obstacles_, nodesToGoal); //True if goal is in bb sometimes and so it found new goal just outside it
        nodesEligible = EvaluateNodes(nodesToGoal, current, goal_, obstacles_, closedSet, openSet);

        if(nodesEligible.empty()){
            //Nodes to obstacles
            GetNodesToObstacles(current, obstacles_, nodesToObstacles);
            nodesEligible = EvaluateNodes(nodesToObstacles, current, goal_, obstacles_, closedSet, openSet);
        }
        
        openSet.insert(openSet.end(), nodesEligible.begin(), nodesEligible.end());
        reachableSet.insert(reachableSet.end(), nodesEligible.begin(), nodesEligible.end());
 
    }

    // Check if the current node is close enough to the goal
    if ((current->data.position - goal_).norm() < pruning_params_.samePositionThreshold) {
        if (ds_.logNodes) current->Log(logNodesFile, "FinalPath");
        if (ds_.printPath) std::cerr << "Here is the path:\n";

        ReconstructPath(current, path_);
        
        response.result = SearchResult::FOUND;
        return response;
    }

    // Print node statistics if enabled
    if (ds_.printNodesStats) PrintNodeStats(closedSet);

    // Select the best candidate from the reachable set
    auto best_candidate_it = GetBestFromSet(reachableSet);
    auto best_candidate = *best_candidate_it;

    double best_to_goal = (best_candidate->data.position - goal_).norm();
    double start_to_goal = (start_.Position() - goal_).norm();

    // Check if progress toward the goal is significant
    if (best_to_goal / start_to_goal < 0.3) {
        std::ostringstream msg;
        msg << "Getting Closer: " << static_cast<int>(best_to_goal) 
            << "/" << static_cast<int>(start_to_goal) << "m to target.";

        response.result = SearchResult::PARTIAL;
        response.failMsg = msg.str();
        
        ReconstructPath(best_candidate, path_);
        return response;
    }

    // If no significant progress, return failure
    if (ds_.logNodes) current->Log(logNodesFile, "End of code");
    response.result = SearchResult::FAIL;
    response.failMsg = "Reached end of code, usually to many obs at starting position";
    return response;

}

bool oal::Generator::IsPathValid(AstarPath path, const Pose& vehicle, const std::vector<ObsPtr>& obstacles_, Eigen::Vector2d &unreachable_wp){
    // the waypoint should be just the ones left to reach, not the whole path returned by the library
    if (path.Data().empty()) {
        if(ds_.printPathUnsafetyReason) std::cerr << "OAL: PATH IS EMPTY -> CHECK = FALSE" << std::endl;
        return false;
    }
    
    EncounterData start_data;
    start_data.position = vehicle.Position();
    start_data.heading = vehicle.Heading();
    NodePtr start = std::make_shared<AStarNode>(start_data);
    //start->Print("first");
    while(!path.Data().empty()){
        auto target = path.Data().front();
        path.Data().pop_front();
        //target->Print();

        for(const auto& obs : obstacles_){
            std::vector<Eigen::Vector3d> collisions;
            if(PathEvaluator::CollisionWithObs(start, target, obs, collisions, false)){
                unreachable_wp = target->data.position;
                if(ds_.printPathUnsafetyReason) 
                    std::cerr<<"Cannot reach wp: "<<unreachable_wp.transpose()<<std::endl;
                return false;
            }
        }

        start = target;
    }
    return true;
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
    data.discoverySource = NodeSearch::DiscoverySource::START;
    std::shared_ptr<AStarNode> start_node;
    if (surrounding_obs.size() == 1){
        auto obs = surrounding_obs[0];
        data.obs_ptr = obs;
        data.vx = NA; //inside obs bb
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
                    data_exit_node->SetCosts(vh_data_, goal_);
                    output.push_back(data_exit_node);
                }    
            }
        }
    }else{
        start_node = std::make_shared<AStarNode>(data);
        start_node->SetCosts(vh_data_, goal_);
    }
    output.push_back(start_node);
    if(output.empty()){
        response.failMsg = "Starting in one bb but no exit found";
        response.result = SearchResult::FAIL;
        return false;
    }   
    return true;
}

std::vector<oal::NodePtr>::iterator oal::Generator::GetBestFromSet(NodeSet& set) const{
    std::vector<NodePtr>::iterator best_it = set.begin();
    auto best = *best_it;
    for (auto it = set.begin(); it != set.end(); it++) {
        auto node = *it;
        if (node->GetTotalCost() <= best->GetTotalCost()) {
            best = node;
            best_it = it;
        }
    }
    return best_it;
}

bool oal::Generator::GetNodesToGoal(const NodePtr& current, const Eigen::Vector2d& goal_, const std::vector<ObsPtr>& obstacles_, NodeSet& successors){
    // Nodes to goal
    auto reachingTime = [](Eigen::Vector2d start, Eigen::Vector2d goal, double speed){
        return std::chrono::duration<double>((goal - start).norm() / speed);
    };
    bool newGoalSet = false;
    for(const auto& speed : vh_data_.velocities){
        Eigen::Vector2d actual_goal = goal_;
        EncounterData data;
        data.approachingSpeed = speed;
        data.position = actual_goal;
        data.time = current->data.time + reachingTime(current->data.position, actual_goal, speed);
        data.discoverySource = NodeSearch::DiscoverySource::AIM_TO_GOAL;
        std::shared_ptr<AStarNode> goal_node = std::make_shared<AStarNode>(data, current); // just for isInBB routine

        std::vector<ObsPtr> so;
        if(IsInBB(data.time, goal_, obstacles_, so)) {//continue;
            // Get the closest point to goal which is out of any surrounding bb
            for(const auto& obs : so){
                std::vector<Eigen::Vector3d> collisions;
                PathEvaluator::CollisionWithObs(current, goal_node, obs, collisions);
                for(const auto& c : collisions){
                    if((c.head(2) - goal_).norm() < (actual_goal - goal_).norm()){
                        actual_goal = c.head(2);
                        data.obs_ptr = obs;
                    }
                }
            }
            data.position = actual_goal;
            data.time = current->data.time + reachingTime(current->data.position, actual_goal, speed);
            data.discoverySource = NodeSearch::DiscoverySource::CLOSEST_TO_GOAL;
            newGoalSet = true;
        }

        std::shared_ptr<AStarNode> actual_goal_node = std::make_shared<AStarNode>(data, current);
        //Setting costs wrt a goal which is actually the node position makes the search finish on this node
        // iff it does pass following checks (collisions and colregs)
        // TODO check how colregs are enforced when obs_ptr exists but vx==NA, until now this pair is used only when starting in a bb        
        actual_goal_node->SetCosts(vh_data_, actual_goal); 
        successors.push_back(actual_goal_node);
    }
    return newGoalSet;
}

void oal::Generator::GetNodesToObstacles(const NodePtr& current, const std::vector<ObsPtr>& obstacles_, NodeSet& successors){
    for(const auto& obs : obstacles_){
        EncounterData e_data;
        e_data.obs_ptr = obs;
        std::vector<Vx> vxs;
        GetVisibleVxsFromVehicle(current, obs, vxs); 
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
                for(const auto& point : ip){
                    e_data.position = point.head(2);
                    e_data.time = current->data.time + std::chrono::duration<double>(point.z());
                    e_data.discoverySource = NodeSearch::DiscoverySource::EXPLORATION;
                    std::vector<ObsPtr> so;
                    if(!IsInBB(e_data.time, e_data.position, obstacles_, so)){
                        auto new_node = std::make_shared<AStarNode>(e_data, current);
                        successors.push_back(new_node);
                    }
                }    
            }
        }
    }
}
    
oal::NodeSet oal::Generator::EvaluateNodes(NodeSet& successors, const NodePtr& current, 
                                           const Eigen::Vector2d& goal_, const std::vector<ObsPtr>& obstacles_, 
                                           NodeSet& closedSet, const NodeSet& openSet) {
    NodeSet output;

    for (NodePtr& tentative_node : successors) {

        // Not doing this check because "similar" nodes can have different paths: 
        //  use logs (commented below) to see how good nodes can still be found in discarded ones
        // if (IsNodeInSet(closedSet, tentative_node)) {
        //     tentative_node->data.discardReason = NodeSearch::DiscardReason::PREVIOUSLY_DISCARDED;
        //     closedSet.push_back(tentative_node);
        //     continue;
        // }

        if (pruning_params_.colregsCompliant && tentative_node->data.obs_ptr &&
            !PathEvaluator::RuleCompliantMotion(current, tentative_node)) {
            tentative_node->data.discardReason = NodeSearch::DiscardReason::NON_COMPLIANT;
            closedSet.push_back(tentative_node);
            continue;
        }
        
        std::vector<Eigen::Vector3d> collisions;
        if (std::any_of(obstacles_.begin(), obstacles_.end(), [&](const auto& obs) {    
                return PathEvaluator::CollisionWithObs(current, tentative_node, obs, collisions, pruning_params_.colregsCompliant); 
            })) {
            tentative_node->data.discardReason = NodeSearch::DiscardReason::COLLISION;
            closedSet.push_back(tentative_node);
            continue;
        }

        // In function good nodes but similar to ones in closedSet are logged
        // bool logNodes = ds_.logNodes;
        // ds_.logNodes = true;
        // IsNodeInSet(closedSet, tentative_node);
        // ds_.logNodes = logNodes;

        if (!IsNodeInSet(openSet, tentative_node)) {
            tentative_node->SetCosts(vh_data_, goal_);
            output.push_back(tentative_node);
        }else{
            // Get best 
        }
    }

    return output;  // Moves the vector to avoid copying
}

void oal::Generator::GetVisibleVxsFromVehicle(const NodePtr& current, const ObsPtr& obstacle, std::vector<Vx>& vxs) {
    // Retrieve obstacle vertices at the current time
    vxs = obstacle->GetVxs(current->data.time);
    std::vector<Vx> visible_vxs;

    // If the current node is associated with the obstacle
    if (current->data.obs_ptr == obstacle) {
        // Set adjacent vertices as visible based on the current vertex
        if (current->data.vx == FR || current->data.vx == RL) {
            visible_vxs = { vxs[FL], vxs[RR] };
        } else {
            visible_vxs = { vxs[FR], vxs[RL] };
        }
    } else {
        // Compute visibility from the current position
        std::vector<Eigen::Vector2d> vxs_positions = { vxs[0].second, vxs[1].second, vxs[2].second, vxs[3].second };
        auto visible = ComputeVisibleVertices(current->data.position, obstacle->GetPosition(current->data.time), vxs_positions);

        // Add only visible vertices
        for (size_t i = 0; i < vxs.size(); ++i) {
            if (visible[i]) {
                visible_vxs.push_back(vxs[i]);
            }
        }
    }

    // Update vxs with the filtered visible vertices
    vxs = std::move(visible_vxs);
}


bool oal::Generator::IsInBB(TimeDouble time, const Eigen::Vector2d& point, const std::vector<ObsPtr>& obstacles,
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

        if(isSimilar && n->data.obs_ptr != nullptr && n->data.obs_ptr == node->data.obs_ptr) {
            isSimilar =  n->data.vx == node->data.vx;
        }

        if (isSimilar) {
            if(ds_.logNodes){
                node->Log(logNodesFile,"tentative");
                n->Log(logNodesFile,"pd");
            }
            return true;
        }
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
    std::cout << "\n\nNodes Stats:\n\n";
    std::cout << "  Discovery Sources:\n";
    double discovery_count, discarded_count;
    for (const auto& [source, count] : discovery_source_counts) {
        std::cout << "      " << source << ": " << count << "\n";
        discovery_count += count;
    }

    std::cout << "\n  Discard Reasons:\n";
    for (const auto& [reason, count] : discard_reason_counts) {
        std::cout << "      " << reason << ": " << count << "\n";
        discarded_count += count;
    }

    std::cout << "\n  Discovered - Discarded: "<< discovery_count - discarded_count<<"\n\n"; 
}

void oal::Generator::LogObstacles(const std::vector<ObsPtr>& obstacles){
    for(const auto& obs : obstacles){
        obs->Log(logObstaclesFile);
    }
}

void oal::Generator::ReconstructPath(const NodePtr& goal, AstarPath& path_) {
    //FINAL PATH DOES NOT HAVE THE START NODE NOW
    auto node = goal;
    while (node->Parent() != nullptr) {
        if (ds_.printPath) node->Print();
        path_.Data().push_front(node);
        node = node->Parent();
    }
}