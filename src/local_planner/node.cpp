#include "oal/local_planner/node.hpp"

oal::AStarNode::AStarNode(EncounterData d, std::shared_ptr<AStarNode> parent) 
: data(d) {
    if(parent != nullptr){
        if(parent->costToReach == -1) std::cerr<<"[WARNING] Linked to node with no costs set\n";
        parent_ = parent;
        Eigen::Vector2d pathToThis = data.position - parent_->data.position;
        data.heading = atan2(pathToThis.y(), pathToThis.x());
    }// else it is first node and so set already 


};

void oal::AStarNode::GetExitVxs(std::vector<Vx> &allowedVxs) const{
    if(abs(data.time.count()) > 0.01) throw std::runtime_error("[GetExitVxs] Time should be 0 for this function");
    auto obs = *data.obs_ptr;
    auto obs_P_vehicle = obs.InitialPose().FromWorld2ThisFrame(data.position);
    auto vxs = obs.GetVxs(data.time);

    auto x = [vxs](VxId id) { return vxs[id].second.x(); };
    auto y = [vxs](VxId id) { return vxs[id].second.y(); };

    bool IsLeftOfDiag1 = (obs_P_vehicle.y() >=
                            y(FL) / x(FL) *
                            obs_P_vehicle.x()); // Being left of diag FL-RR
    bool IsLeftOfDiag2 = (obs_P_vehicle.y() >=
                            y(FR) / x(FR) *
                            obs_P_vehicle.x()); // Being left of diag FR-LL

    if (IsLeftOfDiag1 && IsLeftOfDiag2) {
        // USV in port side of bb
        //  go for FL or RL
        allowedVxs.push_back(vxs[FL]);
        allowedVxs.push_back(vxs[RL]);
    } else {
        if (IsLeftOfDiag1) {
            // USV in stern side of bb
            //  go for RL or RR
            allowedVxs.push_back(vxs[RL]);
            allowedVxs.push_back(vxs[RR]);
        } else {
            if (IsLeftOfDiag2) {
                // USV in bow side of bb
                //  go for FR or FL
                allowedVxs.push_back(vxs[FR]);
                allowedVxs.push_back(vxs[FL]);
            } else {
                // USV in starboard side of bb
                //  go for FR or RR
                allowedVxs.push_back(vxs[FR]);
                allowedVxs.push_back(vxs[RR]);
            }
        }
    }
}

void oal::AStarNode::SetCosts(VehicleData vh_data, Eigen::Vector2d goal){
    costToReach = data.time.count(); 
    Eigen::Vector2d dist_to_goal = goal - data.position;
    costToGoal = dist_to_goal.norm() / *std::max_element(std::begin(vh_data.velocities), std::end(vh_data.velocities));

    if(vh_data.max_yaw_rate > ZERO_NUMERICAL){
        // The time contribution of a node is also:
        //    - how much it takes to change course to reach this node
        //    - how much it takes to change course to reach goal from this node
        if(parent_ != nullptr){
            double changeInHeadingToThis = std::min(parent_->data.heading - data.heading, data.heading - parent_->data.heading);
            costToReach += changeInHeadingToThis / vh_data.max_yaw_rate;
        }
        if(dist_to_goal.norm() > 0.01){
            double changeInHeadingToGoal = std::min(data.heading - atan2(dist_to_goal.y(), dist_to_goal.x()), atan2(dist_to_goal.y(), dist_to_goal.x()) - data.heading);
            costToGoal += changeInHeadingToGoal / vh_data.max_yaw_rate;
        }
    }
}
