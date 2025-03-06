#include "oal/local_planner/path_evaluator.hpp"

std::vector<std::string> oal::PathEvaluator::highPriorityObstacles;
bool oal::PathEvaluator::isHPListSet = false;

bool oal::PathEvaluator::RuleCompliantMotion(const NodePtr& start, NodePtr& goal) {

    if (goal->data.obs_ptr == nullptr || goal->data.obs_ptr->Velocity().norm() <= ZERO_VELOCITY) return true;
    
    /*auto it = std::find(goal.overtakingObsList.begin(), goal.overtakingObsList.end(), goal.obs_ptr->id);
    if( it != goal.overtakingObsList.end()){
      // Moving to overtaking obs
    }*/

    // Check only if moving between different obstacles
    if (start->data.obs_ptr.get() == goal->data.obs_ptr.get()) {
        // otherwise is compliant iff goal vx is not limited because of past maneuver
        // for (const vx_id &vx: start.currentObsLimitedVxs) {
        //     if (vx == goal.vx) return false;
        // }
        return true;
    }
    // Get the approaching angle of own ship wrt target ship
    Eigen::Vector2d path = goal->data.position - start->data.position;
    double theta = GetBearing(path, goal->data.obs_ptr->InitialPose().Heading());
    // Check colregs depending on situation
    auto encounterType = EncounterType(theta);
    if(encounterType == EncounterTypes::HEAD_ON){
        if (goal->data.vx == FR || goal->data.vx == RR) {
            // should be on left
            return false;
        }
        //goal.currentObsLimitedVxs.push_back(FR);
    }else if(encounterType == EncounterTypes::VH_OVERTAKING){
        // if (goal.data.vx == FR) {
        //     goal.currentObsLimitedVxs.push_back(RR);
        //     goal.currentObsLimitedVxs.push_back(FL);
        // }
        // if (goal.data.vx == FL) {
        //     goal.currentObsLimitedVxs.push_back(FR);
        // }
        //goal.overtakingObsList.push_back(goal.obs_ptr->id);
        // avoid future crossings
    }else if(encounterType == EncounterTypes::VH_CROSSING_FROM_LEFT){
        // give way
        if (goal->data.vx == FR || goal->data.vx == FL) {
            // should give way
            return false;
        }
        // goal.currentObsLimitedVxs.push_back(FR);
        // goal.currentObsLimitedVxs.push_back(FL);
    }else if(encounterType == EncounterTypes::VH_CROSSING_FROM_RIGHT){
        if(HasHigherPriority(goal->data.obs_ptr->ObsClass())){
            // crossing from right, should be stand on but
            //  if execution comes here, obs is high priority,
            //  so give way but avoid rear vxs
            //goal.currentObsLimitedVxs.push_back(RR);  // avoid the maneuver to become a head on
            if (goal->data.vx == RR || goal->data.vx == RL) {
                return false;
            }
        }
    }
    
    return true;
}

bool oal::PathEvaluator::HasHigherPriority(const std::string& obsClass) {
    if(!isHPListSet) throw std::runtime_error("Calling HasHigherPriority() but HighPriorityList not set");
    return std::find(highPriorityObstacles.begin(), highPriorityObstacles.end(), obsClass) != highPriorityObstacles.end();
}

void oal::PathEvaluator::SetHighPriorityObstacles(const std::vector<std::string>& obstacles) {
    highPriorityObstacles = obstacles;
    isHPListSet = true;
}

std::string oal::PathEvaluator::EncounterType(double angle){
    //angle is the angle between obs forward axis and ownVh heading upon approach (on vx)
    if(abs(angle) <= EncounterTypes::HeadOnAngle) return EncounterTypes::HEAD_ON;
    if(abs(angle) >= EncounterTypes::OvertakingAngle) return EncounterTypes::VH_OVERTAKING;
    if(angle < 0) return EncounterTypes::VH_CROSSING_FROM_LEFT;
    //if(angle > 0) 
    return EncounterTypes::VH_CROSSING_FROM_RIGHT;
}

bool oal::PathEvaluator::CollisionWithObs(const NodePtr& start, const NodePtr& goal, const ObsPtr& obs, std::vector<Eigen::Vector3d>& collisions, bool colregs){
    // Check if path collide with the obstacle (true == collision)

    Eigen::Vector2d path = goal->data.position - start->data.position;

    auto vxs = obs->GetVxs(start->data.time);
    if(vxs.size() != 4){
        throw std::runtime_error("Obstacle " + obs->Id() + " has not 4 vertexes");
    }
    Eigen::Vector3d obs_velocity(obs->Velocity().x(), obs->Velocity().y(), 1);

    bool isDepartingObs = false;
    bool isApproachingObs = false;

    if (start->data.obs_ptr != nullptr) {
        isDepartingObs = obs->Id() == start->data.obs_ptr->Id();
    }
    if (goal->data.obs_ptr != nullptr) {
        isApproachingObs = obs->Id() == goal->data.obs_ptr->Id();
    }

    // Colregs check (ignore crossing from right TS)
    if (colregs) {
        // Angle between
        double theta = GetBearing(Eigen::Vector2d(path.x(), path.y()), obs->InitialPose().Heading());
        if(EncounterType(theta) == EncounterTypes::VH_CROSSING_FROM_RIGHT && !HasHigherPriority(obs->ObsClass()) &&
            obs->Velocity().norm() > ZERO_VELOCITY) {
            // crossing from right, stand on
            return false;
        }
    }

    auto get3DVector = [& vxs] (int index) {
        return Eigen::Vector3d(vxs[index].second.x(), vxs[index].second.y(), 0.0);
    };

    // Starts in TS bb and does NOT go to same obs
    if (isDepartingObs && start->data.vx == NA) {
        if (!isApproachingObs) {
            // Check both bb diagonals for collisions
            //DOUBT: does getVxs return all 4 of them since we are inside?
            Eigen::Vector3d p1 = {start->data.position.x(), start->data.position.y(), 0};
            Eigen::Vector3d p2 = {goal->data.position.x(), goal->data.position.y(), goal->data.time.count() - start->data.time.count()};

            Eigen::Vector3d diag1 = get3DVector(FR) - get3DVector(RL);
            Eigen::Vector3d diag2 = get3DVector(FL) - get3DVector(RR);
            Eigen::Vector3d planeNormal1 = obs_velocity.cross(diag1);
            Eigen::Vector3d planeNormal2 = obs_velocity.cross(diag2);
            Eigen::Vector3d cp1, cp2;

            bool cutThroughDiag1 = FindLinePlaneIntersection(p1, p2, get3DVector(FR), planeNormal1, cp1);
            bool cutThroughDiag2 = FindLinePlaneIntersection(p1, p2, get3DVector(FL), planeNormal2, cp2);

            if (cutThroughDiag1 || cutThroughDiag2) {
                if(cutThroughDiag1) collisions.push_back(cp1);
                if(cutThroughDiag2) collisions.push_back(cp2);
                return true;
            }
        }
        // If it is both departing and approaching, then we ar
        return false;
    }

    // Do not check the goal obstacle: the algorithm already aims for the visible vxs
    // This condition goes here because an obs could be both departing and approaching obs
    if (isApproachingObs) return false;

    // search each of the bb 4 sides for collisions with path
    std::vector<std::pair<int, int>> side_idxs = {{0, 2},
                                                {0, 1},
                                                {3, 2},
                                                {3, 1}};
    int vx_idx1, vx_idx2;
    for (const auto &side_idx: side_idxs) {
        vx_idx1 = side_idx.first;
        vx_idx2 = side_idx.second;

        auto vx1_pos = get3DVector(vx_idx1);
        auto vx2_pos = get3DVector(vx_idx2);

        // if the path starts from the current obs, do not check the sides of the departing vx.
        //  still, the diagonals of start.obs will be checked for collision
        if (isDepartingObs && (start->data.vx == vx_idx1 || start->data.vx == vx_idx2)) {
            continue;
        }
        
        Eigen::Vector3d p1 = {start->data.position.x(), start->data.position.y(), 0};
        Eigen::Vector3d p2 = {goal->data.position.x(), goal->data.position.y(), goal->data.time.count() - start->data.time.count()};
        Eigen::Vector3d planeNormal = obs_velocity.cross(vx1_pos - vx2_pos);
        Eigen::Vector3d cp;
        bool collision = FindLinePlaneIntersection(p1, p2, vx1_pos, planeNormal, cp);
        if(collision){
            // Get vertexes at time t'
            Eigen::Vector3d vertex1_position = vx1_pos + obs_velocity * cp.z();
            Eigen::Vector3d vertex2_position = vx2_pos + obs_velocity * cp.z();
            // Check if point is inside those two vertexes
            Eigen::Vector3d P1 = cp - vertex1_position;
            Eigen::Vector3d P2 = cp - vertex2_position;
            if (P1.dot(P2) <= 0) {
                collisions.push_back(cp);
                return true;
            }
        }
            
    }

    // check the diagonal opposite to departing vx
    if (isDepartingObs) {
        // check the opposite diagonal wrt the start vx
        if (start->data.vx == FL || start->data.vx == RR) {
            vx_idx1 = FR;
            vx_idx2 = RL;
        } else {
            vx_idx1 = FL;
            vx_idx2 = RR;
        }

        Eigen::Vector3d p1 = {start->data.position.x(), start->data.position.y(), 0};
        Eigen::Vector3d p2 = {goal->data.position.x(), goal->data.position.y(), goal->data.time.count() - start->data.time.count()};
        Eigen::Vector3d planeNormal = obs_velocity.cross(get3DVector(vx_idx1) - get3DVector(vx_idx2));
        Eigen::Vector3d cp;
        if(FindLinePlaneIntersection(p1, p2, get3DVector(vx_idx1), planeNormal, cp)){
            collisions.push_back(cp);
            return true;
        }
        
    }


    return false;
}