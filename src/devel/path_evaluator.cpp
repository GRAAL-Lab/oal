#include "oal/devel/path_evaluator.hpp"

bool oal::PathEvaluator::RuleCompliantMotion(const NodePtr& start, NodePtr& goal) {
    // Check only if colregs compliance is requested
    if (goal->data.obs_ptr->Velocity().norm() <= ZERO_VELOCITY) return true;
    

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
    if (abs(theta) <= HeadOnAngle) {
        // head on
        if (goal->data.vx == FR || goal->data.vx == RR) {
            // should be on left
            return false;
        }
        //goal.currentObsLimitedVxs.push_back(FR);
    } else {
        if (abs(theta) >= OvertakingAngle) {
            // overtaking
            // if (goal.data.vx == FR) {
            //     goal.currentObsLimitedVxs.push_back(RR);
            //     goal.currentObsLimitedVxs.push_back(FL);
            // }
            // if (goal.data.vx == FL) {
            //     goal.currentObsLimitedVxs.push_back(FR);
            // }
            //goal.overtakingObsList.push_back(goal.obs_ptr->id);
            // avoid future crossings
        } else {
            if (theta < 0) {
                // crossing from left, give way
                if (goal->data.vx == FR || goal->data.vx == FL) {
                    // should give way
                    return false;
                }
                // goal.currentObsLimitedVxs.push_back(FR);
                // goal.currentObsLimitedVxs.push_back(FL);
            } else {
                if (theta > 0) {
                    // crossing from right, stand on but
                    //  if execution comes here, obs is high priority,
                    //  so give way but avoid rear vxs
                    //goal.currentObsLimitedVxs.push_back(RR);  // avoid the maneuver to become a head on
                    if (goal->data.vx == RR || goal->data.vx == RL) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

bool oal::PathEvaluator::HasHigherPriority(std::string obsClass){
    // Check if the obstacle has higher priority than own ship
    std::cerr<<"[WARNING] HasHigherPriority not implemented"<<std::endl;
    return false;
}

bool oal::PathEvaluator::CollisionWithObs(const NodePtr& start, const NodePtr& goal, const ObsPtr& obs, bool colregs){
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
        if (theta > HeadOnAngle && theta < OvertakingAngle && !HasHigherPriority(obs->ObsClass()) &&
            obs->Velocity().norm() > ZERO_VELOCITY) {
            // crossing from right, stand on
            return false;
        }
    }


    auto get3DVector = [& vxs] (int index) {
        return Eigen::Vector3d(vxs[index].second.x(), vxs[index].second.y(), 0.0);
    };

    // Starts in TS bb and does NOT go to same obs
    //if (isDepartingObs && start.vx == NA) {
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
                return true;
            }
        }
        return false;
    }

    // Do not check the goal obstacle: the algorithm already aims for the visible vxs
    // This condition goes here because an obs could be both departing and approaching obs
    if (isApproachingObs) return false;

    // search each of the bb 4 sides for collisions with path
    std::vector<std::vector<int>> side_idxs = {{0, 2},
                                                {0, 1},
                                                {3, 2},
                                                {3, 1}};
    int vx_idx1, vx_idx2;
    for (const auto &side_idx: side_idxs) {
        vx_idx1 = side_idx.at(0);
        vx_idx2 = side_idx.at(1);

        // if the path starts from the current obs, do not check the sides of the departing vx.
        //  still, the diagonals of start.obs will be checked for collision
        if (isDepartingObs && (start->data.vx == vx_idx1 || start->data.vx == vx_idx2)) {
            return false;
        }
        
        Eigen::Vector3d p1 = {start->data.position.x(), start->data.position.y(), 0};
        Eigen::Vector3d p2 = {goal->data.position.x(), goal->data.position.y(), goal->data.time.count() - start->data.time.count()};
        Eigen::Vector3d planeNormal = obs_velocity.cross(get3DVector(vx_idx1) - get3DVector(vx_idx2));
        Eigen::Vector3d cp;
        bool collision = FindLinePlaneIntersection(p1, p2, get3DVector(vx_idx1), planeNormal, cp);
        if(collision){
            // The desired path crosses the face of the bb defined by the two vxs and its direction
            // if (collision_points != nullptr) {
            //     // optional argument is given, append collision point
            //     Node cp_node;
            //     Eigen::Vector2d collision_point_2d(collision_point.x(), collision_point.y());
            //     cp_node.position = collision_point_2d;
            //     cp_node.obs_ptr = obs_ptr;
            //     cp_node.vx = NA;
            //     cp_node.time = std::chrono::duration<double>(start.time.count() + collision_point.z());
            //     collision_points->push_back(cp_node);
            return true;
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
            return true;
        }
        
    }


    return false;
}