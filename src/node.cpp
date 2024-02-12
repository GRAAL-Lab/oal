#include "oal/data_structs/node.hpp"
#include "oal/helper_functions.hpp"

void Node::UpdateCosts(const Eigen::Vector2d &goal, double highest_speed, double rot_speed) {
  costToReach = time; //if the cost is the time to reach the target
  Eigen::Vector2d dist_to_goal = goal - position;
  costToGoal = dist_to_goal.norm() / highest_speed;

  double rotation_wasted_time = 0;
  if(rot_speed>0){
    // The time contribution of a node is also:
    //    - how much it takes to change course to reach this node
    //    - how much it takes to change course to reach goal from this node
    Eigen::Vector2d t1 = goal - position;
    rotation_wasted_time = (GetHeadingChange() + std::acos(t1.normalized().dot(Eigen::Vector2d(1,0)))) / rot_speed;
  }

  costTotal = costToReach + costToGoal + rotation_wasted_time;
}

void Node::GetCloser(const std::vector<Node> &nodes_list, Node &closer) const {
  if (nodes_list.empty()) {
    closer = *this;
  } else {
    closer = nodes_list.at(0);
    Eigen::Vector2d dist_v = closer.position - position;
    double min_distance = dist_v.norm();
    for (const auto &node: nodes_list) {
      dist_v = node.position - position;
      double dist = dist_v.norm();
      if (dist < min_distance) {
        closer = node;
        min_distance = dist;
      }
    }
  }
}

bool Node::IsUnique(Node other) {
  // TODO uniqueness should be checked when looking for interceptable vxs
  while (other.parent != nullptr) {
    if (/*node_pos == other_pos && */obs_ptr.get() == other.obs_ptr.get() && vx == other.vx) {
      return false;
    }
    other = *other.parent;
  }
  return true;
}

void Node::FindVisibilityVxs(Obstacle target_obs, std::vector<Vertex> &vxs_abs) {
  if (obs_ptr != nullptr) {
    // Ownship on a bb
    if (obs_ptr->id == target_obs.id) {
      // Ownship already on the target bb
      if (vx == NA){
        // OS in TS bb, visible are the exit ones
        std::vector<vx_id> allowedVxs;
        FindExitVxs(allowedVxs);
        vxs_abs[allowedVxs[0]].isVisible = true;
        vxs_abs[allowedVxs[1]].isVisible = true;
      } else {
        // Set the adjacent vxs as visible
        if (vx == FR || vx == RL) {
          vxs_abs[1].isVisible = true;
          vxs_abs[2].isVisible = true;
        } else {
          vxs_abs[0].isVisible = true;
          vxs_abs[3].isVisible = true;
        }
      }
      return;
    }
  }

  // Find the angles between vectors from vehicle to each vxs and the vector from vehicle to the obs center,
  //  then the visible vxs are the ones with min/max angle + the ones closer to the vehicle than these two
  Eigen::Vector2d obs_position = ComputePosition(target_obs, time);
  Eigen::Vector2d obs_vh = obs_position - position;

  std::vector<double> thetas;  // angles wrt abs frame
  std::vector<Eigen::Vector2d> vxs_vh;

  for (Vertex &vert: vxs_abs) {
    Eigen::Vector2d vx_vh = vert.position - position; //if 0 there's a problem
    double sign = obs_vh.x() * vx_vh.y() -
                  obs_vh.y() * vx_vh.x(); // same as z value of a cross product (and so its direction)
    double theta = sign / abs(sign) * acos(obs_vh.normalized().dot(vx_vh.normalized()));
    thetas.push_back(theta);
    vxs_vh.push_back(vx_vh);
  }
  auto max_v_ptr = std::max_element(thetas.begin(), thetas.end());
  auto max_v = std::distance(thetas.begin(), max_v_ptr);
  auto min_v_ptr = std::min_element(thetas.begin(), thetas.end());
  auto min_v = std::distance(thetas.begin(), min_v_ptr);
  // min/max angle vertexes are visible
  vxs_abs[min_v].isVisible = true;
  vxs_abs[max_v].isVisible = true;
  // are also visible vertexes which distance is smaller than those two
  double distance, min_v_distance, max_v_distance;
  min_v_distance = vxs_vh[min_v].norm();
  max_v_distance = vxs_vh[max_v].norm();
  for (size_t i = 0; i < vxs_vh.size(); ++i) {
    if (i == min_v || i == max_v) {
      continue;
    }
    distance = vxs_vh[i].norm();
    if (distance < min_v_distance && distance < max_v_distance) {
      vxs_abs[i].isVisible = true;
    }
  }
}

/*bool Node::RemoveWorstDuplicates(std::multiset<Node> &set) {
  return true;
  for (auto node_it = set.begin(); node_it != set.end();) {
    bool isSimilar = ((position-node_it->position).norm()<0.01 && obs_ptr.get() == node_it->obs_ptr.get() && vx == node_it->vx);
    if (isSimilar) {
      hasSimilar = true;
      std::cout << " SIMILAR: " << std::endl;
      node_it->print();
      this->print();
      std::cout << "___" << std::endl;

      // Alternative costs, take advantage of nodes redundancy
      bool minAncestors = mtrcs.n_ancestors < node_it->mtrcs.n_ancestors;
      bool minTotHeadingChange = mtrcs.totHeadingChange < node_it->mtrcs.totHeadingChange;
      bool minMaxHeadingChange = mtrcs.maxHeadingChange < node_it->mtrcs.maxHeadingChange;
      bool minMaxSpeed = mtrcs.maxSpeed < node_it->mtrcs.maxSpeed;
      bool minMaxAcceleration = mtrcs.maxSpeedChange < node_it->mtrcs.maxSpeedChange;
      bool minDistance = mtrcs.totDistance < node_it->mtrcs.totDistance;
      bool minAverageSpeed = mtrcs.averageSpeed < node_it->mtrcs.averageSpeed;

      if (minTotHeadingChange) {
        // *this is better, remove *node_it and everyone that has ancestor *node_it

        // delete also descendents
        for (auto node_it2 = set.begin(); node_it2 != set.end();) {
          if (node_it2->HasAncestor(*node_it)) {
            node_it2 = set.erase(node_it2);
          } else {
            ++node_it2;
          }
        }
        node_it = set.erase(node_it);
        return true;
      }else {
        return false;
      }
    }else{
      ++node_it;
    }
  }
  return true;
}*/

void Node::FindExitVxs(std::vector<vx_id> &allowedVxs) const {
  Obstacle obs = *obs_ptr;
  Eigen::Vector2d bodyObs_e = GetProjectionInObsFrame(position, obs, 0.0);

  bool IsLeftOfDiag1 = (bodyObs_e.y() >=
                        obs.vxs[1].position.y() / obs.vxs[1].position.x() *
                        bodyObs_e.x()); // Being left of diag FL-RR
  bool IsLeftOfDiag2 = (bodyObs_e.y() >=
                        obs.vxs[0].position.y() / obs.vxs[0].position.x() *
                        bodyObs_e.x()); // Being left of diag FR-LL

  if (IsLeftOfDiag1 && IsLeftOfDiag2) {
    // USV in port side of bb
    //  go for FL or RL
    allowedVxs.push_back(FL);
    allowedVxs.push_back(RL);
  } else {
    if (IsLeftOfDiag1) {
      // USV in stern side of bb
      //  go for RL or RR
      allowedVxs.push_back(RL);
      allowedVxs.push_back(RR);
    } else {
      if (IsLeftOfDiag2) {
        // USV in bow side of bb
        //  go for FR or FL
        allowedVxs.push_back(FR);
        allowedVxs.push_back(FL);
      } else {
        // USV in starboard side of bb
        //  go for FR or RR
        allowedVxs.push_back(FR);
        allowedVxs.push_back(RR);
      }
    }
  }
}

/*bool Node::HasAncestor(const Node &node) const {
  if(this->parent != nullptr){
    Node current = *this->parent;
    while (current.parent != nullptr) {
      if (current == node) {
        return true;
      }
      current = *current.parent;
    }
  }
  return false;
}*/






