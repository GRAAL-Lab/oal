#include "oal/path_planner.hpp"

// Compute the path to reach the goal
bool path_planner::ComputePath(const Eigen::Vector2d &goal_position, bool colregs, Path &path) {
  path = Path();
  colregs_compliance = colregs;
  // Plot stuff
  {
    if (plotWpsFile_.is_open()) {
      plotWpsFile_.close();
    }
    if (colregs_compliance) {
      plotWpsFile_.open("WPlogC.txt", std::ofstream::trunc);
    } else {
      plotWpsFile_.open("WPlogN.txt", std::ofstream::trunc);
    }
    plotWpsFile_ << std::setprecision(4);
    plotWpsFile_ << "Start_" << v_info_.position(0) << "_" << v_info_.position(1) << std::endl;
    plotWpsFile_ << "Goal_" << goal_position(0) << "_" << goal_position(1) << std::endl;
  }

  std::multiset<Node> open_set;  // set of reachable nodes, ordered by total cost (ascending), still to analyze
  std::multiset<Node> reachable_full_set; // set of all the reachable nodes (open_set + closed set)
  Node goal, current;
  // Goal node setup
  goal.position = goal_position;
  // Set obss local vxs
  FindObssLocalVxs(true);
  // Plot stuff
  {
    plotWpsFile_ << "Time_" << 0 << std::endl;
    plotWpsFile_ << "Waypoint_" << v_info_.position.x() << "_" << v_info_.position.y() << std::endl;
    for (const obs_ptr &obs_ptr: obss_info_.obstacles) {
      plotWpsFile_ << obs_ptr->plotStuff(0);
    }
  }

  // Root node setup
  if (!RootSetup(goal_position, open_set)) return false;
  // Initialize reachable with root
  reachable_full_set = open_set;

  // Run until every node is studied or a path is found
  bool found = false;
  while (!open_set.empty()) {
    current = *open_set.begin();
    open_set.erase(open_set.begin());

    if (open_set.size() > 50000 && (open_set.size() % 10000 == 0)) {
      std::cout << "Number of nodes to analyze: " << open_set.size() << std::endl;
      /*path.debug_flag = true;
      found = true;
      return true;*/
    }

    // Check if goal is reachable
    if (CheckFinal(current, goal, open_set, reachable_full_set)) {
      found = true;
      break;
    }

    for (const obs_ptr &obs_ptr: obss_info_.obstacles) {
      std::vector<Vertex> visible_vxs, reachable_vxs;
      Obstacle obs = *obs_ptr.get();
      obs.FindAbsVxs(current.time, visible_vxs); //Find vertexes in abs frame
      current.FindVisibilityVxs(obs, visible_vxs);
      for (double &speed: v_info_.velocities) {
        FindInterceptPoints(current, obs, speed, visible_vxs,
                            reachable_vxs); //Compute when every visible vertex is intercepted
      }
      // Check collision for each of the visible and intercept-able vxs
      for (const Vertex &vx: reachable_vxs) {
        // Create new Node
        Node new_node(vx.intercept_point, obs_ptr, vx.id, current);
        new_node.speed_to_it = vx.intercept_speed;
        if (CheckColreg(current, new_node)) {
          // The path to vx respects the colregs (if it needs to)
          if (CheckCollision(current, new_node)) {
            // Collision free
            if (!new_node.IsUnique(current)) {
              // Not unique means it is returning on a vertex already visited.
              continue;
            }
            // Save new node
            new_node.UpdateCosts(goal_position, GetHighestSpeed(), v_info_.rot_speed);
            open_set.insert(new_node);
            reachable_full_set.insert(new_node);
            /* if (new_node.RemoveWorstDuplicates(reachable_full_set)) {
               // new_node is unique: either always been or it was better than similar and so the other was trashed
               //     otherwise discarded because better one already analyzed
               new_node.RemoveWorstDuplicates(open_set); // update open_set too, outcome already known
               // Save new node
               for (double &speed: v_info_.velocities) {
                 // Every node will expand with the different velocities
                 new_node.vh_speed = speed;
                 new_node.UpdateCosts(goal_position);
                 open_set.insert(new_node);
                 reachable_full_set.insert(new_node);
               }
             }*/
          }
        }
      }
    }
  }

  if (!found || current.time == 0) {
    // No path found OR GOAL == START
    if (current.parent == nullptr) return false; // still in start

    std::cout << " oal search did not find any path, returning best estimate if any " << std::endl;
    // Find the node with the smallest est. time to goal among the analyzed ones
    Node best_goal = current;
    while (!reachable_full_set.empty()) {
      Node node = *reachable_full_set.begin();
      reachable_full_set.erase(reachable_full_set.begin());
      if (node.costToGoal < best_goal.costToGoal) {
        best_goal = node;
      }
    }

    if ((best_goal.position - v_info_.position).norm() < acceptanceRadius) return false; // new goal is starting point
    current = best_goal;
  }

  //current.print();
  BuildPath(current, path);

  //path.overtakingObsList = current.overtakingObsList;
  //std::cout << "count: " << current.parent.use_count() << std::endl;
  return true;
}

bool path_planner::CheckFinal(const Node &start, Node goal, std::multiset<Node> &open_set,
                              std::multiset<Node> &reachable_full_set) {
  // True if start == goal
  if ((goal.position - start.position).norm() <= acceptanceRadius || start.is_final) {
    return true;
  }

  for (double &speed: v_info_.velocities) {
    goal.speed_to_it = speed;
    std::shared_ptr<std::vector<obs_ptr>> surrounding_obs(new std::vector<obs_ptr>);
    goal.time = start.time + (goal.position - start.position).norm() / goal.speed_to_it;
    // Check if goal is unreachable just for it being in one bb (this, for every possible velocity)
    if (IsInAnyBB({goal.position, goal.time}, surrounding_obs)) {
      // the goal is unreachable, let's try and find a new goal.
      std::shared_ptr<std::vector<Node>> collision_nodes(new std::vector<Node>);
      CheckCollision(start, goal, collision_nodes); //run check collision to get the collision points
      // The new goal might be the closer collision point iff it is a surrounding obs
      // otherwise there's an actual collision before getting to that goal
      // closer collision, maybe move it in checkCollision
      Node closer_collision;
      start.GetCloser(*collision_nodes, closer_collision);
      for (const auto &s_obs_ptr: *surrounding_obs) {
        // If closer collision is with any of the surrounding obs
        if (s_obs_ptr.get() == closer_collision.obs_ptr.get()) {
          // and if this new point is worth it (not too close)
          if ((closer_collision.position - start.position).norm() > acceptanceRadius) {
            // Add a new goal which can be the final one
            Node new_node(TPoint({closer_collision.position, closer_collision.time}), s_obs_ptr, NA, start);
            new_node.speed_to_it = speed;
            new_node.is_final = true;
            new_node.UpdateCosts(goal.position, GetHighestSpeed(), v_info_.rot_speed);
            open_set.insert(new_node);
            reachable_full_set.insert(new_node);
            break;
          }
        }
      }
    } else {
      if (CheckCollision(start, goal)) {
        // From start with current speed, goal is reachable => add goal as node in open_set
        // TODO actually if speed is highest when this condition is verified, then this is best option, maybe optimize it
        goal.SetParent(start);
        goal.UpdateCosts(goal.position, GetHighestSpeed(), v_info_.rot_speed);
        open_set.insert(goal);
        reachable_full_set.insert(goal);
      }
    }
  }
  return false;
}


bool path_planner::CheckColreg(const Node &start, Node &goal) const {
  // Check only if colregs compliance is requested
  if (!colregs_compliance || goal.obs_ptr->speed <= 0.01) {
    return true;
  }

  /*auto it = std::find(goal.overtakingObsList.begin(), goal.overtakingObsList.end(), goal.obs_ptr->id);
  if( it != goal.overtakingObsList.end()){
    // Moving to overtaking obs
  }*/

  // Check only if moving between different obstacles
  if (start.obs_ptr.get() == goal.obs_ptr.get()) {
    // otherwise is compliant iff goal vx is not limited because of past maneuver
    for (const vx_id &vx: start.currentObsLimitedVxs) {
      if (vx == goal.vx) return false;
    }
    return true;
  }
  // Get the approaching angle of own ship wrt target ship
  Eigen::Vector2d path = goal.position - start.position;
  double theta = GetBearing(path, goal.obs_ptr->head);
  // Check colregs depending on situation
  if (abs(theta) <= HeadOnAngle) {
    // head on
    if (goal.vx == FR || goal.vx == RR) {
      // should be on left
      return false;
    }
    goal.currentObsLimitedVxs.push_back(FR);
  } else {
    if (abs(theta) >= OvertakingAngle) {
      // overtaking
      if (goal.vx == FR) {
        goal.currentObsLimitedVxs.push_back(RR);
      }
      //goal.overtakingObsList.push_back(goal.obs_ptr->id);
      // avoid future crossings
    } else {
      if (theta < 0) {
        // crossing from left, give way
        if (goal.vx == FR || goal.vx == FL) {
          // should give way
          return false;
        }
        goal.currentObsLimitedVxs.push_back(FR);
        goal.currentObsLimitedVxs.push_back(FL);
      } else {
        if (theta > 0) {
          // crossing from right, stand on but
          //  if execution comes here, obs is high priority,
          //  so give way but avoid rear vxs
          goal.currentObsLimitedVxs.push_back(RR);  // avoid the maneuver to become a head on
          if (goal.vx == RR || goal.vx == RL) {
            return false;
          }
        }
      }
    }
  }
  return true;
}

// Check if the path between start and goal collide with any obstacle
bool path_planner::CheckCollision(const Node &start, Node &goal,
                                  const std::shared_ptr<std::vector<Node>> &collision_points) {
  /* Function returns true if path between start and goal crosses a bb.
   *  Special situations arise when start and goal points belongs to obstacles bb:
   *    if leaving from INSIDE an obs,
   *      checks path do not cross any diagonal, ensuring no more damage can be done.
   *    if going to an obs,
   *      just check collision with every other obstacles since goal on goal_obs is safe by definition
   *    if leaving from an obs,
   *      do not check collision with start_obs sides with start_vx as one of the vx.
   *        (goal not being inside a bb ensure path won't cross such sides)
   *      check opposed diagonal for collision.
   *        (it can go around an obs only by moving between consecutive vxs)
  */
  Eigen::Vector3d start3d = Get3dPos(start);
  Eigen::Vector3d goal3d = Get3dPos(goal);
  // vehicle path to check for collision
  Eigen::Vector3d path = goal3d - start3d;

  // Plot stuff
  {
    plotCKFile_ << "---" << std::endl;
    plotCKFile_ << "Start_" << start3d(0) << "_" << start3d(1) << "_" << start3d(2) << std::endl;
    plotCKFile_ << "Goal_" << goal3d(0) << "_" << goal3d(1) << "_" << path.norm() / goal.speed_to_it << std::endl;
  }

  for (const obs_ptr &obs_ptr: obss_info_.obstacles) {
    Obstacle obs = *obs_ptr.get();
    // Find vertexes in abs frame
    std::vector<Vertex> vxs_abs;
    obs.FindAbsVxs(start.time, vxs_abs);
    // Compute obstacle direction in x-y-time
    Eigen::Vector3d bb_direction(obs.speed * cos(obs.vel_dir), obs.speed * sin(obs.vel_dir), 1);
    //bb_direction.normalize(); no, because multiplied by t' it returns the position at time t'

    bool isDepartingObs = obs_ptr.get() == start.obs_ptr.get();
    bool isApproachingObs = obs_ptr.get() == goal.obs_ptr.get();

    // Plot stuff
    {
      plotCKFile_ << "Obs_" << obs.id << std::endl;
      // Uncomment to plot
      plotCKFile_ << "PlotIt" << std::endl;
      plotCKFile_ << "Direction_" << bb_direction(0) << "_" << bb_direction(1) << "_" << bb_direction(2) << std::endl;
      for (Vertex vx: vxs_abs) {
        plotCKFile_ << "Vx_" << vx.position.x() << "_" << vx.position.y() << std::endl;
      }
      plotCKFile_ << "-" << std::endl;
    }

    // Colregs check (ignore crossing from right TS)
    if (colregs_compliance) {
      // Angle between
      double theta = GetBearing(Eigen::Vector2d(path.x(), path.y()), obs.head);
      if (theta > HeadOnAngle && theta < OvertakingAngle && !obs_ptr->higher_priority &&
          obs_ptr->speed > OBSTACLE_STATIC_SPEED_THRESHOLD) {
        // crossing from right, stand on
        goal.ignoring_obs_debug = true;
        continue;
      }
    }

    Eigen::Vector3d collision_point;

    // Starts in TS bb and does NOT go to same obs
    //if (isDepartingObs && start.vx == NA) {
    if (isDepartingObs && start.vx == NA) {
      if (!isApproachingObs) {
        // Check both bb diagonals for collisions
        if (FindLinePlaneIntersectionPoint(vxs_abs[FR], vxs_abs[RL], bb_direction, start, goal, collision_point)
            || FindLinePlaneIntersectionPoint(vxs_abs[FL], vxs_abs[RR], bb_direction, start, goal, collision_point)) {
          return false;
        }
      }
      continue;
    }

    // Do not check the goal obstacle: the algorithm already aims for the visible vxs
    // This condition goes here because an obs could be both departing and approaching obs
    if (isApproachingObs) continue;

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
      if (isDepartingObs && (start.vx == vx_idx1 || start.vx == vx_idx2)) {
        continue;
      }

      if (FindLinePlaneIntersectionPoint(vxs_abs[vx_idx1], vxs_abs[vx_idx2], bb_direction, start, goal,
                                         collision_point)) {
        // The desired path crosses the face of the bb defined by the two vxs and its direction
        if (collision_points != nullptr) {
          // optional argument is given, append collision point
          Node cp_node;
          Eigen::Vector2d collision_point_2d(collision_point.x(), collision_point.y());
          cp_node.position = collision_point_2d;
          cp_node.obs_ptr = obs_ptr;
          cp_node.vx = NA;
          cp_node.time = start.time + collision_point.z();
          collision_points->push_back(cp_node);
        }
        return false;
      }
    }

    // check the diagonal opposite to departing vx
    if (isDepartingObs) {
      // check the opposite diagonal wrt the start vx
      if (start.vx == FL || start.vx == RR) {
        vx_idx1 = FR;
        vx_idx2 = RL;
      } else {
        vx_idx1 = FL;
        vx_idx2 = RR;
      }

      if (FindLinePlaneIntersectionPoint(vxs_abs[vx_idx1], vxs_abs[vx_idx2], bb_direction, start, goal,
                                         collision_point)) {
        return false;
      }
    }
  }
  plotCKFile_ << "Good" << std::endl;
  return true;
}

bool path_planner::FindLinePlaneIntersectionPoint(Vertex vx1, Vertex vx2, const Eigen::Vector3d &bb_direction,
                                                  const Node &start, const Node &goal,
                                                  Eigen::Vector3d &collision_point) {

  Eigen::Vector3d goal3d = Get3dPos(goal);
  Eigen::Vector3d path = goal3d - Get3dPos(start);
  goal3d[2] = path.norm() / goal.speed_to_it;
  path = goal3d - Get3dPos(start);
  Eigen::Vector3d vx1_pos = Get3dPos(vx1);
  Eigen::Vector3d vx2_pos = Get3dPos(vx2);

  // Get the plane normal
  Eigen::Vector3d side = vx1_pos - vx2_pos;
  Eigen::Vector3d planeNormal = bb_direction.cross(side);
  planeNormal.normalize();

  // if the plane and the path are //
  if (planeNormal.dot(path) == 0) return false; //do we need tolerance here?

  // Compute position along the path where the collision happens (from http://paulbourke.net/geometry/pointlineplane/)
  Eigen::Vector3d startToOne = vx1_pos - Get3dPos(start);

  double c_pos = planeNormal.dot(startToOne) / planeNormal.dot(path);
  c_pos = std::round(c_pos * 1000.0) / 1000.0;

  if (c_pos > 0 && c_pos <= 1) { // intersect_point between start and goal
    // there's the equals since the function does NOT care for the starting vx sides and the goal obs
    // LOOK, NOW IS >0 INSTEAD OF >=0, IS IT COOL?
    // Get collision time and point
    Eigen::Vector3d doable_path = path * c_pos; // Part of the path prior the collision
    doable_path.z() = 0;
    double collision_time = doable_path.norm() / goal.speed_to_it; // When the collision happens
    doable_path.z() = collision_time;
    collision_point = Get3dPos(start) + doable_path; // Where the collision happens
    // Get vertexes at time t'
    Eigen::Vector3d vertex1_position = vx1_pos + bb_direction * collision_time;
    Eigen::Vector3d vertex2_position = vx2_pos + bb_direction * collision_time;
    // Check if point is inside those two vertexes
    Eigen::Vector3d P1 = collision_point - vertex1_position;
    Eigen::Vector3d P2 = collision_point - vertex2_position;
    if (P1.dot(P2) <= 0) return true;
  }
  return false;
}

void path_planner::BuildPath(Node &current, Path &path) {    // Build path from goal to initial position
  /*std::cout<<" Length: "<<path.metrics.totDistance<<" meters\n"
           <<" Total course change: "<<path.metrics.totHeadingChange<<" radians\n"
           <<" Max course change: "<<path.metrics.maxHeadingChange<<" radians"<<std::endl;*/

  // goal is supposed to have at least one root
  path.waypoints.push(current);
  {
    plotWpsFile_ << "Time_" << current.time << std::endl;
    plotWpsFile_ << "Waypoint_" << current.position(0) << "_" << current.position(1) << std::endl;
    for (const obs_ptr &obs_ptr: obss_info_.obstacles) {
      plotWpsFile_ << obs_ptr->plotStuff(current.time);
      plotWpsFile_.flush();
    }
  }
  std::shared_ptr<Node> ptr = current.parent;

  do {
    current = *ptr;
    path.waypoints.push(current);
    //  Plot stuff
    {
      plotWpsFile_ << "Time_" << current.time << std::endl;
      plotWpsFile_ << "Waypoint_" << current.position(0) << "_" << current.position(1) << std::endl;
      for (const obs_ptr &obs_ptr: obss_info_.obstacles) {
        plotWpsFile_ << obs_ptr->plotStuff(current.time);
        plotWpsFile_.flush();
      }
    }
    ptr = current.parent;
  } while (ptr != nullptr);

  // update path metrics in oal_interface
}

// Given some obstacle vertexes, find the intercept points
void path_planner::FindInterceptPoints(const Node &start, Obstacle &obstacle, double vh_speed,
                                       std::vector<Vertex> visible_vxs, std::vector<Vertex> &reachable_vxs) {
  //std::vector<Vertex> vxs;
  Eigen::Vector3d bb_timeDirection(obstacle.speed * cos(obstacle.vel_dir), obstacle.speed * sin(obstacle.vel_dir), 1);
  for (auto i = 0; i < 4; i++) {
    if (visible_vxs[i].isVisible) {
      std::vector<double> t_instants;
      Eigen::Vector2d vertex_vehicle = visible_vxs[i].position - start.position;
      if (obstacle.speed < 0.001) { // TODO choose value
        t_instants.push_back(vertex_vehicle.norm() / vh_speed);
      } else {
        double det = pow(vh_speed, 2) + 1;
        double c2 = 1 - (pow(bb_timeDirection.x(), 2) + pow(bb_timeDirection.y(), 2) + 1) / det;
        double c1 = -(bb_timeDirection.x() * vertex_vehicle.x() + bb_timeDirection.y() * vertex_vehicle.y()) / det;
        double c0 = -(pow(vertex_vehicle.x(), 2) + pow(vertex_vehicle.y(), 2)) / det;
        double delta = pow(c1, 2) - c0 * c2;
        if (abs(c2) > 0.001 && delta >= 0) {
          // 2 points
          double t1 = (-c1 + sqrt(delta)) / c2;
          double t2 = (-c1 - sqrt(delta)) / c2;
          if (t1 > 0) t_instants.push_back(t1);
          if (t2 > 0 && abs(t1 - t2) > 0.001) t_instants.push_back(t2);
        } else {
          if (abs(c2) <= 0.001 && abs(c1) > 0.001) {
            // 1 point
            double t = -c0 / (2 * c1);
            if (t > 0) t_instants.push_back(t);
          } else {
            // no points
            continue;
          }
        }
      }
      std::vector<Eigen::Vector3d> points;
      for (double t: t_instants) {
        Eigen::Vector3d point_3d = Get3dPos(visible_vxs[i]) + bb_timeDirection * t;
        TPoint intercept_point;
        intercept_point.pos.x() = point_3d.x();
        intercept_point.pos.y() = point_3d.y();
        intercept_point.time = point_3d.z();
        std::shared_ptr<std::vector<obs_ptr>> surrounding_obs(new std::vector<obs_ptr>);
        bool isInAnyBB = IsInAnyBB(intercept_point, surrounding_obs);
        if (isInAnyBB && surrounding_obs->size() == 1 && surrounding_obs->at(0)->id == obstacle.id) {
          // if vx is in its own bb, ignore
          isInAnyBB = false;
        }
        // Save the point
        if (!isInAnyBB) {
          Vertex vx = visible_vxs[i];
          vx.intercept_point = intercept_point;
          vx.intercept_speed = vh_speed;
          reachable_vxs.push_back(vx);
        }
      }
    }
  }
  //vxs_abs = vxs;
}

bool path_planner::IsInAnyBB(TPoint time_point,
                             const std::shared_ptr<std::vector<obs_ptr>> &surrounding_obs) {
  for (const obs_ptr &obs_ptr: obss_info_.obstacles) {
    if (obs_ptr->IsInBB(time_point)) {
      if (surrounding_obs != nullptr) {
        surrounding_obs->push_back(obs_ptr);
      }
      return true;
    }
  }
  return false;
}

bool path_planner::CheckPath(const Eigen::Vector2d &vh_pos, Path path) {
  // the waypoint should be just the ones left to reach, not the whole path returned by the library
  if (path.empty()) {
    std::cout << " PATH IS EMPTY -> CHECK = FALSE" << std::endl;
    return false;
  }

  // Update vxs position based on ownship position (without uncertainty gap)
  FindObssLocalVxs(false);

  Node start;
  start.position = vh_pos;
  start.time = 0;
  // TODO temptative
  start.obs_ptr = path.top().obs_ptr;
  start.vx = path.top().vx;

  Path whole_path = path;// to print later

  path.pop();

  //colregs_compliance = false; // should not change the value, otherwise ignoring obs (giving way) would be threat
  while (!path.empty()) {
    Node goal = path.top();
    goal.speed_to_it = path.top().speed_to_it;
    path.pop();
    if (!CheckCollision(start, goal)) {
      std::cout << " COLLISION WHILE CHECKING PATH: " << std::endl;
      whole_path.print(true);
      std::cout << " collision in checked path between (" << start.position.x() << ", " << start.position.y()
                << ") and (" << goal.position.x() << ", " << goal.position.y() << ") " << std::endl;
      return false;
    }

    start = Node(goal); // TODO check =operator definition, or if Node(other) is fine
  }
  return true;
}

bool path_planner::RootSetup(const Eigen::Vector2d &goal_position, std::multiset<Node> &open_set) {
  std::shared_ptr<std::vector<obs_ptr>> surrounding_obs(new std::vector<obs_ptr>);
  IsInAnyBB({v_info_.position, 0}, surrounding_obs);
  Obstacle surr_obs;
  Node root;
  root.position = v_info_.position;
  root.time = 0;
  if (!surrounding_obs->empty()) {
    if (surrounding_obs->size() > 1) {
      // The starting point is in a number of different bb, at the moment no solution
      std::cout << "Start in multiple bb.. abort." << std::endl;
      return false;
    }
    root.obs_ptr = surrounding_obs->at(0);
    root.vx = NA;
    // TODO reposition this piece of code
    // Check which Ship has to give way
    // TODO here colregs compliance is always assumed, change it (?)
    /*Eigen::Vector2d BodyObs_vh = GetProjectionInObsFrame(v_info_.position, surr_obs, 0);
    if(BodyObs_vh.x()>0 && atan2(BodyObs_vh.y(),BodyObs_vh.x()) <= M_PI - OvertakingAngle){
      // Own ship is in a bb of a vessel coming from behind: it is that one that should avoid os
      // -> keep the original starting node OR stand on and wait TODO for now just return path not found
      std::cout << std::endl<<" Target ships colliding with own ship.. abort." << std::endl;
      return false;
    }*/
  }
  root.starting_heading = v_info_.heading;
  root.UpdateCosts(goal_position, GetHighestSpeed(), v_info_.rot_speed);
  open_set.insert(root);
  return true;
}

void path_planner::FindObssLocalVxs(bool with_uncertainty) {
  // Set the bounding box dimension based on the vehicle start distance from the obstacles
  for (const obs_ptr &obs: obss_info_.obstacles) {
    obs->uncertainty = with_uncertainty;
    obs->FindLocalVxs(v_info_.position);
  }
}


