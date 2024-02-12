#include "oal/data_structs/obstacle.hpp"
#include "oal/helper_functions.hpp"

void Obstacle::SetSize(double dist_x, double dist_y, double theta,
                       double &dim_x_bow, double &dim_x_stern, double &dim_y_starboard, double &dim_y_port) const {
  bool isAhead = (abs(theta) <= M_PI / 2);
  bool isStarboard = (theta < 0);

  double bb_ratio_x;
  double bb_ratio_y;
  double bb_ratio_x_safety;
  double bb_ratio_y_safety;

  if (isAhead) {
    dim_x_stern = bb.dim_x / 2 * bb.max_x_stern;
    bb_ratio_x = bb.max_x_bow;
    bb_ratio_x_safety = bb.safety_x_bow;
  } else {
    dim_x_bow = bb.dim_x / 2 * bb.max_x_bow;
    bb_ratio_x = bb.max_x_stern;
    bb_ratio_x_safety = bb.safety_x_stern;
  }

  if (isStarboard) {
    dim_y_port = bb.dim_y / 2 * bb.max_y_port;
    bb_ratio_y = bb.max_y_starboard;
    bb_ratio_y_safety = bb.safety_y_starboard;
  } else {
    dim_y_starboard = bb.dim_y / 2 * bb.max_y_starboard;
    bb_ratio_y = bb.max_y_port;
    bb_ratio_y_safety = bb.safety_y_port;
  }

  bool x_safety = (dist_x <= bb.dim_x / 2 * bb_ratio_x_safety);
  bool y_safety = (dist_y <= bb.dim_y / 2 *bb_ratio_y_safety);
  bool x_max = (dist_x >= bb.dim_x / 2 * bb_ratio_x);
  bool y_max = (dist_y >= bb.dim_y / 2 * bb_ratio_y);
  bool x_between = !x_safety && !x_max;
  bool y_between = !y_safety && !y_max;

  double bb_dim_x = bb.dim_x / 2 * bb_ratio_x;
  double bb_dim_y = bb.dim_y / 2 * bb_ratio_y;

  if (x_safety && !y_max) {
    bb_dim_x = bb.dim_x / 2 * bb_ratio_x_safety;
  }
  if (y_safety && !x_max) {
    bb_dim_y = bb.dim_y / 2* bb_ratio_y_safety;
  }

  if (y_between && !x_max) {
    bb_dim_y = dist_y;
  }
  if (x_between && !y_max) {
    bb_dim_x = dist_x;
  }

  if (isAhead) {
    dim_x_bow = bb_dim_x;
  } else {
    dim_x_stern = bb_dim_x;
  }
  if (isStarboard) {
    dim_y_starboard = bb_dim_y;
  }else{
    dim_y_port = bb_dim_y;
  }

  if(!uncertainty){
    dim_x_stern -= bb.gap;
    dim_x_bow -= bb.gap;
    dim_y_port -= bb.gap;
    dim_y_starboard -= bb.gap;
  }
}

void Obstacle::FindAbsVxs(double time, std::vector<Vertex> &vxs_abs) {
  Eigen::Vector2d current_obs_position = ComputePosition(*this, time);
  for (const Vertex &vx: vxs) {
    Vertex vx_abs;
    vx_abs.id = vx.id;
    Eigen::Rotation2D<double> rotation(head);
    vx_abs.position = current_obs_position + rotation * vx.position;
    vxs_abs.push_back(vx_abs);
  }
}

void Obstacle::FindLocalVxs(const Eigen::Vector2d &vhPos) {
  // Distance obs-vehicle wrt obs frame, on x and y
  Eigen::Vector2d bodyObs_vhPos = GetProjectionInObsFrame(vhPos, *this, 0);

  double dist_x = abs(bodyObs_vhPos.x());
  double dist_y = abs(bodyObs_vhPos.y());
  double theta = atan2(bodyObs_vhPos.y(), bodyObs_vhPos.x()); // error for (0,0)
  double  bb_dim_x_bow, bb_dim_x_stern, bb_dim_y_starboard, bb_dim_y_port ;
  // asymmetric bb x dimension computation
  SetSize(dist_x, dist_y, theta, bb_dim_x_bow, bb_dim_x_stern, bb_dim_y_starboard, bb_dim_y_port);
  // Find the local vertexes position
  vxs.emplace_back(FR, Eigen::Vector2d(bb_dim_x_bow, -bb_dim_y_starboard));
  vxs.emplace_back(FL, Eigen::Vector2d(bb_dim_x_bow, bb_dim_y_port));
  vxs.emplace_back(RR, Eigen::Vector2d(-bb_dim_x_stern, -bb_dim_y_starboard));
  vxs.emplace_back(RL, Eigen::Vector2d(-bb_dim_x_stern, bb_dim_y_port));
}

Eigen::Vector2d Obstacle::GetProjectionInLocalFrame(TPoint &time_point) {
  Eigen::Vector2d element_obs = time_point.pos - ComputePosition(*this, time_point.time);
  Eigen::Rotation2D<double> rotation(head);
  return rotation.inverse() * element_obs;
}

bool Obstacle::IsInBB(TPoint &time_point) {
  Eigen::Vector2d bodyObs_element = GetProjectionInLocalFrame(time_point);
  // asymmetric bb x dimension computation
  double theta = atan2(bodyObs_element.y(), bodyObs_element.x()); // Approaching angle, error for (0,0)
  // choose comparing dimension based on theta
  bool IsAhead = (abs(theta) <= M_PI / 2);
  if (IsAhead) {
    return (abs(bodyObs_element.x()) < abs(vxs[0].position.x()) &&
            abs(bodyObs_element.y()) < abs(vxs[0].position.y()));
  } else {
    return (abs(bodyObs_element.x()) < abs(vxs[2].position.x()) &&
            abs(bodyObs_element.y()) < abs(vxs[2].position.y()));
  }
}

std::string Obstacle::plotStuff(double time) {
  std::ostringstream stream;
  stream << "Obs_" << id << std::endl;
  Eigen::Vector2d abs_position = ComputePosition(*this, time);
  stream << "Position_" << abs_position.x() << "_" << abs_position.y() << std::endl;
  stream << "Heading_" << head << std::endl;
  stream << "Vel_" << vel_dir << std::endl;
  stream << "Dimx_" << bb.dim_x << std::endl;
  stream << "Dimy_" << bb.dim_y << std::endl;
  stream << "Safety_" << bb.safety_x_bow << "_" << bb.safety_x_stern << "_" << bb.safety_y_starboard<< "_" <<bb.safety_y_port<< std::endl;
  stream << "Max_" << bb.max_x_bow << "_" << bb.max_x_stern << "_" << bb.max_y_starboard<< "_" <<bb.max_y_port<< std::endl;
  std::vector<Vertex> vxs_abs;
  FindAbsVxs(time, vxs_abs);
  for (Vertex &vx: vxs_abs) {
    stream << "Vx_" << vx.position.x() << "_" << vx.position.y() << std::endl;
  }
  //std::cout << vxs_abs[wp.vx].position.x() << " " << vxs_abs[wp.vx].position.y() << " _ "  << std::endl;
  stream << "-" << std::endl;
  return stream.str();
}
