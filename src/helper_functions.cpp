#include "oal/helper_functions.hpp"

double GetBearing(Eigen::Vector2d dir, double obs_heading) {
  double heading_v = atan2(dir.y(), dir.x()); // error for (0,0)
  double theta = M_PI + obs_heading - heading_v; //Interception angle
  double result = remainder(theta, 2 * M_PI);
  if (result >= M_PI) result -= 2 * M_PI; // not tested
  return result;
}

Eigen::Vector2d GetProjectionInObsFrame(const Eigen::Vector2d &point, const Obstacle &obs, double time) {
  Eigen::Vector2d element_obs = point - ComputePosition(obs, time);
  Eigen::Rotation2D<double> rotation(obs.head);
  return rotation.inverse() * element_obs;
}


