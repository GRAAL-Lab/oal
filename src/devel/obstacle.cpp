#include "oal/devel/obstacle.hpp"

std::vector<oal::Vx> oal::Obstacle::GetVxs(TimeDouble time, bool compensate_localization_error) const {
    /* Get the absolute position of the vertexes at a given time
        * @param time: time instant
        * @param compensate_localization_error: if true, it is checking old path, so reduce the bb size
        * @return: vector of vertexes
    */
    std::vector<oal::Vx> vxs_abs;
    Pose pose_t(GetPosition(time), initial_pose.Heading());
    for (Vx vx: vxs) {
        Vx vx_abs;
        vx_abs.first = vx.first;
        if(compensate_localization_error) vx.second -= Eigen::Vector2d(bb_data.reductionWhileCheckingPath, bb_data.reductionWhileCheckingPath);
        vx_abs.second = pose_t.FromThis2WorldFrame(vx.second);
        vxs_abs.push_back(vx_abs);
    }
    return vxs_abs;
}

Eigen::Vector2d oal::Obstacle::GetPosition(TimeDouble time) const {
    Eigen::Vector2d shift = velocity * time.count();
    return initial_pose.Position() + shift;
}

void oal::Obstacle::SetLocalVxs(){
    auto bodyObs_vhPos = obs_P_vehicle;
    double dist_x = abs(bodyObs_vhPos.x());
    double dist_y = abs(bodyObs_vhPos.y());
    double theta = atan2(bodyObs_vhPos.y(), bodyObs_vhPos.x()); // error for (0,0)

    bool isAhead = (abs(theta) <= M_PI / 2);
    bool isStarboard = (theta < 0);


    /* Actual bb vxs computation:
        - Keep max bb on the side opposite to the ASV approaching ones
        - Select the approaching side max and safety (in dim_*_max and dim_*_safety)
        - Choose the latter values depending on the ASV distance from the obstacle
    */
    double dim_x_bow, dim_x_stern, dim_y_starboard, dim_y_port; 
    double dim_x_max, dim_x_safety, dim_y_max, dim_y_safety;
    if (isAhead) {
        dim_x_stern = bb_data.max_x_stern;
        dim_x_max = bb_data.max_x_bow;
        dim_x_safety = bb_data.safety_x_bow;
    } else {
        dim_x_bow = bb_data.max_x_bow;
        dim_x_max = bb_data.max_x_stern;
        dim_x_safety = bb_data.safety_x_stern;
    }

    if (isStarboard) {
        dim_y_port = bb_data.max_y_port;
        dim_y_max = bb_data.max_y_starboard;
        dim_y_safety = bb_data.safety_y_starboard;
    } else {
        dim_y_starboard = bb_data.max_y_starboard;
        dim_y_max = bb_data.max_y_port;
        dim_y_safety = bb_data.max_y_starboard;
    }

    bool x_safety = (dist_x <= dim_x_safety); //ASV inside safety
    bool y_safety = (dist_y <= dim_y_safety); //ASV inside safety
    bool x_max = (dist_x >= dim_x_max); //ASV outside max
    bool y_max = (dist_y >= dim_y_max); //ASV outside max
    bool x_between = !x_safety && !x_max; //ASV in between
    bool y_between = !y_safety && !y_max; //ASV in between

    // Selection of the ASV approaching side bb size
    double bb_dim_x = dim_x_max;
    double bb_dim_y = dim_y_max;
    if (x_safety && !y_max) {
        bb_dim_x = dim_x_safety;
    }
    if (y_safety && !x_max) {
        bb_dim_y = dim_y_safety;
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
    } else {
        dim_y_port = bb_dim_y;
    }

    // if (!compensate_localization_error) {
    //     dim_x_stern -= bb_data.reductionWhileCheckingPath;
    //     dim_x_bow -= bb_data.reductionWhileCheckingPath;
    //     dim_y_port -= bb_data.reductionWhileCheckingPath;
    //     dim_y_starboard -= bb_data.reductionWhileCheckingPath;
    // }
    // Find the local vertexes position
    vxs.push_back({FR, Eigen::Vector2d(dim_x_bow, -dim_y_starboard)});
    vxs.push_back({FL, Eigen::Vector2d(dim_x_bow, dim_y_port)});
    vxs.push_back({RR, Eigen::Vector2d(-dim_x_stern, -dim_y_starboard)});
    vxs.push_back({RL, Eigen::Vector2d(-dim_x_stern, dim_y_port)});

}