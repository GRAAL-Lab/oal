#include "oal/obstacle.hpp"
#include "oal/helper_functions.hpp"

void Obstacle::ComputeLocalVxsBasedOnVhDist(const Eigen::Vector2d &bodyObs_vhPos, bool compensate_localization_error) {
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

    if (!compensate_localization_error) {
        dim_x_stern -= bb_data.reductionWhileCheckingPath;
        dim_x_bow -= bb_data.reductionWhileCheckingPath;
        dim_y_port -= bb_data.reductionWhileCheckingPath;
        dim_y_starboard -= bb_data.reductionWhileCheckingPath;
    }
    // Find the local vertexes position
    vxs.emplace_back(FR, Eigen::Vector2d(dim_x_bow, -dim_y_starboard));
    vxs.emplace_back(FL, Eigen::Vector2d(dim_x_bow, dim_y_port));
    vxs.emplace_back(RR, Eigen::Vector2d(-dim_x_stern, -dim_y_starboard));
    vxs.emplace_back(RL, Eigen::Vector2d(-dim_x_stern, dim_y_port));

    // std::cerr << "-------------------------\n Case determination:\n";
    // std::cerr << "Distances:\n";
    // std::cerr << "  dist_x: " << dist_x << "\n";
    // std::cerr << "  dist_y: " << dist_y << "\n";
    // std::cerr << "  isAhead: " << (isAhead ? "true" : "false") << "\n";
    // std::cerr << "  isStarboard: " << (isStarboard ? "true" : "false") << "\n";

    // std::cerr << "Dimensions:\n";
    // std::cerr << "  dim_x_bow: " << dim_x_bow << "\n";
    // std::cerr << "  dim_x_stern: " << dim_x_stern << "\n";
    // std::cerr << "  dim_y_starboard: " << dim_y_starboard << "\n";
    // std::cerr << "  dim_y_port: " << dim_y_port << "\n";

    // std::cerr << "Safety and max checks:\n";
    // std::cerr << "  x_safety: " << (x_safety ? "true" : "false") << "\n";
    // std::cerr << "  y_safety: " << (y_safety ? "true" : "false") << "\n";
    // std::cerr << "  x_max: " << (x_max ? "true" : "false") << "\n";
    // std::cerr << "  y_max: " << (y_max ? "true" : "false") << "\n";
    // std::cerr << "  x_between: " << (x_between ? "true" : "false") << "\n";
    // std::cerr << "  y_between: " << (y_between ? "true" : "false") << "\n";

    // std::cerr << "Selected dimensions:\n";
    // std::cerr << "  bb_dim_x: " << bb_dim_x << "\n";
    // std::cerr << "  bb_dim_y: " << bb_dim_y << "\n";

    // std::cerr << "Vertex positions (vxs):\n";
    // for (const auto& vertex : vxs) {
    //     std::cerr << "  Vertex: (" 
    //             << vertex.position.x() << ", " << vertex.position.y() << ")\n";
    // }

}

void Obstacle::FindAbsVxs(std::chrono::duration<double> time, std::vector<Vertex> &vxs_abs) {
    // Eigen::Vector2d current_obs_position = ComputePosition(time);
    for (const Vertex &vx: vxs) {
        Vertex vx_abs = vx;
        //vx_abs.id = vx.id;
        Eigen::Rotation2D<double> rotation(pose.heading);
        vx_abs.position = pose.position + rotation * vx.position;
        vxs_abs.push_back(vx_abs);
    }
}


Eigen::Vector2d Obstacle::GetProjectionInLocalFrame(TPoint &time_point) {
    Eigen::Vector2d element_obs = time_point.position - ComputePosition(*this, std::chrono::duration_cast<std::chrono::seconds>(time_point.time));
    Eigen::Rotation2D<double> rotation(pose.heading);
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

std::string Obstacle::plotStuff(std::chrono::duration<double> time) {
    std::ostringstream stream;
    // stream << "Obs_" << id << std::endl;
    // Eigen::Vector2d abs_position = ComputePosition(*this, time);
    // stream << "Position_" << abs_position.x() << "_" << abs_position.y() << std::endl;
    // stream << "Heading_" << head << std::endl;
    // stream << "Vel_" << vel_dir << std::endl;
    // stream << "Dimx_" << bb.dim_x << std::endl;
    // stream << "Dimy_" << bb.dim_y << std::endl;
    // stream << "Safety_" << bb.safety_x_bow << "_" << bb.safety_x_stern << "_" << bb.safety_y_starboard << "_"
    //        << bb.safety_y_port << std::endl;
    // stream << "Max_" << bb.max_x_bow << "_" << bb.max_x_stern << "_" << bb.max_y_starboard << "_" << bb.max_y_port
    //        << std::endl;
    // std::vector<Vertex> vxs_abs;
    // FindAbsVxs(time, vxs_abs);
    // for (Vertex &vx: vxs_abs) {
    //     stream << "Vx_" << vx.position.x() << "_" << vx.position.y() << std::endl;
    // }
    // //std::cout << vxs_abs[wp.vx].position.x() << " " << vxs_abs[wp.vx].position.y() << " _ "  << std::endl;
    // stream << "-" << std::endl;
    return stream.str();
}
