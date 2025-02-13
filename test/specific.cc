#include <random>
#include "oal/path_planner.hpp"

#include <chrono>

std::vector<double> generateVel(double avg_value, int size) {
    std::vector<double> output;
    output.push_back(avg_value);
    if (size == 1) return output;
    double sign = 1;
    double gap = 0.1;
    while (output.size() < size) {
        output.push_back(avg_value + sign * gap);
        if (sign < 0) {
            if (gap == avg_value - 0.1) gap = 0.05;
            gap += 0.1;
            sign = 1;
        } else {
            sign = -1;
        }
    }
    return output;
}

std::vector<double> generateRange(double start, double end, double step) {
    std::vector<double> result;
    for (double i = start; i <= end; i += step) {
        double num = std::round(i * 100) / 100;
        if (num != 0) result.push_back(num);
    }
    return result;
}



int main(int, char **) {

    VehicleInfo v_info;
    v_info.heading = 1.55929;
    v_info.position = {-0.019284, -0.0236757};
    v_info.velocities = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1 };

    oal::PathPlanner planner;
    std::vector<Obstacle> obstacles;
    BoundingBoxData bb_data;
        bb_data.dim_x = 10;
        bb_data.dim_y = 3;
        bb_data.Set(3, {0,0}, "", 0, 0, 0, 0, 0, 0, 0);

    Obstacle obs1("obs1", {-20.1474, -0.666654}, 1.57, 0, 0, bb_data);

    obstacles.push_back(obs1);

    planner.SetVhData(v_info);
    planner.SetAccRadius(2);
    
    planner.SetObssData(obstacles);
    Path path = Path();
    
    if (!planner.ComputePath({-26.7766, -1.37875}, false, path)) return false;

    auto temp = path;
    while(!temp.empty()){
        std::cerr<<temp.top().speed_to_it<<"\n";
        temp.pop();
    }
    std::cerr<<"-.-.-\n";




}