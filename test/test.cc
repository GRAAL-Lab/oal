
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

struct Entry {
    bool colregs = false;
    int n_nodes{};
    double computation_time{};
    double path_length{};
    double path_time{};
    int n_waypoints{};
    bool found = true;
    double dist_from_goal{};

    void print(std::ofstream &file) const {
        file << n_nodes << "_" << computation_time << "_" <<
             path_length << "_" << path_time << "_" << n_waypoints << "_" << found << "_" << dist_from_goal << "_"
             << colregs << ";";
    }
};

int main(int, char **) {
    bool loop_perf = true;
    bool find_case = false;
    if (loop_perf) {

        int test_number_obs = 5;
        int test_number_speeds = 5;
        int runs = 5000;

        bool colregs = true;
        bool colregs_compare = false;
        char input;
        std::cout << "Colregs compare [N/y]: ";
        std::cin >> input;
        if (input == 'y') {
            colregs_compare = true;
        } else {
            std::cout << "Colregs [N/y]: ";
            std::cin >> input;
            if (input == 'y') colregs = true;
        }


        std::cout << "Only static obs [N/y]: ";
        std::cin >> input;
        double obs_max_speed = 1.5;
        if (input == 'y') {
            obs_max_speed = 0;
        }

        std::cout << "Number max obs: ";
        std::cin >> test_number_obs;
        std::cout << "Number max speeds: ";
        std::cin >> test_number_speeds;
        std::cout << "Number runs: ";
        std::cin >> runs;


        VehicleInfo v_info;
        v_info.position = {-50, -50};
        Eigen::Vector2d goal = {50, 50};
        double area_size = 50;


        int speed_size;
        int obs_number;
        int not_found = 0;

        bb_data bb_dimension(6, 2,
                             4, 3,
                             3, 3,
                             2, 2,
                             2, 2,
                             1);

        std::random_device r;
        std::default_random_engine e1(r());
        std::uniform_real_distribution<double> pos_gen(-area_size / 2, area_size / 2);
        std::uniform_real_distribution<double> speed_gen(0, obs_max_speed);
        std::uniform_real_distribution<double> heading_gen(-M_PI, M_PI);
        std::uniform_real_distribution<double> vel_dir_gen(-M_PI / 8, M_PI / 8);


        std::ofstream file;
        file.open("data.csv", std::ofstream::out | std::ofstream::trunc);
        file << "  \n";
        for (obs_number = 1; obs_number <= test_number_obs; obs_number = obs_number+2) {
            for (speed_size = 1; speed_size <= test_number_speeds; speed_size++) {
                if (colregs_compare) {
                    file << "\n CObs" << obs_number << ",Speed" << speed_size << ",";
                } else {
                    file << "\n Obs" << obs_number << ",Speed" << speed_size << ",";
                }

                std::cout << " [" << obs_number << "/" << test_number_obs << "]  [" << speed_size << "/"
                          << test_number_speeds << "] " << std::endl;
                /*double max_speed = 0.1 + ((double)speed_size - 1)/10;
                v_info.velocities = generateRange(0.1, max_speed, 0.1);*/
                v_info.velocities = generateVel(1, speed_size);
                //for (auto v : v_info.velocities) std::cout<<v<<"_";
                double max_speed = *std::max_element(v_info.velocities.begin(), v_info.velocities.end());
                int counter = 0;

                while (counter < runs) {
                    bool is_good = true;

                    std::vector<Obstacle> obstacles;
                    for (auto i = 0; i < obs_number; i++) {
                        double heading = heading_gen(e1);
                        Obstacle obs(std::to_string(i + 1), {pos_gen(e1), pos_gen(e1)}, heading, speed_gen(e1),
                                     heading + vel_dir_gen(e1), bb_dimension);
                        //obs.print();
                        obstacles.push_back(obs);
                    }
                    auto t_start_inner = std::chrono::high_resolution_clock::now();
                    for (auto i = 0; i < 2; i++) {
                        Entry entry{};
                        if (i == 0) {
                            if (colregs_compare) colregs = false;
                        } else if (i == 1 && !colregs_compare) {
                            break;
                        } else {
                            colregs = true;
                        }

                        double ACC_RADIUS = 2;
                        path_planner planner(v_info, obstacles, ACC_RADIUS);
                        Path path, temp;
                        if (!planner.ComputePath(goal, colregs, path)) {
                            not_found++;
                            entry.found = false;
                        } else {
                            temp = path;
                            entry.dist_from_goal = planner.dist_from_goal;
                            entry.n_waypoints = (int) path.size() - 2;
                            //if(!colregs && !colregs_compare && entry.n_waypoints == 0) is_good = false;
                            auto last = path.top();
                            path.pop();
                            while (!path.empty()) {
                                entry.path_length += (path.top().position - last.position).norm();
                                last = path.top();
                                path.pop();
                            }
                            entry.path_length += (last.position - goal).norm();
                            entry.path_length = entry.path_length / (v_info.position - goal).norm();
                            //std::cout<<max_speed<<", "<<last.time<<", "<<(v_info.position - goal).norm()<<"\n";
                            entry.path_time = last.time / (v_info.position - goal).norm() / max_speed;

                            if (entry.path_length < 1) {
                                std::cout << entry.path_length * (v_info.position - goal).norm() << "\n";
                                temp.print(true);
                            }
                        }
                        entry.colregs = colregs;
                        entry.n_nodes = planner.n_node_analyzed;

                        auto t_end_inner = std::chrono::high_resolution_clock::now();
                        double elapsed_time_ms_inner = std::chrono::duration<double, std::milli>(
                                t_end_inner - t_start_inner).count();
                        entry.computation_time = elapsed_time_ms_inner;
                        if (is_good) {
                            entry.print(file);
                            counter++;
                        }
                    }
                }


            }
        }
        std::cout << " Not found number: " << not_found << std::endl;
        file.close();

        return 0;

    }else if (find_case){
        while(true) {
            // trying to get diff speeds
            VehicleInfo v_info;
            v_info.position = {0, 0};
            Eigen::Vector2d goal = {0, 60};
            v_info.heading = M_PI/2;
            //double area_size = 50;

            bb_data bb_dimension(6, 2,
                                 4, 3,
                                 3, 3,
                                 2, 2,
                                 2, 2,
                                 1);

            std::random_device r;
            std::default_random_engine e1(r());
            std::uniform_real_distribution<double> pos_gen(-10, 40);
            std::uniform_real_distribution<double> speed_gen(0, 0.3);
            std::uniform_real_distribution<double> heading_gen(-M_PI*100, M_PI*100);
            std::uniform_real_distribution<double> vel_dir_gen(-M_PI / 8, M_PI / 8);

            //v_info.velocities = generateVel(1, 50);
            v_info.velocities = {0.5,0.9,1.1,1.5,1.9};
            double ACC_RADIUS = 2;

            std::cout<<"-----------------\n";
            std::vector<Obstacle> obstacles;

            bool rand = true;
            if(rand){
                for (auto i = 0; i < 3; i++) {
                    double heading = (double) ((int) heading_gen(e1) / 100);
                    /*Obstacle obs(std::to_string(i + 1), {pos_gen(e1), pos_gen(e1)}, (double) ((int) heading / 100),
                                 speed_gen(e1),
                                 (double) ((int) heading / 100), bb_dimension);*/
                    Obstacle obs(std::to_string(i + 1), {pos_gen(e1), pos_gen(e1)}, heading,
                                 speed_gen(e1),
                                 heading+vel_dir_gen(e1), bb_dimension);
                    //obs.print();
                    obstacles.push_back(obs);
                    obs.print();
                }
            }else{
                obstacles.push_back(Obstacle("1", {-2.72504, 37.3673}, 0, 0.117363, 0.108478, bb_dimension));
                obstacles.push_back(Obstacle("2", {-0.869205, 15.8701}, 1, 0.116447, 0.764097, bb_dimension));
                obstacles.push_back(Obstacle("3", {12.9714, 1.84414}, 3, 0.122745, 2.96205, bb_dimension));


            }

            auto v_info_c = v_info;

            path_planner planner(v_info, obstacles, ACC_RADIUS);


            Path path, path_c;

            auto start = std::chrono::system_clock::now();

            if(!rand){
                planner.ComputePath(goal, false, path);


                //return 0;
            }else{
                if (planner.ComputePath(goal, false, path)) {
                    Path temp = path;
                    std::vector<double> diff_speeds ={1.0};
                    while (!path.empty()) {
                        auto top = path.top();
                        path.pop();
                        // Check if path is empty before accessing its top element
                        if (!diff_speeds.empty() && top.speed_to_it != 0 &&
                            std::find(diff_speeds.begin(), diff_speeds.end(), top.speed_to_it) == diff_speeds.end()) {
                            diff_speeds.push_back(top.speed_to_it);
                            std::cout<<".\n";
                        }
                    }
                    // Check if diff_speeds is not empty before finding max and min elements
                    if (!diff_speeds.empty()) {
                        double max_speed = *std::max_element(diff_speeds.begin(), diff_speeds.end());
                        double min_speed = *std::min_element(diff_speeds.begin(), diff_speeds.end());
                        // Check if max element minus 0.6 is greater than min element
                        std::cout<<max_speed<<"_"<<min_speed<<std::endl;
                        if (max_speed - 0.3 > min_speed) {

                            temp.print(true);
                            return 0;
                        }
                    }

                /* if(path.size()>3)return 0;
                 std::cout<<".\n";
                 while(!path.empty()){
                    *//* if(path.top().speed_to_it != 0 && path.top().speed_to_it <= 1  )
                        {

                            path.print(true);

                            return 0;
                        }*//*



                        path.pop();

                    }*/
                }
            }
            // Some computation here
            auto end = std::chrono::system_clock::now();
            //path.print(true);
            std::chrono::duration<double> elapsed_seconds = end-start;
            /*std::cout<<"computation time: "<<elapsed_seconds.count()<<std::endl;
            std::cout<<"n nodes analyzed: "<<planner.n_node_analyzed<<std::endl;
            std::cout<<"collision check avg time: "<<planner.average_time.check_collision<<"                          over n checks: "<<planner.average_time.n_ck<<std::endl;
            std::cout<<"find intercept points per speed avg time: "<<planner.average_time.find_points<<"          over n searches: "<<planner.average_time.n_points<<std::endl;
            std::cout<<"re-order open set avg time: "<<planner.average_time.set_order<<"                       over n runs: "<<planner.average_time.set_ordered<<std::endl;
            *//*if(elapsed_seconds.count()>10) {
                path.print(true);
                return 0;}*//*
            %return 0;*/
        }

        while(true){
            // trying to get diff paths with rotational cost
            VehicleInfo v_info;
            v_info.position = {0, 0};
            Eigen::Vector2d goal = {0, 60};
            v_info.heading = M_PI/2;
            //double area_size = 50;

            bb_data bb_dimension(6, 2,
                                 4, 3,
                                 3, 3,
                                 2, 2,
                                 2, 2,
                                 1);

            std::random_device r;
            std::default_random_engine e1(r());
            std::uniform_real_distribution<double> pos_gen(-10, 40);
            //std::uniform_real_distribution<double> speed_gen(0, 0.3);
            std::uniform_real_distribution<double> heading_gen(-M_PI*100, M_PI*100);
            //std::uniform_real_distribution<double> vel_dir_gen(-M_PI / 8, M_PI / 8);

            v_info.velocities = generateVel(1, 5);
            double ACC_RADIUS = 2;

            std::cout<<"-----------------\n";
            std::vector<Obstacle> obstacles;

            bool rand = false;
            if(rand){
                for (auto i = 0; i < 2; i++) {
                    double heading = heading_gen(e1);
                    Obstacle obs(std::to_string(i + 1), {pos_gen(e1), pos_gen(e1)}, (double) ((int) heading / 100),
                                 0.0,
                                 (double) ((int) heading / 100), bb_dimension);
                    //obs.print();
                    obstacles.push_back(obs);
                    obs.print();
                }
            }else{
                obstacles.push_back(Obstacle("1", {21.0481, 11.9245}, -2, 0, -2, bb_dimension));
                obstacles.push_back(Obstacle("2", {-1.51251, 37.3104}, 1, 0, 1, bb_dimension));
            }

            v_info.rot_speed = 0;
            auto v_info_c = v_info;
            v_info_c.rot_speed = 0.314;
            path_planner planner(v_info, obstacles, ACC_RADIUS);
            path_planner planner_c(v_info_c, obstacles, ACC_RADIUS);

            Path path, path_c;

            if(!rand){
                //planner.ComputePath(goal, false, path);
                planner_c.ComputePath(goal, false, path_c);
                return 0;
            }else{
                if (planner.ComputePath(goal, false, path) && planner_c.ComputePath(goal, false, path_c)) {
                    std::cout<<".\n";
                    while(!path.empty()){
                        if((path.top().position - path_c.top().position).norm() > 1){
                            std::cout<<" Found one with diff: "<<(path.top().position - path_c.top().position).norm()<<std::endl;
                            path.print(true);
                            path_c.print(true);
                            return 0;
                        }
                        path.pop();
                        path_c.pop();
                    }
                }
            }
        }
    }

    std::cout << "----------------------------------------\n STARTING NEW PLAN " << std::endl;

    VehicleInfo v_info;
    Eigen::Vector2d goal;
    std::vector<Obstacle> obstacles;
    bool colregs = true;

    /*bb_data bb_dimension(2, 1,
                         4, 3,
                         3, 3,
                         2, 2,
                         2, 2,
                         1);*/
    bb_data bb_dimension(2, 1,
                         2, 3,
                         2, 2,
                         1, 1,
                         1, 1,
                         1);

    int scenario = 21;
    std::cin >> scenario;

    v_info.velocities = generateRange(1.0, 1.0, 0.1);
    // Keep constants
    v_info.position = {-50, -50};
    goal = {50, 50};


    /* TODO scenario
     * one where the vehicle should stand on but the ts is limited and so fast os cannot reach the front vxs
     *    ..does it return 'no path found' as it should?
     *
    */

    switch (scenario) {


        case 21: {
            //Overtaking and crossing situation on the high seas
            // https://www.advanced.ecolregs.com/index.php?option=com_k2&view=item&id=172:overtaking-and-crossing-situation-on-the-high-seas&Itemid=359&lang=en
            v_info.position = {10, 0};
            Obstacle obsB("B", {20, -10}, M_PI / 2, 1, M_PI / 2, bb_dimension);
            Obstacle obsC("C", {40, 40}, -M_PI * 5 / 6, 1, -M_PI * 5 / 6, bb_dimension);
            obstacles.push_back(obsB);
            obstacles.push_back(obsC);
            goal = {10, 50};
            break;

        }
        case 22: {
            //Overtaking and crossing situation on the high seas
            // https://www.advanced.ecolregs.com/index.php?option=com_k2&view=item&id=367:overtaking-and-crossing-situation-on-the-high-seas&Itemid=359&lang=en
            v_info.position = {10, 0};
            Obstacle obsB("B", {15, -10}, M_PI / 2, 1, M_PI / 2, bb_dimension);
            Obstacle obsC("C", {40, 30}, -M_PI, 1, -M_PI, bb_dimension);
            obstacles.push_back(obsB);
            obstacles.push_back(obsC);
            goal = {10, 50};
            break;
        }
        case 23: {
            //Overtaking and head-on situation on the high seas
            // https://www.advanced.ecolregs.com/index.php?option=com_k2&view=item&id=370:overtaking-and-head-on-situation-on-the-high-seas&Itemid=359&lang=en
            v_info.position = {10, 0};
            Obstacle obsB("B", {10, 20}, -M_PI / 2, 1, -M_PI / 2, bb_dimension);
            Obstacle obsC("C", {13.2, 25}, -M_PI / 2, 1, -M_PI / 2, bb_dimension);
            obstacles.push_back(obsB);
            obstacles.push_back(obsC);
            goal = {10, 20};
            break;
        }
        case 24: {
            //
            v_info.position = {10, 0};
            Obstacle obsB("B", {-5, 15}, 0, 1, 0, bb_dimension);
            Obstacle obsC("C", {-5, 10}, 0, 1, 0, bb_dimension, true);
            obstacles.push_back(obsB);
            obstacles.push_back(obsC);
            goal = {10, 20};
            break;
        }


        case 2: {// head on WORKS (clear differences with/without Colregs)
            v_info.position = {10, 0};
            Obstacle obs("1", {10.5, 35}, -M_PI / 2, 1, -M_PI / 2, bb_dimension);
            obstacles.push_back(obs);
            goal = {10, 20};
            break;
        }
        case 3: {// TS crossing from right
            v_info.position = {10, 0};
            Obstacle obs("1", {24.4009, 10}, M_PI * 6 / 7, 0.5, M_PI * 6 / 7, bb_dimension);
            obstacles.push_back(obs);
            goal = {10, 20};
            break;
        }
        case 4: {// crossing left
            v_info.position = {10, 0};
            //Obstacle obs4_1 = Obstacle("1", {8.5, 4}, 0, 1, 0.5, 0.5, 2, 1.5);
            //obss_info.obstacles.push_back(obs4_1);
            goal = {10, 10};
            break;
        }

        case 5: // overtake
            break;
        case 6: // overtaken
            break;

        case 8: {// start in bb
            v_info.position = {10, 0};
            //Obstacle obs1("1", {13, 4.5}, 0, 3.12, 0, bb_dimension);
            Obstacle obs1("1", {-4, 0.5}, 0, 0.9, 0, bb_dimension);
            //Obstacle obs2("2", {10, 16}, -M_PI / 2, 0.9, -M_PI / 2, bb_dimension);
            //obstacles.push_back(obs1);
            obstacles.push_back(obs1);
            goal = {10, 40};
            break;
        }
        case 9: {// goal in bb
            v_info.position = {10, 0};
            Obstacle obs1("1", {10, 18}, M_PI / 2, 0, M_PI / 2, bb_dimension, true);
            obstacles.push_back(obs1);
            goal = {10, 20};
            break;
        }
        case 10: { // bb overlap
            v_info.position = {10, 0};
            Obstacle obs1("1", {9, 3.7}, 0, 0, 0, bb_dimension);
            obstacles.push_back(obs1);
            Obstacle obs2("2", {11, 4}, 0, 0, 0, bb_dimension);
            obstacles.push_back(obs1);
            goal = {10, 10};
            break;
        }
        case 11: { // goal surrounded
            v_info.position = {10, 0};
            Obstacle obs1("1", {10, 9}, 0, 0, 0, bb_dimension);
            obstacles.push_back(obs1);
            Obstacle obs2("2", {10, 11}, 0, 0, 0, bb_dimension);
            obstacles.push_back(obs1);
            Obstacle obs3("3", {11, 10}, 0, 0, 0, bb_dimension);
            obstacles.push_back(obs1);
            Obstacle obs4("4", {9, 10}, 0, 0, 0, bb_dimension);
            obstacles.push_back(obs1);
            obstacles.push_back(obs1);
            obstacles.push_back(obs2);
            obstacles.push_back(obs3);
            obstacles.push_back(obs4);
            goal = {10, 10};
            break;
        }
        default:
            break;
    }

    double ACC_RADIUS = 2;

    path_planner planner1(v_info, obstacles, ACC_RADIUS);
    path_planner planner2(v_info, obstacles, ACC_RADIUS);

    /*Path path1;
    std::cout << std::endl << "Colregs: false";
    if (planner1.ComputePath(goal, false, path1)) {
      std::cout << std::endl << "Found." << std::endl;
    } else {
      std::cout << std::endl << "Not found." << std::endl;
    }*/

    Path path2;
    std::cout << std::endl << "Colregs: " << false << std::endl;
    if (planner2.ComputePath(goal, false, path2)) {
        std::cout << " n. inner waypoints: " << path2.size() - 2 << std::endl;
        planner2.print(goal);
        std::cout << " planner done" << std::endl;
        path2.UpdateMetrics(v_info.position, 0, v_info.rot_speed);
        path2.print();
        /*if (path2.debug_flag) {
          break;
        }*/
        v_info.position.x() += 1;
        v_info.position.y() += -1;
        Eigen::Vector2d unreachable_wp;
        if (planner1.CheckPath(v_info.position, path2, unreachable_wp)) {
            std::cout << "Checked!!" << std::endl;
        } else {
            std::cout << " Could not reach wp: " << unreachable_wp.x() << ", " << unreachable_wp.y() << std::endl;
        }
    } else {
        std::cout << std::endl << "Not found." << std::endl;
    }

    std::cout << std::endl << "Colregs: " << true << std::endl;
    if (planner2.ComputePath(goal, true, path2)) {
        std::cout << " n. inner waypoints: " << path2.size() - 2 << std::endl;
        planner2.print(goal);
        std::cout << " planner done" << std::endl;
        path2.UpdateMetrics(v_info.position, 0, v_info.rot_speed);
        path2.print();
        /*if (path2.debug_flag) {
          break;
        }*/
        v_info.position.x() += 1;
        v_info.position.y() += -1;
        Eigen::Vector2d unreachable_wp;
        if (planner1.CheckPath(v_info.position, path2, unreachable_wp)) {
            std::cout << "Checked!!" << std::endl;
        } else {
            std::cout << " Could not reach wp: " << unreachable_wp.x() << ", " << unreachable_wp.y() << std::endl;
        }
    } else {
        std::cout << std::endl << "Not found." << std::endl;
    }

    // this after I'm done following the path (even a part of it)
    //auto priorOvertakenVesselsList = path2.overtakingObsList;

    /*int s_count = 0;
      double s_value = 0;
      Node wp;
      while (!path2.empty()) {
        wp = path2.top();
        path2.pop();
        if (wp.vh_speed != s_value) {
          s_count++;
          s_value = wp.vh_speed;
        }
      }
      std::cout << "  Time: " << wp.time;
      if(wp.time<=0) throw std::invalid_argument("wtf");
      std::cout << "  speed_change: " << s_count<< std::endl;*/


}

