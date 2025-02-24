
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <cmath>
#include <iostream>
#include <iomanip>

#include "oal/devel/geometric_utilities.hpp"
#include "oal/devel/generator.hpp"

using namespace std;
using namespace oal;

DebugSettings ds;
Pose vh_pose;
Eigen::Vector2d goal;
PruningParams pp;

bool Test_Generator();
bool Test_GeneratorWithObstacles();
bool Test_Node(bool print = false);
bool Test_Obstacle(bool print = false);
bool Test_NodeObstacles(bool print = false);

int main() {
    bool test_results = true;

    ds.printCurrentNode = false;
    ds.printPath = true;
    ds.logNodes = true;
    ds.logNodesPathFile = "/home/graal/graal_ws/oal/logs/test.txt";
    ds.printNodesStats = true;

    vh_pose = Pose({0,0}, 0.6);
    goal = {150,0};

    // test_results &= Test_Node();
    //test_results &= Test_Obstacle();
    // test_results &= Test_NodeObstacles(true);
    //test_results &= Test_Generator();
    test_results &= Test_GeneratorWithObstacles();

    if(test_results) {
        cerr<<"Test: Ok\n";
    }else{
        cerr<<"Test: Fail\n";
    }

    cerr<<"\n--------------\n";
    return 0;
}

BoundingBoxData bb_data_default();
VehicleData vh_data_default();
EncounterData encounter_data_default(Eigen::Vector2d position, double time, double speed, std::shared_ptr<Obstacle> obs_ptr = nullptr, VxId vx = NA);
void PrintVxs(std::vector<Vx> vxs);

bool Test_Generator(){

    auto g = Generator(vh_data_default(), pp, ds);
    std::vector<ObsPtr> obstacles;

    AstarPath path;
    auto out = g.FindPath(vh_pose, goal, obstacles, path);
    cerr << "Output: ["<< out.result << "] " << out.failMsg << endl;

    if(out.result == SearchResult::FOUND) return true;
    return false;

}

bool Test_GeneratorWithObstacles(){

    VehicleData vh_data = vh_data_default();
    PruningParams pp;
    auto g = Generator(vh_data, pp, ds);

    goal = {150,150};

    std::vector<ObsPtr> obstacles;
    Pose obs1_pose = {{10,5}, 0};
    auto obs1 = Obstacle("obs1", "", obs1_pose,{0,0}, bb_data_default(), obs1_pose.FromWorld2ThisFrame(vh_pose.Position()));
    obstacles.push_back(std::make_shared<Obstacle>(obs1));
    Pose obs2_pose = {{70,5}, 0};
    auto obs2 = Obstacle("obs2", "", obs2_pose,{0,0}, bb_data_default(), obs2_pose.FromWorld2ThisFrame(vh_pose.Position()));
    obstacles.push_back(std::make_shared<Obstacle>(obs2));

    AstarPath path;
    auto out = g.FindPath(vh_pose, goal, obstacles, path);

    cerr << "Output: ["<< out.result << "] " << out.failMsg << endl;

    if(out.result == SearchResult::FOUND) return true;
    return false;

}

bool Test_NodeObstacles(bool print){
    Pose obs1_pose = {{10,5}, 0};
    Pose obs2_pose = {{30,5}, 0};
    auto obs1 = Obstacle("obs1", "", obs1_pose,{0,0}, bb_data_default(), obs1_pose.FromWorld2ThisFrame(vh_pose.Position()));
    auto obs2 = Obstacle("obs2", "", obs2_pose,{0,0}, bb_data_default(), obs2_pose.FromWorld2ThisFrame(vh_pose.Position()));
    auto start = make_shared<AStarNode>(AStarNode(encounter_data_default(vh_pose.Position(),0,0)));
    start->SetCosts(vh_data_default(), goal);
    auto second = make_shared<AStarNode>(AStarNode(encounter_data_default({10,0},10,1, make_shared<Obstacle>(obs1),FR), start));
    second->SetCosts(vh_data_default(), goal);
    auto third = make_shared<AStarNode>(AStarNode(encounter_data_default({30,0},30,1,  make_shared<Obstacle>(obs2),RR), second));
    third->SetCosts(vh_data_default(), goal);
    auto current = third;
    while (current->Parent() != nullptr){
        current->SetCosts(vh_data_default(), {50,0});
        if(print) current->Print();
        current = current->Parent();
    }
    return true;
}

bool Test_Obstacle(bool print){
    auto obs1 = Obstacle("obs1", "", {{10,10},0},{10,0}, bb_data_default(), {10,10});
    auto obs2 = Obstacle("obs2", "", {{30,10},0},{0,10}, bb_data_default(), {30,10});
    if(print) {
        obs1.Print();
        obs2.Print();
    }

    std::vector<Vx> vxs;
    
    vxs = obs1.GetVxs(std::chrono::duration<double>(0));
    PrintVxs(vxs);
    vxs = obs1.GetVxs(std::chrono::duration<double>(10));
    PrintVxs(vxs);

    vxs = obs2.GetVxs(std::chrono::duration<double>(0));
    PrintVxs(vxs);
    vxs = obs2.GetVxs(std::chrono::duration<double>(10));
    PrintVxs(vxs);

    return true;
}

bool Test_Node(bool print){
    auto start = make_shared<AStarNode>(AStarNode(encounter_data_default({0,0},0,0)));
    auto second = make_shared<AStarNode>(AStarNode(encounter_data_default({10,0},10,1), start));
    auto third = make_shared<AStarNode>(AStarNode(encounter_data_default({20,0},20,1), second));
    auto current = third;
    while (current->Parent() != nullptr){
        current->SetCosts(vh_data_default(), {50,0});
        if(print) current->Print();
        current = current->Parent();
    }
    return true;
}


EncounterData encounter_data_default(Eigen::Vector2d position, double time, double speed, std::shared_ptr<Obstacle> obs_ptr, VxId vx){
    EncounterData data;
    data.position = position;
    data.time = chrono::duration<double>(time);
    data.approachingSpeed = speed;
    data.obs_ptr = obs_ptr;
    data.vx = vx;
    return data;
}

BoundingBoxData bb_data_default(){
    BoundingBoxData bb_data;
    bb_data.dim_x = 10;
    bb_data.dim_y = 3;

    bb_data.minDistFromObs = 1;
    bb_data.reductionWhileCheckingPath = 1; // TODO this could depend only on the ASV capability of staying on the predicted course?
    bb_data.safeMaxGap = 0;
    bb_data.lookAheadSafetySpan = 0; //seconds

    bb_data.Set(3, {0,0}, "", 0, 0, 0, 0, 0, 0, 0);
    return bb_data;
}

VehicleData vh_data_default(){
    VehicleData vh_data;
    vh_data.velocities ={10,1};
    return vh_data;
}

void PrintVxs(std::vector<Vx> vxs){
    if(!ds.printComputedVxs) return;
    for(const auto& vx:vxs){
        cerr<<vx.first<<": "<<vx.second.transpose()<<"\n";
    }
};