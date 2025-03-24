
#include <cmath>
#include <eigen3/Eigen/Eigen>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

#include "oal/geometric_utilities.hpp"
#include "oal/local_planner/generator.hpp"

using namespace std;
using namespace oal;

std::shared_ptr<DebugSettings> ds;
Pose vh_pose;
Eigen::Vector2d goal;
PruningParams pp;

bool Test_Generator();
bool Test_GeneratorWithObstacles();
bool Test_Node(bool print = false);
bool Test_Obstacle(bool print = false);
bool Test_NodeObstacles(bool print = false);
bool Test_Random(int iterations, bool print = false);
bool Test_Specific(bool print = false);

int main()
{
    bool test_results = true;

    ds = std::make_shared<DebugSettings>();

    ds->printCurrentNode = false;
    ds->printPath = false;

    ds->completePathNodesLog.log = true;
    // NodeLogger failedPath;
    // NodeLogger validPath;
    // NodeLogger notValidPath;
    // ds->logNodes = true;
    // ds->logNodesPathFile = "/home/graal/graal_ws/oal/logs/nodes.txt";
    ds->logObstacles = true;
    ds->logObstaclesPathFile = "/home/graal/graal_ws/oal/logs/obstacles.txt";
    ds->printNodesStats = true;

    vh_pose = Pose({ 0, 0 }, 0.6);
    goal = { 150, 0 };

    std::vector<std::string> highPriorityList = { "sailboat" };
    oal::PathEvaluator::SetHighPriorityObstacles(highPriorityList);

    // test_results &= Test_Node();
    // test_results &= Test_Obstacle();
    // test_results &= Test_NodeObstacles();
    // test_results &= Test_Generator();
    // test_results &= Test_GeneratorWithObstacles();
    // test_results &= Test_Random();
    test_results &= Test_Random(100000, true);
    // test_results &= Test_Specific(true);

    cerr << std::endl
         << std::endl;
    if (test_results) {
        cerr << "Test: Ok\n";
    } else {
        cerr << "Test: Fail\n";
    }

    cerr << "\n--------------\n";
    return 0;
}

BoundingBoxData bb_data_default();
VehicleData vh_data_default();
EncounterData encounter_data_default(Eigen::Vector2d position, double time, double speed, std::shared_ptr<Obstacle> obs_ptr = nullptr, VxId vx = NA);
double GetRandomInRange(double min, double max);
void PrintVxs(std::vector<Vx> vxs);

bool Test_Generator()
{

    auto g = Generator(vh_data_default(), pp, ds);
    std::vector<ObsPtr> obstacles;

    AstarPath path;
    auto out = g.FindPath(vh_pose, goal, obstacles, path);
    cerr << "Output: [" << out.result << "] " << out.failMsg << endl;

    if (out.result == SearchResult::FOUND)
        return true;
    return false;
}

bool Test_GeneratorWithObstacles()
{

    VehicleData vh_data = vh_data_default();
    PruningParams pp;
    auto g = Generator(vh_data, pp, ds);

    goal = { 150, 150 };

    std::vector<ObsPtr> obstacles;
    Pose obs1_pose = { { 0, 5 }, 0 };
    auto obs1 = Obstacle("obs1", "", obs1_pose, { 0, 0 }, bb_data_default(), vh_pose.Position());
    obstacles.push_back(std::make_shared<Obstacle>(obs1));
    Pose obs2_pose = { { 70, 5 }, 0 };
    auto obs2 = Obstacle("obs2", "", obs2_pose, { 0, 0 }, bb_data_default(), vh_pose.Position());
    obstacles.push_back(std::make_shared<Obstacle>(obs2));

    AstarPath path;
    auto out = g.FindPath(vh_pose, goal, obstacles, path);

    cerr << "Output: [" << out.result << "] " << out.failMsg << endl;

    if (out.result == SearchResult::FOUND)
        return true;
    return false;
}

bool Test_Specific(bool print)
{
    VehicleData vh_data = vh_data_default();
    PruningParams pp;

    goal = { 500, 0 };

    auto g = Generator(vh_data, pp, ds);

    std::vector<ObsPtr> obstacles;
    std::vector<std::tuple<std::string, Pose, Eigen::Vector2d>> obstacle_data = {
        { "5", { { 168.812, 4.05233 }, 124.505 * M_PI / 180 }, { 0, 0 } },
        { "4", { { 134.7, -6.8 }, 176.6 * M_PI / 180 }, { 0, 0 } },
        { "3", { { 365.6, 6.9 }, -121.3 * M_PI / 180 }, { 0, 0 } },
        { "2", { { 344.1, 8.4 }, -2.5 * M_PI / 180 }, { 0, 0 } },
        { "1", { { 164.5, 17.5 }, 45.7 * M_PI / 180 }, { 0, 0 } }
    };

    for (const auto& [obs_id, obs_pose, obs_vel] : obstacle_data) {
        auto obs = Obstacle(obs_id, "", obs_pose, obs_vel, bb_data_default(), vh_pose.Position());
        obstacles.push_back(std::make_shared<Obstacle>(obs));
    }

    AstarPath path;
    auto out = g.FindPath(vh_pose, goal, obstacles, path);

    if (out.result != SearchResult::FOUND) {
        cerr << "Output: [" << out.result << "] " << out.failMsg << endl;

        if (print) {
            cerr << "Obstacles: \n";

            while (!obstacles.empty()) {
                auto obs = obstacles.back();
                obstacles.pop_back();
                obs->Print();
            }
        }

        if (out.result != SearchResult::PARTIAL)
            return false;
    } else {
        path.Print();
    }

    return true;
}

bool Test_Random(int iterations, bool print)
{

    VehicleData vh_data = vh_data_default();
    PruningParams pp;

    goal = { 500, 0 };

    std::cerr << "- Random test\n";
    int count;
    while (iterations) {
        pp.colregsCompliant = !pp.colregsCompliant;

        auto g = Generator(vh_data, pp, ds);
        if (iterations % 100 == 0)
            std::cerr << ".";
        if (iterations % 10000 == 0)
            std::cerr << "\n";
        std::vector<ObsPtr> obstacles;
        for (int i = 1; i < GetRandomInRange(1, 10); i++) {
            Pose obs_pose = { { GetRandomInRange(-100, 600), GetRandomInRange(-20, 20) }, GetRandomInRange(-M_PI, M_PI) };
            // Eigen::Vector2d obs_vel = {0,0};
            Eigen::Vector2d obs_vel = { GetRandomInRange(0, 1), GetRandomInRange(0, 1) };
            std::string obs_id = "" + std::to_string(i);
            auto obs = Obstacle(obs_id, "", obs_pose, obs_vel, bb_data_default(), vh_pose.Position());
            obstacles.push_back(std::make_shared<Obstacle>(obs));
        }

        AstarPath path;
        auto out = g.FindPath(vh_pose, goal, obstacles, path);

        if (out.result != SearchResult::FOUND) {
            cerr << "Output: [" << out.result << "] " << out.failMsg << endl;

            if (print) {
                cerr << "Obstacles: \n";

                while (!obstacles.empty()) {
                    auto obs = obstacles.back();
                    obstacles.pop_back();
                    obs->Print();
                }
            }

            if (out.result != SearchResult::PARTIAL)
                return false;
        }

        Eigen::Vector2d uwp;
        auto start = path.Data().front();
        path.Data().pop_front();
        auto new_vh_pose = Pose((vh_pose.Position() + Eigen::Vector2d(GetRandomInRange(-10, 10), GetRandomInRange(-10, 10))), vh_pose.Heading());
        if (!g.IsPathValid(path, new_vh_pose, obstacles, uwp)) {
            // start->Print("_OriginalStart");
            // cerr<<" New vehicle pose: "<<new_vh_pose.Position().transpose()<<std::endl;
            // path.Print();
            count++;
        }

        iterations--;
    }

    cerr << "\nNot valid paths after moving the vehicle: " << count << std::endl;

    return true;
}

bool Test_NodeObstacles(bool print)
{
    Pose obs1_pose = { { 10, 5 }, 0 };
    Pose obs2_pose = { { 30, 5 }, 0 };
    auto obs1 = Obstacle("obs1", "", obs1_pose, { 0, 0 }, bb_data_default(), vh_pose.Position());
    auto obs2 = Obstacle("obs2", "", obs2_pose, { 0, 0 }, bb_data_default(), vh_pose.Position());
    auto start = make_shared<AStarNode>(encounter_data_default(vh_pose.Position(), 0, 0));
    start->SetCosts(vh_data_default(), goal);
    auto second = make_shared<AStarNode>(encounter_data_default({ 10, 0 }, 10, 1, make_shared<Obstacle>(obs1), FR), start);
    second->SetCosts(vh_data_default(), goal);
    auto third = make_shared<AStarNode>(encounter_data_default({ 30, 0 }, 30, 1, make_shared<Obstacle>(obs2), RR), second);
    third->SetCosts(vh_data_default(), goal);
    auto current = third;
    while (current->Parent() != nullptr) {
        current->SetCosts(vh_data_default(), { 50, 0 });
        if (print)
            current->Print();
        current = current->Parent();
    }
    return true;
}

bool Test_Obstacle(bool print)
{
    auto obs1 = Obstacle("obs1", "", { { 10, 10 }, 0 }, { 10, 0 }, bb_data_default(), { 10, 10 });
    auto obs2 = Obstacle("obs2", "", { { 30, 10 }, 0 }, { 0, 10 }, bb_data_default(), { 30, 10 });
    if (print) {
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

bool Test_Node(bool print)
{
    auto start = make_shared<AStarNode>(encounter_data_default({ 0, 0 }, 0, 0));
    auto second = make_shared<AStarNode>(encounter_data_default({ 10, 0 }, 10, 1), start);
    auto third = make_shared<AStarNode>(encounter_data_default({ 20, 0 }, 20, 1), second);
    auto current = third;
    while (current->Parent() != nullptr) {
        current->SetCosts(vh_data_default(), { 50, 0 });
        if (print)
            current->Print();
        current = current->Parent();
    }
    return true;
}

EncounterData encounter_data_default(Eigen::Vector2d position, double time, double speed, std::shared_ptr<Obstacle> obs_ptr, VxId vx)
{
    EncounterData data;
    data.position = position;
    data.time = chrono::duration<double>(time);
    data.approachingSpeed = speed;
    data.obs_ptr = obs_ptr;
    data.vx = vx;
    return data;
}

BoundingBoxData bb_data_default()
{
    BoundingBoxData bb_data;
    bb_data.dim_x = 10;
    bb_data.dim_y = 3;

    bb_data.gain = 3;

    bb_data.minDistFromObs = 1;
    bb_data.reductionWhileCheckingPath = 1; // TODO this could depend only on the ASV capability of staying on the predicted course?
    bb_data.safeMaxGap = 0;
    bb_data.lookAheadSafetySpan = 0; // seconds

    bb_data.Set({ 0, 0 }, "", 0, 0, 0, 0, 0, 0, 0);
    return bb_data;
}

VehicleData vh_data_default()
{
    VehicleData vh_data;
    vh_data.velocities = { 10, 1 };
    return vh_data;
}

void PrintVxs(std::vector<Vx> vxs)
{
    if (!ds->printComputedVxs)
        return;
    for (const auto& vx : vxs) {
        cerr << vx.first << ": " << vx.second.transpose() << "\n";
    }
};

double GetRandomInRange(double min, double max)
{
    std::random_device r;
    std::default_random_engine e1(r());
    std::uniform_real_distribution<double> rand(min, max);
    return rand(e1);
}