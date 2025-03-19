#include <algorithm>
#include <boost/filesystem.hpp>
#include <eigen3/Eigen/Eigen>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>
using json = nlohmann::ordered_json;

#include "oal/geometric_utilities.hpp"
#include "oal/local_planner/generator.hpp"

using namespace oal;
namespace fs = boost::filesystem;

std::string avoidanceLogsPath = "/home/graal/ros2_ws/log/avoidance_logs/";

Eigen::Vector2d parseVector(const json& j);
Pose parsePose(const json& j);
BoundingBoxData parseBoundingBox(const json& j);
Vx parseVx(const json& j);
Obstacle parseObstacle(const json& j);
void parseSearchStatus(const json& j, Pose& vhPose, Eigen::Vector2d& goal, std::vector<ObsPtr>& obstacles, VehicleData& vhData, PruningParams& pp);
AStarNode parseNode(const json& j, const std::vector<ObsPtr>& knownObstacles, const NodePtr& parent = nullptr);
void parsePathInfo(const json& j, const std::vector<ObsPtr>& knownObstacles, const Eigen::Vector2d& goal, const VehicleData& vhData, AstarPath& path, std::string& info);
json parseFromFile(const std::string& avoidanceLogsPath);

int main(int argc, char* argv[])
{
    // Use provided argument as new avoidance logs path, if available.
    if (argc > 1) {
        avoidanceLogsPath = argv[1];
    }

    json j = parseFromFile(avoidanceLogsPath);

    Pose vhPose;
    Eigen::Vector2d goal;
    std::vector<ObsPtr> obstacles;
    VehicleData vhData;
    PruningParams pParams;
    DebugSettings ds;
    ds.printCurrentNode = true;
    ds.printNodeEvolutionStats = true;
    ds.printNodesStats = true;
    ds.printPath = true;
    parseSearchStatus(j.at("SearchStatus"), vhPose, goal, obstacles, vhData, pParams);

    // for (auto it = obstacles.begin(); it != obstacles.end();) {
    //     if ((*it)->Id() != "obs4") {
    //         obstacles.erase(it);
    //     } else {
    //         ++it;
    //     }
    // }

    for (const auto& obs : obstacles) {
        obs->Print();
    }

    std::string type = j.at("Type").get<std::string>();
    // new, switch, fail

    AstarPath path, oldPath, betterPath;
    std::string pathInfo, oldPathInfo, betterPathInfo;
    if (type == "switch") {
        parsePathInfo(j.at("PreviousPath"), obstacles, goal, vhData, oldPath, oldPathInfo);
        std::cout << "PreviousPath info: " << oldPathInfo << "\n";
        parsePathInfo(j.at("NewPath"), obstacles, goal, vhData, betterPath, betterPathInfo);
        std::cout << "NewPath info: " << betterPathInfo << "\n";
    } else if (type == "new") {
        parsePathInfo(j.at("NewPath"), obstacles, goal, vhData, path, pathInfo);
        std::cout << "Path info: " << pathInfo << "\n";
    } else {
        // path failed
        return 0;
    }

    Eigen::Vector2d uwp;
    auto dsPtr = std::make_shared<DebugSettings>(ds);
    auto g = Generator(vhData, pParams, dsPtr);
    // focusing of exists better path
    while (true) {
        if (type == "switch") {

            //dsPtr->printCollisionReason = true;

            std::cerr << "\n\n---Path old---\n";
            oldPath.Print();
            std::cerr << "Is this valid? --> " << (bool)g.IsPathValid(oldPath, vhPose, obstacles, uwp) << "\n";

            std::cerr << "\n---Path better---\n";
            betterPath.Print();
            std::cerr << "Is this valid? --> " << (bool)g.IsPathValid(betterPath, vhPose, obstacles, uwp) << "\n";

        } else {

            dsPtr->printCurrentNode = true;
            dsPtr->printPath = false;

            // std::vector<ObsPtr> so;
            // bool o = g.IsInBB(oal::TimeDouble(0), Eigen::Vector2d(-7.4, 0.7), obstacles, so);
            // std::cerr << "Is in bb? " << o << "\n";

            AstarPath pathNow;
            auto out = g.FindPath(vhPose, goal, obstacles, pathNow);
            std::cerr << "\n\n---Path now---\n";
            pathNow.Print();
            std::cerr << "Is this valid? --> " << (bool)g.IsPathValid(path, vhPose, obstacles, uwp) << "\n";

            std::cerr << "\n---Path from log---\n";
            path.Print();
            std::cerr << "Is this valid? --> " << (bool)g.IsPathValid(path, vhPose, obstacles, uwp) << "\n";
        }
    }

    return 0;
}

// --- Function Implementations ---

Eigen::Vector2d parseVector(const json& j)
{
    return Eigen::Vector2d(j.at("x").get<double>(), j.at("y").get<double>());
}

Pose parsePose(const json& j)
{
    Pose p(parseVector(j), j.at("heading").get<double>());
    return p;
}

BoundingBoxData parseBoundingBox(const json& j)
{
    BoundingBoxData bb;
    bb.dim_x = j["Dimensions"]["x"].get<double>();
    bb.dim_y = j["Dimensions"]["y"].get<double>();
    bb.minDistFromObs = j.at("MinDistanceFromObstacle").get<double>();
    bb.reductionWhileCheckingPath = j.at("ReductionWhileCheckingPath").get<double>();
    bb.safeMaxGap = j.at("SafeMaxGap").get<double>();
    bb.lookAheadSafetySpan = j.at("LookAheadSafetySpan").get<double>();
    bb.max_x_bow = j["MaxSize"]["Bow"].get<double>();
    bb.max_x_stern = j["MaxSize"]["Stern"].get<double>();
    bb.max_y_starboard = j["MaxSize"]["Starboard"].get<double>();
    bb.max_y_port = j["MaxSize"]["Port"].get<double>();
    bb.safety_x_bow = j["SafetySize"]["Bow"].get<double>();
    bb.safety_x_stern = j["SafetySize"]["Stern"].get<double>();
    bb.safety_y_starboard = j["SafetySize"]["Starboard"].get<double>();
    bb.safety_y_port = j["SafetySize"]["Port"].get<double>();
    return bb;
}

Vx parseVx(const json& j)
{
    VxId id = StringToVxId(j.at("VxID").get<std::string>());
    Vx vx(id, Eigen::Vector2d(j["Position"]["x"].get<double>(), j["Position"]["y"].get<double>()));
    return vx;
}

Obstacle parseObstacle(const json& j)
{
    std::string id = j.at("ObstacleID").get<std::string>();
    std::string obsClass = j.at("Class").get<std::string>();
    Pose pose = parsePose(j.at("InitialPose"));
    auto velocity = parseVector(j.at("Velocity"));
    auto obsFrame_vhPos = parseVector(j.at("VehicleInObstacleFrame"));
    BoundingBoxData bbData = parseBoundingBox(j.at("BoundingBox"));
    std::cerr << "[WARNING] currently no using vxs from json file.\n";
    std::vector<Vx> currentVxs;
    for (const auto& vxJson : j.at("Vxs")) {
        currentVxs.push_back(parseVx(vxJson));
    }
    Obstacle obs(id, obsClass, pose, velocity, bbData, obsFrame_vhPos);
    return obs;
}

void parseSearchStatus(const json& j, Pose& vhPose, Eigen::Vector2d& goal, std::vector<ObsPtr>& obstacles, VehicleData& vhData, PruningParams& pp)
{
    goal = parseVector(j.at("Goal"));
    vhPose = parsePose(j.at("VehiclePose"));
    obstacles.clear();
    for (const auto& obsJson : j.at("Obstacles")) {
        obstacles.push_back(std::make_shared<Obstacle>(parseObstacle(obsJson)));
    }

    auto vhDataJson = j.at("VehicleData");
    for (const auto& vel : vhDataJson.at("Velocities")) {
        vhData.velocities.push_back(vel.get<double>());
    }
    // vhData.max_yaw_rate = j["VehicleData"]["MaxYawRate"].get<double>();

    // TODO
    //  pp.onlyOnceOnSameVx = j["PruningParams"]["OnlyOnceOnSameVx"].get<bool>();
    //  pp.stopSearchIfGoalInBB = j["PruningParams"]["StopSearchIfGoalInBB"].get<bool>();
    //  pp.samePositionThreshold = j["PruningParams"]["SamePositionThreshold"].get<double>();
    //  pp.sameTimeThreshold = j["PruningParams"]["SameTimeThreshold"].get<double>();
}

AStarNode parseNode(const json& j, const std::vector<ObsPtr>& knownObstacles, const NodePtr& parent)
{
    EncounterData eData;
    eData.position = parseVector(j.at("Position"));
    eData.heading = j.at("Heading").get<double>();
    eData.time = std::chrono::duration<double>(j.at("Time").get<double>());
    eData.approachingSpeed = j.at("Speed").get<double>();
    eData.obs_ptr = nullptr;
    for (const auto& obs : knownObstacles) {
        if (obs->Id() == j.at("Obstacle").get<std::string>()) {
            eData.obs_ptr = obs;
            eData.vx = StringToVxId(j.at("Vx").get<std::string>());
        }
    }
    AStarNode node(eData, parent);
    return node;
}

void parsePathInfo(const json& j, const std::vector<ObsPtr>& knownObstacles, const Eigen::Vector2d& goal, const VehicleData& vhData,
    AstarPath& path, std::string& info)
{

    std::ostringstream oss;
    // Check if "Path" is empty before processing
    if (j.at("Path").empty()) {
        oss << "Path is empty. No valid path generated";
    } else {
        NodePtr parent = nullptr;
        for (auto it = j.at("Path").rbegin(); it != j.at("Path").rend(); ++it) {
            auto node = std::make_shared<AStarNode>(parseNode(*it, knownObstacles, parent));
            path.Data().push_back(node);
            parent = node;
            parent->SetCosts(vhData, goal);
        }
    }

    // oss << "Colregs: " << j.at("Colregs").get<bool>()
    //     << ", PathCreationTime: " << j.at("PathCreationTime").get<long long>()
    oss << ", SearchResult: " << j.at("SearchResult").get<std::string>()
        << ", FailMsg: " << j.at("FailMsg").get<std::string>();
    info = oss.str();
}

json parseFromFile(const std::string& avoidanceLogsPath)
{
    // List folders (log directories) in the avoidanceLogsPath
    std::vector<fs::directory_entry> folders;
    try {
        fs::path basePath(avoidanceLogsPath);
        if (!fs::exists(basePath) || !fs::is_directory(basePath)) {
            std::cerr << "Invalid path: " << avoidanceLogsPath << std::endl;
            return 1;
        }
        for (fs::directory_iterator itr(basePath); itr != fs::directory_iterator(); ++itr) {
            if (fs::is_directory(itr->path())) {
                folders.push_back(*itr);
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error reading directory: " << e.what() << std::endl;
        return 1;
    }

    if (folders.empty()) {
        std::cerr << "No folders found in " << avoidanceLogsPath << std::endl;
        return 1;
    }

    // Sort folders by last modification time (newer first)
    std::sort(folders.begin(), folders.end(), [](const fs::directory_entry& a, const fs::directory_entry& b) {
        return fs::last_write_time(a.path()) > fs::last_write_time(b.path());
    });

    // Display folder menu
    std::cout << "Select a folder:" << std::endl;
    for (size_t i = 0; i < folders.size(); ++i) {
        std::cout << i + 1 << ". " << folders[i].path().filename().string() << std::endl;
    }

    size_t folderChoice = 0;
    std::cin >> folderChoice;
    if (folderChoice < 1 || folderChoice > folders.size()) {
        std::cerr << "Invalid folder selection." << std::endl;
        return 1;
    }
    fs::path selectedFolder = folders[folderChoice - 1].path();

    // List JSON files in the selected folder
    std::vector<fs::directory_entry> jsonFiles;
    try {
        for (fs::directory_iterator itr(selectedFolder); itr != fs::directory_iterator(); ++itr) {
            if (fs::is_regular_file(itr->path()) && itr->path().extension() == ".json") {
                jsonFiles.push_back(*itr);
            }
        }
    } catch (const fs::filesystem_error& e) {
        std::cerr << "Error reading folder: " << e.what() << std::endl;
        return 1;
    }

    if (jsonFiles.empty()) {
        std::cerr << "No JSON files found in " << selectedFolder.string() << std::endl;
        return 1;
    }

    // Sort JSON files by last modification time (newer first)
    std::sort(jsonFiles.begin(), jsonFiles.end(), [](const fs::directory_entry& a, const fs::directory_entry& b) {
        return fs::last_write_time(a.path()) > fs::last_write_time(b.path());
    });

    // Display JSON file menu
    std::cout << "Select a JSON file:" << std::endl;
    for (size_t i = 0; i < jsonFiles.size(); ++i) {
        std::cout << i + 1 << ". " << jsonFiles[i].path().filename().string() << std::endl;
    }

    size_t fileChoice = 0;
    std::cin >> fileChoice;
    if (fileChoice < 1 || fileChoice > jsonFiles.size()) {
        std::cerr << "Invalid file selection." << std::endl;
        return 1;
    }
    fs::path selectedFile = jsonFiles[fileChoice - 1].path();
    std::string filePath = selectedFile.string();

    // Process the selected JSON file as before
    std::ifstream inFile(filePath);
    if (!inFile.is_open()) {
        std::cerr << "Error opening file: " << filePath << std::endl;
        return 1;
    }

    json j;
    inFile >> j;
    inFile.close();
    return j;
}