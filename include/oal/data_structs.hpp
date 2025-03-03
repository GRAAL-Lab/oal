#ifndef DATASTRUCTS_HPP
#define DATASTRUCTS_HPP

#include <eigen3/Eigen/Dense>
#include <chrono>
#include <memory>
#include <vector>

namespace oal{

class AStarNode;
class Obstacle;

using NodePtr = std::shared_ptr<AStarNode>;
using ObsPtr = std::shared_ptr<Obstacle>;
using NodeSet = std::vector<NodePtr>;   
using TimeDouble = std::chrono::duration<double>;


class Pose{
private:
    // Pose in world frame
    Eigen::Vector2d position_;
    double heading_;

    Eigen::Matrix3d this2World_, world2this_;
    bool matSet = false;

public:

    Pose() = default;

    Pose(Eigen::Vector2d position, double heading) 
        : position_(position), heading_(heading)
    {
        double c = cos(heading);
        double s = sin(heading);
        this2World_ <<  c, -s, position.x(),
                        s,  c, position.y(),
                        0,  0, 1;

        world2this_ <<  c,  s, -position.x() * c - position.y() * s,
                        -s,  c,  position.x() * s - position.y() * c,
                        0,  0,  1;
        matSet = true;
    }

    auto Position() const -> const Eigen::Vector2d&{ return position_; };
    auto Heading() const -> const double&{ return heading_; };

    Eigen::Vector2d FromThis2WorldFrame(const Eigen::Vector2d& local_P) const {
        if(!matSet) throw std::runtime_error("Calling Tmatrix from non-set Pose object"); 
        return (this2World_*Eigen::Vector3d(local_P.x(), local_P.y(), 1)).head(2);
    }

    Eigen::Vector2d FromWorld2ThisFrame(const Eigen::Vector2d& world_P) const{
        if(!matSet) throw std::runtime_error("Calling Tmatrix from non-set Pose object");
        return (world2this_*Eigen::Vector3d(world_P.x(), world_P.y(), 1)).head(2);
    }

};

struct VehicleData{
    std::vector<double> velocities; //linear
    double max_yaw_rate;
};

enum VxId {
    FR = 0, // forward right
    FL = 1, // forward left
    RR = 2, // rear right
    RL = 3, // rear left
    NA = 5  // inside bb, default
};
using Vx = std::pair<VxId, Eigen::Vector2d>;
inline std::string VxIdToString(VxId id){
    if(id == 0) return "FR";
    if(id == 1) return "FL";
    if(id == 2) return "RR";
    if(id == 3) return "RL";
    if(id == 5) return "NA";
    throw std::runtime_error("Not a valid vx id");
}

struct EncounterData{
    // Describes the interception of the ASV with an obstacle vertex
    Eigen::Vector2d position;
    double heading; //this is the heading when on the node (used for heading change computation)
    
    std::chrono::duration<double> time = std::chrono::duration<double>::zero();
    double approachingSpeed{};
    std::shared_ptr<Obstacle> obs_ptr = nullptr;
    VxId vx = NA;

    //Debug
    std::string discoverySource = "";
    std::string discardReason = "";

};

// struct MotionData{
//     // Data relative to the motion between nodes, dependent on the parent node
//     double headingChange = 0;
// };

struct PruningParams{
    bool onlyOnceOnSameVx = true;
    bool colregsCompliant = false;
    bool stopSearchIfGoalInBB = true; // !! false -> infinite search || path is an aggroviglio around the goal
    double samePositionThreshold = 0.5;
    double sameTimeThreshold = 0.1;
};



namespace NodeSearch {
    namespace DiscoverySource {
        inline const std::string START = "START";   //First node
        inline const std::string EXIT_VX = "EXIT_VX";   //Starting inside a bb
        inline const std::string AIM_TO_GOAL = "AIM_TO_GOAL"; //First exploration phase is toward the goal
        inline const std::string EXPLORATION = "EXPLORATION";   //Looking to reach other obstacles/vxs
        inline const std::string CLOSEST_TO_GOAL = "CLOSEST_TO_GOAL"; //Goal is in bb
    };
    namespace DiscardReason {
        inline const std::string PREVIOUSLY_DISCARDED = "PREVIOUSLY_DISCARDED";
        inline const std::string NON_COMPLIANT = "NON_COMPLIANT";
        inline const std::string COLLISION = "COLLISION";
        inline const std::string EXPLORED = "EXPLORED";
    };
}

namespace SearchResult {
    inline const std::string FOUND = "FOUND";
    inline const std::string PARTIAL = "PARTIAL";
    inline const std::string FAIL = "FAIL";
};
struct PathReport {
    std::string result;
    std::string failMsg;
};

struct BoundingBoxData {
    // Set manually
    // obs size
    double dim_x;
    double dim_y;

    double minDistFromObs = 6;
    double reductionWhileCheckingPath = 1; // TODO this could depend only on the ASV capability of staying on the predicted course?
    double safeMaxGap = 0;
    double lookAheadSafetySpan = 1; //seconds

    // Computed 
    // bb max size
    double max_x_bow;
    double max_x_stern;
    double max_y_starboard;
    double max_y_port;
    // bb safety size
    double safety_x_bow;
    double safety_x_stern;
    double safety_y_starboard;
    double safety_y_port;


    void Set( 
        double k,
        const Eigen::Vector2d& velocity, 
        std::string obs_class, 
        double size_x_sigma,
        double size_y_sigma,
        double pose_x_sigma,
        double pose_y_sigma,
        double pose_yaw_sigma,
        double vel_x_sigma,
        double vel_y_sigma){
        //assuming errors have normal distribution

        //TODO make a standard on the message covariance (cov on lat/long or x/y? cov on yaw or orientation quaternion?)

        //safety keeps in account size and velocity error
        //max also positional error
        //yaw error should reduce the difference in lenght and width

        double min_safety_dim = reductionWhileCheckingPath + minDistFromObs;

        safety_x_bow = min_safety_dim + (dim_x + k*size_x_sigma + k*vel_x_sigma*lookAheadSafetySpan)/2;
        safety_x_stern = min_safety_dim + (dim_x + k*size_x_sigma)/2;
        //std::cerr<<" dim: "<<dim_x<<"\n safety x bow: "<<safety_x_bow<<"\n safety x stern: "<<safety_x_bow<<"\n";
        safety_y_port = safety_y_starboard = min_safety_dim + (dim_y + k*size_y_sigma + k*vel_y_sigma*lookAheadSafetySpan)/2;

        max_x_bow = safeMaxGap + safety_x_bow + k*pose_x_sigma / 2;
        max_x_stern = safeMaxGap + safety_x_stern + k*pose_x_sigma / 2;
        max_y_port = max_y_starboard = safeMaxGap + safety_y_port + k*pose_y_sigma / 2;

        //reductionWhileCheckingPath = 5; 
        
        
    }

};

}
#endif