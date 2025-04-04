#ifndef PATH_EVALUATOR_HPP
#define PATH_EVALUATOR_HPP

#include "oal/data_structs.hpp"
#include "oal/geometric_utilities.hpp"
#include "oal/local_planner/node.hpp"
#include "oal/obstacle.hpp"

#define ZERO_VELOCITY 0.01

namespace oal {

class PathEvaluator {
private:
    static std::vector<std::string> highPriorityObstacles;
    static bool isHPListSet;

    // Check if an obstacle class has higher priority
    static bool HasHigherPriority(const std::string& obsClass);

public:
    static void SetHighPriorityObstacles(const std::vector<std::string>& obstacles);

    static std::string EncounterType(double angle);

public:
    static bool CollisionWithObs(const NodePtr& start, const NodePtr& goal, const ObsPtr& obs, std::vector<Eigen::Vector3d>& collisions, bool checkingPath = false, bool colregs = false);

    static bool RuleCompliantMotion(const NodePtr& start, NodePtr& goal);
};

}
#endif // PATH_EVALUATOR_HPP