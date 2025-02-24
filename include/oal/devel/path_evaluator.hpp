#ifndef PATH_EVALUATOR_HPP
#define PATH_EVALUATOR_HPP

#include "oal/devel/geometric_utilities.hpp"
#include "oal/devel/obstacle.hpp"
#include "oal/devel/node.hpp"

#define HeadOnAngle (15*(M_PI/180))
#define OvertakingAngle (112*(M_PI/180))
#define ZERO_VELOCITY 0.01

namespace oal{
    

    class PathEvaluator{

            static bool HasHigherPriority(std::string obsClass);

        public:

            static bool CollisionWithObs(const NodePtr& start, const NodePtr& goal, const ObsPtr& obs, bool colregs = false);

            static bool RuleCompliantMotion(const NodePtr& start, NodePtr& goal);
    
            
    };


}
#endif //PATH_EVALUATOR_HPP