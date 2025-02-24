#ifndef ASTAR_PATH_HPP
#define ASTAR_PATH_HPP

#include <eigen3/Eigen/Eigen>
#include <memory>
#include <list>

#include "oal/devel/node.hpp"

namespace oal{

class AstarPath{

    std::list<AStarNode> data_;

public:

    auto Data() -> std::list<AStarNode>& {return data_;};


};

}
#endif //ASTAR_PATH_HPP