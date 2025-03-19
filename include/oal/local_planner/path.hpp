#ifndef ASTAR_PATH_HPP
#define ASTAR_PATH_HPP

#include <eigen3/Eigen/Eigen>
#include <memory>
#include <list>

#include "oal/local_planner/node.hpp"

namespace oal{

class AstarPath{

    std::list<NodePtr> data_;

public:

    auto Data() -> std::list<NodePtr>& {return data_;};

    double Length(Eigen::Vector2d position) const{
        double l;
        for(const auto& wp : data_){
            l += (wp->data.position - position).norm();
            position = wp->data.position;
        }
        return l;
    }

    void Print() const{
        for(const auto& node : data_){
            node->Print();
        }
    }

    json LogTrace() const {
        if(data_.empty()) return json::array();
        return data_.back()->LogTrace();
    }

};

}
#endif //ASTAR_PATH_HPP