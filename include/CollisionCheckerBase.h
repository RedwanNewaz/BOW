//
// Created by airlab on 12/25/24.
//

#ifndef MBOW_COLLISIONCHECKERBASE_H
#define MBOW_COLLISIONCHECKERBASE_H

#include <vector>
#include <array>
#include <memory>
#include "param_manager.h"

namespace mbow{
    class CollisionCheckerBase : public std::enable_shared_from_this<CollisionCheckerBase> {
    public:
        using CCPtr = std::shared_ptr<CollisionCheckerBase>;
        CollisionCheckerBase(const ParamPtr& pm) : _pm(pm) {

        }
        virtual bool isCollision(const std::vector<std::array<double, 5>>& trajectory) const = 0;
        virtual void initCollisionCheker(const std::vector<std::vector<float>>& obsList, float robotRadius, float obsLen) = 0;
        void init(){
            auto obsList = _pm->get_ndarray<float>("obstacles");
            auto robotRadius = _pm->get_param<float>("robot_radius");
            float obsLen = _pm->get_param<float>("obstacle_length");
            _bounds = _pm->get_param<std::vector<float>>("boundary");
            // invoke a virtual function with following parameters
            initCollisionCheker(obsList, robotRadius, obsLen);
        }
        CCPtr getSharedPtr()
        {
            return shared_from_this();
        }

        bool withinBoundary(const std::array<double, 5>& point) const {
            return point[0] >= _bounds[0] && point[0] <= _bounds[1] && point[1] >= _bounds[2] && point[1] <= _bounds[3];
        }
    protected:
        ParamPtr _pm;
        std::vector<float> _bounds;

    };
    using CCPtr = std::shared_ptr<CollisionCheckerBase>;
}

#endif //MBOW_COLLISIONCHECKERBASE_H
