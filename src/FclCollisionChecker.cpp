//
// Created by airlab on 12/25/24.
//

#include "FclCollisionChecker.h"

namespace mbow {
    FclCollisionChecker::FclCollisionChecker(const ParamPtr &pm) : CollisionCheckerBase(pm) {
        init();
    }

    bool FclCollisionChecker::isCollision(const std::vector<std::array<double, 5>> &trajectory) const {
        auto geom = std::make_shared<fcl::Sphere<float>>(_robotRadius);
        auto robot_traj_list = getCollisionObjectList(trajectory, geom);

        fcl::DynamicAABBTreeCollisionManager<float> manager2;
        manager2.registerObjects(robot_traj_list);
        manager2.setup();

        fcl::DefaultCollisionData<float> collision_data;
        manager2.collide(_manager.get(), &collision_data,  fcl::DefaultCollisionFunction);

        return collision_data.result.isCollision();
    }

    void FclCollisionChecker::initCollisionCheker(const std::vector<std::vector<float>> &obsList, float robotRadius,
                                                  float obsLen) {
        auto geom = std::make_shared<fcl::Box<float>>(obsLen, obsLen, obsLen);
        _obs_list = getCollisionObjectList(obsList, geom);
        _robotRadius = robotRadius;
        // Initialize and setup the collision manager in the constructor
        _manager = std::make_shared<fcl::DynamicAABBTreeCollisionManagerf>();
        _manager->registerObjects(_obs_list);
        _manager->setup();

    }
} // mbow