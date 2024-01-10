//
// Created by airlab on 11/22/23.
//

#ifndef WINDOWBOPLANNER_COLLISION_CHECKER_H
#define WINDOWBOPLANNER_COLLISION_CHECKER_H
#include <iostream>
#include "fcl/config.h"
#include "fcl/geometry/bvh/BVH_model.h"
#include "fcl/narrowphase/collision.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree_array.h"
#include "fcl/broadphase/default_broadphase_callbacks.h"
#include "fcl/geometry/geometric_shape_to_BVH_model.h"
#include "fcl/broadphase/broadphase_SSaP.h"

template <typename BV>
class CollisionChecker {
public:
    using S = typename BV::S;

    explicit CollisionChecker(const std::vector<std::vector<S>>& obstacles) {

        // convert obstacle points to boxed shape collision objects
        std::shared_ptr<fcl::CollisionGeometry<S>> obs_geometry(
                new fcl::Box<S>(0.750000, 0.750000, 0.750000));

        for (const auto& o: obstacles)
        {
            auto obs_collisions = new fcl::CollisionObject<S>(
                    obs_geometry, Eigen::Quaterniond(1, 0, 0, 0).matrix(),
                    Eigen::Vector3d(o[0], o[1], 0.500000));
            env.emplace_back(obs_collisions);
        }

        // initialized collision manager with static obstacles
        manager = new fcl::DynamicAABBTreeCollisionManager<S>();
        manager->registerObjects(env);
        manager->setup();

    }
    /**
     * collision checking interface
     * @param state (x, y, theta)
     * @return true if collide else false
     */
    bool collide(const std::array<S, 3>& state) const
    {
        return collision_check(state);
    }

    ~CollisionChecker() {
        delete manager;
    }

private:
    std::vector<fcl::CollisionObject<S>*> env;
    fcl::DynamicAABBTreeCollisionManager<S>* manager;

    bool collision_check(const std::array<S, 3>& state) const
    {
        std::shared_ptr<fcl::CollisionGeometry<S>> robot_geometry(
                new fcl::Box<S>(0.345, 0.5, 0.1));

        fcl::DefaultCollisionData<S> cdata;
        cdata.request.num_max_contacts = 40.0;
        fcl::CollisionObject<S> robot(
                robot_geometry, Eigen::AngleAxisd(state[2], Eigen::Vector3d(0,0,1)).matrix(),
                Eigen::Vector3d(state[0], state[1], 0.500000));
        manager->collide(&robot, &cdata, fcl::DefaultCollisionFunction);

        return cdata.result.isCollision();
    }
};
#endif //WINDOWBOPLANNER_COLLISION_CHECKER_H
