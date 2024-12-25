//
// Created by airlab on 12/25/24.
//

#ifndef MBOW_FCLCOLLISIONCHECKER_H
#define MBOW_FCLCOLLISIONCHECKER_H
#include <vector>
#include <array>
#include <memory>
#include <fcl/fcl.h>
#include "param_manager.h"
#include "CollisionCheckerBase.h"
namespace mbow {

    class FclCollisionChecker : public CollisionCheckerBase {
    public:
        FclCollisionChecker(const ParamPtr& pm);


        bool isCollision(const std::vector<std::array<double, 5>>& trajectory) const override;

        void
        initCollisionCheker(const std::vector<std::vector<float>> &obsList, float robotRadius, float obsLen) override;

    private:
        float _robotRadius;
        std::vector<fcl::CollisionObject<float>*> _obs_list;
        std::shared_ptr<fcl::BroadPhaseCollisionManager<float>> _manager;

        template <typename OBJ_LIST, typename GEOM>
        std::vector<fcl::CollisionObject<float>*> getCollisionObjectList(const OBJ_LIST& obstacles, const std::shared_ptr<GEOM>& geom) const
        {
            std::vector<fcl::CollisionObject<float>*> obs_list;
            obs_list.reserve(obstacles.size());

            for (const auto& o : obstacles) {
                fcl::Transform3f pose = fcl::Transform3f::Identity();
                pose.linear() = Eigen::Quaternionf::Identity().matrix();
                pose.translation() = Eigen::Vector3f(float(o[0]), float(o[1]), 0.5f);

                obs_list.emplace_back(new fcl::CollisionObject<float>(geom, pose));
            }

            return obs_list;
        }


    };

} // mbow

#endif //MBOW_FCLCOLLISIONCHECKER_H
