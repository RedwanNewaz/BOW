//
// Created by airlab on 12/25/24.
//

#ifndef MBOW_COLLISIONCHECKER_H
#define MBOW_COLLISIONCHECKER_H
#include "MinkowskiSumComputer.h"
#include "param_manager.h"
#include <unordered_set>

namespace mbow{
    // Alias for the point type
    using POINT = std::pair<int, int>;

    // Custom hash function for POINT
    struct PointHash {
        std::size_t operator()(const POINT& p) const {
            return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
        }
    };

    // Custom equality comparator for POINT (optional, default works for pairs)
    struct PointEqual {
        bool operator()(const POINT& p1, const POINT& p2) const {
            return p1.first == p2.first && p1.second == p2.second;
        }
    };

    class CollisionChecker : public std::enable_shared_from_this<CollisionChecker> {
    public:
        using CCPtr = std::shared_ptr<CollisionChecker>;

        CollisionChecker(const ParamPtr& pm);

        CCPtr getSharedPtr();

        bool isCollision(const std::vector<std::array<double, 5>>& trajectory) const;

    private:
        float _robotRadius;
        ParamPtr _pm;
        float _grid_step;
        std::unordered_set<POINT, PointHash, PointEqual> _obstacles_set;
        void addToObstaclesSet(const std::vector<float>& obsList, float robotRadius, float obsLen);

    };


    using CCPtr = std::shared_ptr<CollisionChecker>;
}

#endif //MBOW_COLLISIONCHECKER_H
