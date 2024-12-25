//
// Created by redwan on 11/9/24.
//

#ifndef MCTS_BOW_COLLISIONCHECKER_H
#define MCTS_BOW_COLLISIONCHECKER_H
#include <vector>
#include <array>
#include <memory>
#include <unordered_set>
#include "param_manager.h"
#include "CollisionCheckerBase.h"
#include "MinkowskiSumComputer.h"

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

    class CollisionChecker : public CollisionCheckerBase {
    public:
        CollisionChecker(const ParamPtr& pm);

        CCPtr getSharedPtr();

        bool isCollision(const std::vector<std::array<double, 5>>& trajectory) const override;

        void
        initCollisionCheker(const std::vector<std::vector<float>> &obsList, float robotRadius, float obsLen) override;

    private:
        float _grid_step;
        std::unordered_set<POINT, PointHash, PointEqual> _obstacles_set;
        void addToObstaclesSet(const std::vector<float>& obsList, float robotRadius, float obsLen);

    };
}

#endif //MCTS_BOW_COLLISIONCHECKER_H
