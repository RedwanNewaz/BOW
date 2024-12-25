#include "CollisionChecker.h"
#include <Eigen/Geometry>
namespace mbow{
    CollisionChecker::CollisionChecker(const ParamPtr& pm): CollisionCheckerBase(pm) {
        init();
    }


    CCPtr CollisionChecker::getSharedPtr()
    {
        return shared_from_this();
    }

    bool CollisionChecker::isCollision(const std::vector<std::array<double, 5>> &trajectory) const {
        for(int i = trajectory.size() - 1; i >= 0; i--)
        {
            auto point = trajectory[i];
            if(!withinBoundary(point))
                return true;
            int X = static_cast<int>(point[0] / _grid_step);
            int Y = static_cast<int>(point[1] / _grid_step);
            if(_obstacles_set.find({X, Y}) != _obstacles_set.end())
                return true;
        }
        return false;
    }

    void CollisionChecker::addToObstaclesSet(const std::vector<float> &obs, float robotRadius, float obsLen) {
        std::vector<Point> rectangle_vertices = {
                {obs[0] - obsLen, obs[1] - obsLen},
                {obs[0] + obsLen, obs[1] - obsLen},
                {obs[0] + obsLen, obs[1] + obsLen},
                {obs[0] - obsLen, obs[1] + obsLen},
                {obs[0] - obsLen, obs[1] - obsLen}
        };

        _grid_step = robotRadius ;
        Point circle_center(0, 0);
        MinkowskiSumComputer computer(circle_center, robotRadius, rectangle_vertices, _grid_step);

        for(auto& p: computer.computeCoordinatesInMinkowskiSum())
        {
            int X = static_cast<int>(p.x() / _grid_step);
            int Y = static_cast<int>(p.y() / _grid_step);
            _obstacles_set.insert({X, Y});
        }
    }

    void CollisionChecker::initCollisionCheker(const std::vector<std::vector<float>> &obsList, float robotRadius,
                                               float obsLen) {
        for(auto& obs: obsList)
        {
            addToObstaclesSet(obs, robotRadius, 2 * obsLen);
        }

    }
}
