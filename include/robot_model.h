
#pragma once
#include "bo_param.h"
#include "collision_checker.h"

class RobotModel;
typedef std::shared_ptr<RobotModel> RobotModelPtr;
typedef std::vector<std::vector<double>> OBS_LIST;

class RobotModel: public std::enable_shared_from_this<RobotModel>{

public:
    BO_PARAM(size_t, dim_in, 2);
    BO_PARAM(size_t, dim_out, 1);
    BO_PARAM(size_t, nb_constraints, 1);

    RobotModel(const OBS_LIST& obstacles)
    {
        x.resize(state_dim);
        x << 0.0, 0.0, M_PI / 8.0, 0.0, 0.0;
        g << 10, 10;
        collisionChecker = std::make_unique<CollisionChecker<fcl::OBB<double>>>(obstacles);
        robotCollide_ = stateUpdated_ = false;
    }

    void setNormalizedControl(const Eigen::Vector2d& u)
    {
        Eigen::Vector2d uu = scaledU(u);
        auto xx = motion(x, uu, dt_);
        robotCollide_ = collisionChecker->collide({xx(0), xx(1), xx(2)});
        if(!robotCollide_)
            x = xx;
        stateUpdated_ = true;
    }

    std::vector<double> getState()
    {
        stateUpdated_ = false;
        return std::vector<double>{x(0), x(1), x(2)};
    }

    RobotModelPtr getPtr()
    {
        return shared_from_this();
    }

    bool hasCollided() const
    {
        return robotCollide_;
    }

    std::string toString() const
    {
        return std::to_string(x(0)) + "," + std::to_string(x(1)) + "\n";
    }

    Eigen::VectorXd operator()(const Eigen::VectorXd& u) const
    {
        Eigen::VectorXd res(2);
        // we _maximize in [0:1]
        Eigen::Vector2d uu = scaledU(u);
        Eigen::VectorXd xx = motion(this->x, uu, dt_);

        res(0) = exp(-goalDistance(xx));

        // testing the constraints
        // 0: infeasible 1: feasible
        res(1) = (collisionChecker->collide({xx(0), xx(1), xx(2)})) ? 0 : 1;
        double time = 0;

        if(res(1) == 1)
            while (time <= predict_time)
            {
                xx = motion(xx, uu, dt_);
                if(collisionChecker->collide({xx(0), xx(1), xx(2)}))
                {
                    res(1) = 0;
                    break;
                }
                time += dt_;
            }

        return res;
    }
    double getRemainDist()const
    {
        return goalDistance(x);
    }

    std::pair<std::vector<double>,std::vector<double>> getTrajectory()
    {
        Eigen::Vector2d u(x(3), x(4));
        double time = dt_;
        std::vector<double> X, Y;
        while (time <= predict_time)
        {
            auto xx = motion(x, u, time);
            time += dt_;
            X.push_back(xx(0));
            Y.push_back(xx(1));
        }
        return std::make_pair(X, Y);
    }

protected:
    Eigen::VectorXd motion(const Eigen::VectorXd& xx, const Eigen::Vector2d& u, double dt) const
    {
        Eigen::VectorXd nx(state_dim);

        nx(2) = xx(2) + u(1) * dt;
        nx(2) = fmod(nx(2) + M_PI, 2 * M_PI) - M_PI;

        nx(0) = xx(0) + u(0) * std::cos(nx(2)) * dt;
        nx(1) = xx(1) + u(0) * std::sin(nx(2)) * dt;
        nx(3) = u(0);
        nx(4) = u(1);
        return nx;
    }



    double goalDistance(const Eigen::VectorXd& xx) const
    {
        return sqrt((g(0) - xx(0)) * (g(0) - xx(0)) + (g(1) - xx(1)) * (g(1) - xx(1)));
    }

    Eigen::Vector2d scaledU(const Eigen::VectorXd& u)const
    {
        Eigen::Vector2d uu;
        uu(0) = -1 + 2 * u(0); // max_speed = 0.345;
        uu(1) = -1.047 + 2.094 * u(1);
        return uu;
    }

private:
    Eigen::VectorXd x; //x, y , theta, v
    Eigen::Vector2d g; //x, y
    const double dt_ = 0.1;
    const size_t state_dim = 5;
    const double predict_time = 3.0;
    std::unique_ptr<CollisionChecker<fcl::OBB<double>>> collisionChecker;
    bool robotCollide_, stateUpdated_;
};