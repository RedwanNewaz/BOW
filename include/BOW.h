#pragma once
#include "bow_param.h"
#include "param_manager.h"
#include "CollisionChecker.h"
#include <vector>
#include <array>
#include <Eigen/Core>
#include <random>
#include <chrono>

namespace mbow {
    using Traj = std::vector<std::array<double, 5>>;
    using State = std::array<double, 5>;
    using Point = std::array<double, 2>;
    using Control = std::array<double, 2>;

    class BOPlanner {
    public:
        // Bayesian Optimization parameters
        BO_PARAM(size_t, dim_in, 4);
        BO_PARAM(size_t, dim_out, 1);
        BO_PARAM(size_t, nb_constraints, 1);

        // Constructor
        BOPlanner(const std::vector<State>& x, const CCPtr& cc, const ParamPtr& pm_);

        // compute optimal control for a finite planning horizon
        Eigen::VectorXd computeControl();

        //  Public interface
        std::pair<bool, std::vector<Traj>> solve(double time, bool verbose=true);

        // Operator for Bayesian optimization
        Eigen::VectorXd operator()(const Eigen::VectorXd& u) const;

    private:
        // Member variables
        std::vector<Point> goal_;
        std::vector<State> x_;
        CCPtr cc_;
        ParamPtr pm_;

        // yaml config parameters
        double max_speed_;
        double min_speed_;
        double max_yawrate_;
        double dt_;
        double goal_radius_;
        double robot_radius_;
        double predict_time_;
        int pref_speed_index_;


        // Helper functions
        Eigen::VectorXd scaledU(const Eigen::VectorXd& u) const;
        State motion(State x, Control u, double dt) const;
        Traj calcTrajectory(State x, double v, double y, const Point& goal) const;
        std::pair<int, double> calcToGoalCost(const Traj& traj, const Point& goal) const;
    };

} // namespace bow