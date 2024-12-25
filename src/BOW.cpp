#include "BOW.h"
#include "bow_param.h"
#include <cmath>
#include <algorithm>
#include <limits>
#include <boost/fusion/include/vector.hpp>
#include <limbo/tools/macros.hpp>
#include <limbo/tools/parallel.hpp>
#include <limbo/experimental/bayes_opt/cboptimizer.hpp>

namespace mbow {

    BOPlanner::BOPlanner(const std::vector<State>& x, const CCPtr & cc, const ParamPtr & pm)
            : x_(x)
            , cc_(cc)
            , pm_(pm)
    {

        max_speed_      = pm_->get_param<double>("max_speed");
        min_speed_      = pm_->get_param<double>("min_speed");
        max_yawrate_    = pm_->get_param<double>("max_yawrate");
        dt_             = pm_->get_param<double>("dt");
        goal_radius_    = pm_->get_param<double>("goal_radius");
        robot_radius_   = pm_->get_param<double>("robot_radius");
        predict_time_   = pm_->get_param<double>("predict_time");
        pref_speed_index_ = pm_->get_param<double>("pref_speed_index");
        auto goal         = pm->get_ndarray<double>("goal");
        goal_.reserve(goal.size());
        for(auto& g: goal)
        {
            goal_.emplace_back(Point{g[0], g[1]});
        }
    }

    Eigen::VectorXd BOPlanner::operator()(const Eigen::VectorXd& u) const {
        Eigen::VectorXd res(2);

        // limbo sample u in [0, 1] range which needs to map/scale to the robot's vel domains
        Eigen::VectorXd uu = scaledU(u);


        // feed forward trajectory for a sampled control for a given predict_time
        Traj traj1 = calcTrajectory(x_[0], uu(0), uu(1),  goal_[0]);
        Traj traj2 = calcTrajectory(x_[1], uu(2), uu(3),  goal_[1]);
        auto[index1, goalDist1] = calcToGoalCost(traj1, goal_[0]);
        auto[index2, goalDist2] = calcToGoalCost(traj2, goal_[1]);
        // erase the part of trajectory that does not help to reach goal location
        traj1.erase(traj1.begin() + index1 + 1, traj1.end());
        traj2.erase(traj2.begin() + index2 + 1, traj2.end());

        std::vector<Traj> trajs{traj1, traj2};
        bool CA = cc_->isCollision(traj1);
        bool CB = cc_->isCollision(traj2);
        res(1) = 1.0 - static_cast<float>(CA || CB);
        res(0) = (res(1) > 0.0) ? 1e3 * std::exp(-(goalDist1 + goalDist2)) : -1.0e3;
        return res;
    }


    Eigen::VectorXd BOPlanner::scaledU(const Eigen::VectorXd& u) const {
        Eigen::VectorXd uu(4);
        uu(0) = min_speed_ + (max_speed_ - min_speed_) * u(0);
        // yaw angle is symmetric (there is no min_yaw rate)
        uu(1) = -max_yawrate_ + 2 * max_yawrate_ * u(1);

        uu(2) = min_speed_ + (max_speed_ - min_speed_) * u(2);
        // yaw angle is symmetric (there is no min_yaw rate)
        uu(3) = -max_yawrate_ + 2 * max_yawrate_ * u(3);

        return uu;
    }

    State BOPlanner::motion(State x, Control u, double dt) const {
        x[2] += u[1] * dt;
        x[2] = std::fmod(x[2] + M_PI, 2 * M_PI) - M_PI;
        x[0] += u[0] * std::cos(x[2]) * dt;
        x[1] += u[0] * std::sin(x[2]) * dt;
        x[3] = u[0];
        x[4] = u[1];
        return x;
    }

    Traj BOPlanner::calcTrajectory(State x, double v, double y, const Point& goal) const {
        Traj traj;
        traj.reserve(static_cast<size_t>(predict_time_ / dt_));
        traj.push_back(x);

        double time = 0.0;
        double distance = std::numeric_limits<double>::max();
        const Control u{{v, y}};

        while (time <= predict_time_ && distance > goal_radius_) {
            x = motion(x, u, dt_);
            const double dx = goal[0] - x[0];
            const double dy = goal[1] - x[1];
            distance = sqrt(dx * dx + dy * dy);
            traj.push_back(x);
            time += dt_;
        }

        return traj;
    }

    std::pair<int, double> BOPlanner::calcToGoalCost(const Traj& traj, const Point& goal) const {
        double minDist = 1e7;
        int bestIndex, index = 0;
        for(auto& final_state : traj)
        {
            const double dx = goal[0] - final_state[0];
            const double dy = goal[1] - final_state[1];
            double dist = sqrt(dx * dx + dy * dy);
            if(dist < minDist)
            {
                minDist = dist;
                bestIndex = index;
            }
            ++index;
        }
        return std::make_pair(bestIndex, minDist);
    }


    Eigen::VectorXd BOPlanner::computeControl() {
        using namespace limbo;
        using Stop_t = boost::fusion::vector<stop::MaxIterations<Params>>;
        using Stat_t = boost::fusion::vector<
                stat::Samples<Params>,
                stat::BestObservations<Params>,
                stat::AggregatedObservations<Params>
        >;
//        using Mean_t = mean::Data<Params>;
        using Kernel_t = kernel::SquaredExpARD<Params>;
        using Mean_t = mean::Constant<Params>;
//        using Kernel_t = kernel::Exp<Params>;
        using GP_t = model::GP<Params, Kernel_t, Mean_t>;
        using Constrained_GP_t = model::GP<Params, Kernel_t, Mean_t>;
        using Acqui_t = experimental::acqui::ECI<Params, GP_t, Constrained_GP_t>;
        using Init_t = init::RandomSampling<Params>;

        tools::par::init();

        experimental::bayes_opt::CBOptimizer<
                Params,
                modelfun<GP_t>,
                acquifun<Acqui_t>,
                statsfun<Stat_t>,
                initfun<Init_t>,
                stopcrit<Stop_t>,
                experimental::constraint_modelfun<Constrained_GP_t>
        > opt;
        Eigen::VectorXd v;
//        opt.optimize(*this);
//        auto uu = opt.best_sample();
//        v = scaledU(uu);

        bool isCollision = true;
        int count = 500;
        do{
            opt.optimize(*this);
            auto uu = opt.best_sample();
            v = scaledU(uu);
            Traj traj1 = calcTrajectory(x_[0], uu(0), uu(1),  goal_[0]);
            Traj traj2 = calcTrajectory(x_[1], uu(2), uu(3),  goal_[1]);
            bool CA = cc_->isCollision(traj1);
            bool CB = cc_->isCollision(traj2);
            isCollision = CA || CB;
        }while(isCollision && --count > 0);
        return v;
    }

    std::pair<bool, std::vector<Traj>> BOPlanner::solve(double time, bool verbose)  {

        std::vector<Traj> result(2);

        // Excute planner and Record end time
        auto start_time = std::chrono::high_resolution_clock::now();

        double elapsed_time;
        bool solution_found = false;

        do{
            BOPlanner mpc(x_, cc_->getSharedPtr(), pm_->getSharedPtr());

            auto v = mpc.computeControl();
            auto traj1 = calcTrajectory(x_[0], v(0), v(1),  goal_[0]);
            auto traj2 = calcTrajectory(x_[1], v(2), v(3),  goal_[1]);
            std::vector<Traj> currentTrajs{traj1, traj2};

            bool goalReach[2] = {false, false};
            for (int i = 0; i < currentTrajs.size(); ++i) {
                if(!currentTrajs[i].empty())
                { // check if we reached the goal
                    double dist = std::hypot(x_[i][0] - goal_[i][0], x_[i][1] - goal_[i][1]);
                    goalReach[i] = dist < goal_radius_;
                    if(!goalReach[i])
                    {
                        // instead of one step, we can use preferred speed
                        int N = std::min((int) currentTrajs[i].size() - 1, pref_speed_index_);
                        x_[i] = currentTrajs[i][N];
                        std::copy(currentTrajs[i].begin(), currentTrajs[i].begin() + N, std::back_inserter(result[i]));
                        dist = std::hypot(x_[i][0] - goal_[i][0], x_[i][1] - goal_[i][1]);
                        goalReach[i] = dist < goal_radius_;
                    }
                }
            }
            // update current state using state transition function
            solution_found = goalReach[0] && goalReach[1];

            auto end_time = std::chrono::high_resolution_clock::now();
            // Calculate duration
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            // Output duration in microseconds
            elapsed_time = (double) duration.count() / 1.0e3;
            if(elapsed_time > time)
            {
                break;
            }

        } while (!solution_found);

        if(verbose)
        {
            if(solution_found)
                std::cout << "[BOW]: Solution found in time: " << elapsed_time << " seconds" << std::endl;
            else
                std::cout << "[BOW]: No solution found in " << elapsed_time << " seconds :-(" << std::endl;
        }

        return std::make_pair(solution_found, result);
    }

} // namespace bow