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

    BOPlanner::BOPlanner(const State& x, const Point& goal, const CCPtr & cc, const ParamPtr & pm)
            : goal_(goal)
            , x_(x)
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
    }


    Eigen::VectorXd BOPlanner::operator()(const Eigen::VectorXd& u) const {
        Eigen::VectorXd res(2);

        // limbo sample u in [0, 1] range which needs to map/scale to the robot's vel domains
        Eigen::Vector2d uu = scaledU(u);
        Control act{uu[0], uu[1]};

        // feed forward trajectory for a sampled control for a given predict_time
        Traj traj = calcTrajectory(x_, uu(0), uu(1),  goal_);
        auto[index, goalDist] = calcToGoalCost(traj, goal_);
        // erase the part of trajectory that does not help to reach goal location
        traj.erase(traj.begin() + index + 1, traj.end());

        res(1) = 1.0 - static_cast<float>(cc_->isCollision(traj));
        res(0) = (res(1) > 0.0) ? 1e3 * std::exp(-goalDist) : -1.0e3;
        return res;
    }

    Eigen::Vector2d BOPlanner::scaledU(const Eigen::VectorXd& u) const {
        Eigen::Vector2d uu;
        uu(0) = min_speed_ + (max_speed_ - min_speed_) * u(0);
        // yaw angle is symmetric (there is no min_yaw rate)
        uu(1) = -max_yawrate_ + 2 * max_yawrate_ * u(1);
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


    Traj BOPlanner::computeControl(Control& u) {
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
        mbow::Traj traj;
        do{
            opt.optimize(*this);
            auto uu = opt.best_sample();
            auto v = scaledU(uu);
            traj = calcTrajectory(x_, v(0), v(1),  goal_);
        }while(cc_->isCollision(traj));
        return traj;
    }

    std::pair<bool, Traj> BOPlanner::solve(double time, bool verbose) {

        auto terminate = [&](const State& x)
        {
            auto dx = x[0] - goal_[0];
            auto dy = x[1] - goal_[1];
            auto dist = sqrt(dx * dx + dy * dy);
            return dist < goal_radius_;
        };

        Traj result;
        State current = x_;
        result.emplace_back(current);

        // Excute planner and Record end time
        auto start_time = std::chrono::high_resolution_clock::now();
        bool solution_found = true;
        double elapsed_time;

        do{
            BOPlanner mpc(current, goal_, cc_->getSharedPtr(), pm_->getSharedPtr());
            Control act;
            auto traj = mpc.computeControl(act);

            // update current state using state transition function
            if(!traj.empty())
            {
//                 instead of one step, we can use preferred speed
                int N = std::min((int) traj.size() - 1, pref_speed_index_);
                current = motion(current, act, dt_);
                current = traj[N];
                std::copy(traj.begin() + 1, traj.begin() + N, std::back_inserter(result));

//                std::copy(traj.begin() + 1, traj.end(), std::back_inserter(result));
//                current = traj.back();
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            // Calculate duration
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
            // Output duration in microseconds
            elapsed_time = (double) duration.count() / 1.0e6;
            if(elapsed_time > time)
            {
                solution_found = false;
                break;
            }

        } while (!terminate(current));

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