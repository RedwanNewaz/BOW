#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include "param_manager.h"
#include "CollisionChecker.h"
#include "BOW.h"

int main(int argc , char *argv[]) {
    assert(argc == 2);
    auto pm(std::make_shared<param_manager>(argv[1]));
    auto cc(std::make_shared<mbow::CollisionChecker>(pm));

    // configure BOW planner
    auto start = pm->get_param<std::vector<double>>("start");
    auto goal = pm->get_param<std::vector<double>>("goal");
    mbow::State s0{start[0], start[1], start[2], 0.0, 0.0};
    mbow::Point g{goal[0], goal[1]};

    // execute bow planner
    mbow::BOPlanner planner(s0, g, cc->getSharedPtr(), pm->getSharedPtr());
    auto [sol, traj] = planner.solve(2.0);
    if(sol)
    {
        std::stringstream ss;
        for(auto& state: traj)
        {
            std::copy(state.begin(), state.end()-1, std::ostream_iterator<double>(ss, ", "));
            ss << state.back() << "\n";
        }

        std::ofstream myfile("result.csv");
        myfile << ss.str();
        myfile.close();
    }

    return 0;
}
