#include <iostream>
#include <iterator>
#include <fstream>
#include <sstream>
#include "param_manager.h"
#include "CollisionChecker.h"
#include "FclCollisionChecker.h"
#include "BOW.h"

int main(int argc , char *argv[]) {
    assert(argc == 2);
    auto pm(std::make_shared<param_manager>(argv[1]));
    auto cc(std::make_shared<mbow::CollisionChecker>(pm));

    // configure BOW planner
    // configure BOW planner
    auto start = pm->get_ndarray<double>("start");
    std::vector<mbow::State> s0;
    s0.reserve(start.size());
    for(auto& state: start)
    {
        s0.push_back({state[0], state[1], state[2], 0.0, 0.0});
    }

    // execute bow planner

    bool terminate = false;

    while (!terminate)
    {
        mbow::BOPlanner planner(s0, cc->getSharedPtr(), pm->getSharedPtr());
        auto [sol, trajs] = planner.solve(2.0);
        if(sol)
        {

            for (int i = 0; i < trajs.size(); ++i) {
                std::stringstream ss;
                for(auto& state: trajs[i])
                {
                    std::copy(state.begin(), state.end()-1, std::ostream_iterator<double>(ss, ", "));
                    ss << state.back() << "\n";
                }

                std::ofstream myfile(std::to_string(i + 1) + "_result.csv");
                myfile << ss.str();
                myfile.close();
            }

        }
        terminate = sol;
    }

    return 0;
}
