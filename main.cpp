#define GRID_SAMPLING
#include <iostream>
#include <fstream>
#include <memory>
#include <vector>
#include <thread>
#include "include/bow_planner.h"
#include "include/matplotlibcpp.h"


namespace plt = matplotlibcpp;

void visualize(RobotModelPtr robot, const OBS_LIST& obstacles)
{
    size_t N = obstacles.size();
    std::vector<double>obsViewX(N), obsViewY(N);
    std::transform(obstacles.begin(), obstacles.end(), obsViewX.begin(), [](const std::vector<double>& val){return val[0];});
    std::transform(obstacles.begin(), obstacles.end(), obsViewY.begin(), [](const std::vector<double>& val){return val[1];});

    do {
        plt::cla();
        plt::plot(obsViewX, obsViewY, "ko");
        auto state = robot->getState();
        plt::plot(std::vector<double>{state[0]}, std::vector<double>{state[1]}, "ro");

        auto traj = robot->getTrajectory();
        plt::plot(traj.first, traj.second, "gx");


        plt::xlim(-1, 15);
        plt::ylim(-1, 15);
        plt::pause(0.01);

    } while (true);
}


int main() {
    std::cout << "Bo Planner" << std::endl;

    OBS_LIST obstacles{
            {0.0, 2.0},
            {4.0, 2.0},
            {5.0, 4.0},
            {5.0, 5.0},
            {5.0, 6.0},
            {5.0, 9.0},
            {8.0, 9.0},
            {7.0, 9.0},
            {8.0, 10.0},
            {9.0, 11.0},
            {12.0, 13.0},
            {12.0, 12.0},
            {15.0, 15.0},
            {13.0, 13.0}
    };

    std::string outfile = "/home/airlab/CLionProjects/BOW/result/path2.csv";
    std::ofstream myfile;
    myfile.open (outfile);
    auto robot = std::make_shared<RobotModel>(obstacles);
    bow_planner bow(robot, &myfile);
//    bow.run();
    auto plannerCore = std::thread(&bow_planner::run, std::ref(bow));
    visualize(robot, obstacles);
    plannerCore.join();
    return 0;
}