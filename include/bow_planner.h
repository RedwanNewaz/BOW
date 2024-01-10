//
// Created by airlab on 11/22/23.
//

#ifndef WINDOWBOPLANNER_BOW_PLANNER_H
#define WINDOWBOPLANNER_BOW_PLANNER_H
#include "bo_param.h"
#include "robot_model.h"


class bow_planner{

public:
    using Stop_t = boost::fusion::vector<stop::MaxIterations<Params>>;
    using Stat_t = boost::fusion::vector<stat::Samples<Params>,
            stat::BestObservations<Params>,
            stat::AggregatedObservations<Params>>;
    using Mean_t = mean::Constant<Params>;
    using Kernel_t = kernel::Exp<Params>;
    using GP_t = model::GP<Params, Kernel_t, Mean_t>;
    using Constrained_GP_t = model::GP<Params, Kernel_t, Mean_t>;

    using Acqui_t = experimental::acqui::ECI<Params, GP_t, Constrained_GP_t>;
#ifdef GRID_SAMPLING
    using Init_t = init::GridSampling<Params>;
#else
    using Init_t = init::RandomSampling<Params>;
#endif
    bow_planner(RobotModelPtr& robot, std::ofstream *outfile = nullptr):robot_(robot), outfile(outfile)
    {

    }

    ~bow_planner()
    {
        if(outfile)
        {
            outfile->close();
        }
    }

    void run()
    {
        tools::par::init();
        experimental::bayes_opt::CBOptimizer<Params,
        modelfun<GP_t>,
        acquifun<Acqui_t>,
        statsfun<Stat_t>,
        initfun<Init_t>,
        stopcrit<Stop_t>,
        experimental::constraint_modelfun<Constrained_GP_t>>
        opt;

        double dist = robot_->getRemainDist();
        int collisionCount = 0;
        while (dist >  0.5)
        {
            opt.optimize(*robot_);
            auto u = opt.best_sample();
            robot_->setNormalizedControl(u);
            dist = robot_->getRemainDist();
            bool collision = robot_->hasCollided();
            collisionCount += static_cast<int>(collision);
            if(outfile)
                *outfile << robot_->toString();
            std::cout << "[Goal Distance]: " << dist << " [Collision Count]: " << collisionCount << std::flush << "\r";
        }
        std::cout <<"\n[collision]: " << collisionCount << std::endl;
    }

private:
    RobotModelPtr robot_;
    std::ofstream *outfile;

};

#endif //WINDOWBOPLANNER_BOW_PLANNER_H
