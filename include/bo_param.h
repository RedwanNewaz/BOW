//
// Created by airlab on 11/22/23.
//

#ifndef WINDOWBOPLANNER_BO_PARAM_H
#define WINDOWBOPLANNER_BO_PARAM_H

#define USE_NLOPT
#include <limbo/limbo.hpp>
#include <limbo/experimental/acqui/eci.hpp>
#include <limbo/experimental/bayes_opt/cboptimizer.hpp>


using namespace limbo;

struct Params {
    struct bayes_opt_cboptimizer : public defaults::bayes_opt_cboptimizer {
    };

    struct init_randomsampling {
        BO_PARAM(int, samples, 15);
    };

    struct init_gridsampling{
        BO_PARAM(int, bins, 15);
    };

    struct kernel : public defaults::kernel {
        BO_PARAM(double, noise, 0.0);
    };

    struct kernel_exp : public defaults::kernel_exp {
    };

    struct bayes_opt_bobase : public defaults::bayes_opt_bobase {
        BO_PARAM(bool, stats_enabled, false);
    };

    struct stop_maxiterations {
        BO_PARAM(int, iterations, 1);
    };

    struct acqui_eci : public defaults::acqui_eci {
    };

    struct mean_constant {
        BO_PARAM(double, constant, 0.90);
    };

#ifdef USE_NLOPT
    struct opt_nloptnograd : public defaults::opt_nloptnograd {
    };
#elif defined(USE_LIBCMAES)
    struct opt_cmaes : public defaults::opt_cmaes {
    };
#else
    struct opt_gridsearch : public defaults::opt_gridsearch {
    };
#endif
};
#endif //WINDOWBOPLANNER_BO_PARAM_H
