/*
 * pso.h
 *
 *  Created on: Jan 25, 2016
 *      Author: mike
 */

#ifndef INCLUDE_RADBOT_PROCESSOR_PSO_H_
#define INCLUDE_RADBOT_PROCESSOR_PSO_H_

#include <math.h>
#include <stdlib.h>
#include <vector>
#include "radbot_processor/util.h"
#include "radbot_processor/costfn.h"

class pso {

public:
    pso(const costfn& cost_fn, sample mins, sample maxs, unsigned int particles,
            unsigned int iter, unsigned int sources);
    ~pso();
    void
    run();
private:
    void
    loop();

    static const double c1 = 1.49;
    static const double c2 = 1.49;
    static const double w = 0.72;

    costfn cost_;
    sample min_, max_;
    unsigned int n_particles_, n_iter_, n_vars_, sources_;

    std::vector<double> v_, particles_, pbest_, pmin_, gbest_;
    double gmin_;
    std::vector<int> neigh_;

    size_t ndx(int r, int c, int dim = n_vars_) const {
        return c + dim * r;
    }

#define GET_ROW(x)(std::vector<double>(particles_.begin()+x,particles_.begin()+x+n_vars_))

};

#endif /* INCLUDE_RADBOT_PROCESSOR_PSO_H_ */
