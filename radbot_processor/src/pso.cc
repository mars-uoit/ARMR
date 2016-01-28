/*
 * pso.cc
 *
 *  Created on: Jan 25, 2016
 *      Author: mike
 */
#include "radbot_processor/pso.h"

pso::pso(const costfn& cost_fn, sample mins, sample maxs,
        unsigned int particles, unsigned int iter, unsigned int sources) :
        cost_(cost_fn), min_(mins), max_(maxs), n_particles_(particles), n_iter_(
                iter), sources_(sources) {
    n_vars_ = 3 * sources_;
    srand(time(NULL));

    neigh_.assign(n_particles_ * 4, -1);

    //Find neighbors
    int dim = floor(n_particles_ ^ .5);
    bool one_more = (n_particles_ % dim) > 0;
    for (int i = 0; i < n_particles_; i++) {
        int r = i / dim;
        int c = i % dim;
        if (r > 0)
            neigh_[ndx(i, 0, 4)] = ndx(r - 1, c, dim);
        if (c > 0)
            neigh_[ndx(i, 1, 4)] = ndx(r, c - 1, dim);
        if ((r + 1) < (dim + one_more)) {
            if (ndx(r + 1, c, dim) < n_particles_)
                neigh_[ndx(i, 2, 4)] = ndx(r + 1, c, dim);
        }
        if ((c + 1) < dim) {
            if (ndx(r, c + 1, dim) < n_particles_)
                neigh_[ndx(i, 3, 4)] = ndx(r, c + 1, dim);
        }
    }
}
pso::~pso() {

}

void pso::run() {

    v_.assign(n_particles_ * n_vars_, 0);
    particles_.assign(n_particles_ * n_vars_, 0);
    pbest_.assign(n_particles_ * n_vars_, 0);
    gbest_.assign(n_vars_, 0);
    pmin_.assign(n_particles_, 0);

    //initialize randomly
    for (int p = 0; p < n_particles_; p++) {
        for (int i = 0; i < sources_; i++) {
            particles_[ndx(p, i * 3)] = min_.x
                    + (max_.x - min_.x) * (double) rand() / RAND_MAX;
            particles_[ndx(p, i * 3 + 1)] = min_.y
                    + (max_.y - min_.y) * (double) rand() / RAND_MAX;
            particles_[ndx(p, i * 3 + 2)] = min_.counts
                    + (max_.counts - min_.counts) * (double) rand() / RAND_MAX;
            //cerr << particles_[ndx(p, i * 3 + 2)] << endl;
        }
    }
    pbest_ = particles_;

    //initial run through cost fn find the global best.
    pmin_[0] = cost_(GET_ROW(0));
    gmin_ = pmin_[0];
    gbest_ = GET_ROW(1);
    for (int i = 1; i < n_particles_; i++) {
        pmin_[i] = cost_(GET_ROW(i));
        if (pmin_[i] < gmin_) {
            gmin_ = pmin_[i];
            gbest_ = GET_ROW(i);
        }
    }
}
