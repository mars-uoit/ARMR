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
                iter), sources_(sources), gmin_(100000000) {
    n_vars_ = 3 * sources_;
    rng_.seed(time(NULL));

    neigh_.assign(n_particles_ * 4, -1);

    //Find neighbors
    int dim = floor(pow(n_particles_, .5));
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
    gbest_.assign(n_vars_, 0);
}
pso::~pso() {

}

std::vector<double> pso::run() {

    v_.assign(n_particles_ * n_vars_, 0);
    particles_.assign(n_particles_ * n_vars_, 0);
    pbest_.assign(n_particles_ * n_vars_, 0);
    pmin_.assign(n_particles_, 0);

    //initialize randomly
    for (int p = 0; p < n_particles_; p++) {
        for (int i = 0; i < sources_; i++) {
            particles_[ndx(p, i * 3)] = min_.x
                    + (max_.x - min_.x) * uniform_(rng_);
            particles_[ndx(p, i * 3 + 1)] = min_.y
                    + (max_.y - min_.y) * uniform_(rng_);
            particles_[ndx(p, i * 3 + 2)] = min_.counts
                    + (max_.counts - min_.counts) * uniform_(rng_);
            //cerr << particles_[ndx(p, i * 3 + 2)] << endl;
        }
    }
    //copy previous best into swarm
    std::copy(gbest_.begin(), gbest_.end(), particles_.begin());
    pbest_ = particles_;

    //initial run through cost fn find the global best.
    pmin_[0] = cost_(GET_ROW(0, particles_));
    gmin_ = pmin_[0];
    gbest_ = GET_ROW(0, particles_);

    for (int i = 1; i < n_particles_; i++) {
        pmin_[i] = cost_(GET_ROW(i, particles_));
        if (pmin_[i] < gmin_) {
            gmin_ = pmin_[i];
            gbest_ = GET_ROW(i, particles_);
        }
    }

    //main loop
    std::vector<double> lbest(n_vars_, 0);
    for (int i = 0; i < n_iter_; i++) {
        for (int j = 0; j < n_particles_; j++) {
            // find local best
            double lmin = 1000000000;
            for (int p = 0; p < 4; p++) {
                if (neigh_[ndx(j, p, 4)] >= 0) {
                    if (lmin > pmin_[neigh_[ndx(j, p, 4)]]) {
                        lbest = GET_ROW(neigh_[ndx(j, p, 4)], pbest_);
                        lmin = pmin_[neigh_[ndx(j, p, 4)]];
                    }
                }
            }
            //main velocity and position update
            for (int p = 0; p < n_vars_; p++) {
                v_[ndx(j, p)] = v_[ndx(j, p)] * kW
                        + kC1 * uniform_(rng_)
                                * (pbest_[ndx(j, p)] - particles_[ndx(j, p)])
                        + kC2 * uniform_(rng_)
                                * (lbest[p] - particles_[ndx(j, p)]);
                particles_[ndx(j, p)] += v_[ndx(j, p)];
            }
            //check bounds
            for (int p = 0; p < sources_; p++) {
                if (particles_[ndx(j, p * 3)] > max_.x)
                    particles_[ndx(j, p * 3)] = max_.x;
                if (particles_[ndx(j, p * 3 + 1)] > max_.y)
                    particles_[ndx(j, p * 3 + 1)] = max_.y;
                if (particles_[ndx(j, p * 3 + 2)] > max_.counts)
                    particles_[ndx(j, p * 3 + 2)] = max_.counts;

                if (particles_[ndx(j, p * 3)] < min_.x)
                    particles_[ndx(j, p * 3)] = min_.x;
                if (particles_[ndx(j, p * 3 + 1)] < min_.y)
                    particles_[ndx(j, p * 3 + 1)] = min_.y;
                if (particles_[ndx(j, p * 3 + 2)] < min_.counts)
                    particles_[ndx(j, p * 3 + 2)] = min_.counts;
            }
            //check for new min
            double tmin = cost_(GET_ROW(j, particles_));
            if (tmin < pmin_[j]) {
                std::copy(particles_.begin() + ndx(j, 0),
                        particles_.begin() + ndx(j + 1, 0),
                        pbest_.begin() + ndx(j, 0)); //pbest(j,:) = particles(j,:);
                pmin_[j] = tmin;
                if (pmin_[j] < gmin_) {
                    gmin_ = pmin_[j];
                    gbest_ = GET_ROW(j, pbest_);
                }
            }
            // stopping criteria (sort is costly)
            std::vector<std::size_t> q(pmin_.size());
            for (int p = 0; p < q.size(); p++) {
                q.at(p) = p;
            }
            sortClass sortFn(pmin_);
            std::partial_sort(q.begin(), q.begin() + kStopTop, q.end(), sortFn);

            for (std::vector<std::size_t>::iterator p = q.begin();
                    p != (q.begin() + kStopTop); p++) {
                double result = 0;
                for (int x = 0; x < n_vars_; x++) {
                    result += pow(pbest_[ndx(*p, x)] - gbest_[x], 2);
                }
                result = pow(result, 0.5);
                if (result > kStopVal)
                    break;
                if (p == (q.begin() + kStopTop - 1)) { //time to stop
                    cout << "cost: " << gmin_ << " iter: " << i << std::endl;
                    return gbest_;
                }
            }
        }
        ROS_DEBUG("iter: %i", i);
    }
    cout << "cost: " << gmin_ << std::endl;
    return gbest_;
}

