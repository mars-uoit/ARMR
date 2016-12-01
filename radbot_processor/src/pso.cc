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
                iter), sources_(sources), gmin_(100000000), stop_top_(10) {
    n_vars_ = 3 * sources_;
    rng_.seed(time(NULL));
    setParticles(n_particles_);
    gbest_.assign(n_vars_, 0);
}
pso::~pso() {

}

std::vector<double> pso::run() {
    prevmin_ = 1000000;
    int run_count = 0;
    bool stop_cond = false;
    while (run_count < kTotalRuns_) {
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
                    v_[ndx(j, p)] =
                            v_[ndx(j, p)] * kW_
                                    + kC1_ * uniform_(rng_)
                                            * (pbest_[ndx(j, p)]
                                                    - particles_[ndx(j, p)])
                                    + kC2_ * uniform_(rng_)
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
                std::partial_sort(q.begin(), q.begin() + stop_top_, q.end(),
                                  sortFn);

                for (std::vector<std::size_t>::iterator p = q.begin();
                        p != (q.begin() + stop_top_); p++) {
                    double result = 0;
                    for (int x = 0; x < n_vars_; x++) {
                        result += pow(pbest_[ndx(*p, x)] - gbest_[x], 2);
                    }
                    result = pow(result, 0.5);
                    if (result > kStopVal_)
                        break;
                    if (p == (q.begin() + stop_top_ - 1)) { //time to stop
                        ROS_INFO_STREAM(
                                "PSO: {Stop condition} cost: " << gmin_ << " iter: " << i);
                        i = n_iter_;
                        j = n_particles_;
                        stop_cond = true;
                        break;
                    }
                }
            }
            ROS_DEBUG("PSO: iter: %i", i);
        }
        if(!stop_cond){
            ROS_INFO_STREAM("PSO: {Max iter} cost: " << gmin_);
        }
        else
            stop_cond = false;
        if(gmin_< (prevmin_-.01))
        {
            run_count = 0;
            prevmin_ = gmin_;
        }
        else
            run_count++;
        ROS_INFO_STREAM("PSO: Remaining Runs: " << (kTotalRuns_-run_count));
    }
    return gbest_;
}

