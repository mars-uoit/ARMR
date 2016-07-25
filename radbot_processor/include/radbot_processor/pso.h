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
#include <algorithm>
#include "radbot_processor/util.h"
#include "radbot_processor/costfn.h"
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real_distribution.hpp>
#include "ros/ros.h"

class pso
{

public:
    pso(const costfn& cost_fn, sample mins, sample maxs, unsigned int particles,
        unsigned int iter, unsigned int sources);
    ~pso();
    std::vector<double>
    run();
    inline void setCostFn(const costfn& cost_fn) {
        cost_ = cost_fn;
    }

    void setSources(unsigned int sources) {
        sources_ = sources;
        n_vars_ = 3 * sources_;
        gbest_.assign(n_vars_, 0);
        gmin_ = 100000000;
    }

    void setBounds(const sample &maxs, const sample &mins) {
        min_ = mins;
        max_ = maxs;
    }
    void setParticles(unsigned int particles) {
        n_particles_ = particles;
        stop_top_ = 0.1 * particles;
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
        ROS_INFO_STREAM("PSO: Changed Particles.");
    }
    double getGMin() {
        return gmin_;
    }

private:
    void
    loop();

    static const double kC1 = 1.49;
    static const double kC2 = 1.49;
    static const double kW = 0.72;
    static const double kStopVal = .02;
    int stop_top_;

    boost::random::mt19937 rng_;
    boost::random::uniform_real_distribution<double> uniform_;

    costfn cost_;
    sample min_, max_;
    unsigned int n_particles_, n_iter_, n_vars_, sources_;

    std::vector<double> v_, particles_, pbest_, pmin_, gbest_;
    double gmin_;
    std::vector<int> neigh_;

#define GET_ROW(x, vect)(std::vector<double>(vect.begin()+x*n_vars_,vect.begin()+(x+1)*n_vars_))

    size_t ndx(int r, int c) const {
        return c + n_vars_ * r;
    }

    size_t ndx(int r, int c, int dim) const {
        return c + dim * r;
    }

    bool sortFn(std::size_t i, std::size_t j) const {
        return (pmin_[i] < pmin_[j]);
    }
};

class sortClass
{
public:
    sortClass(std::vector<double> a) {
        vec = a;
    }
    bool operator()(std::size_t i, std::size_t j) {
        return (vec[i] < vec[j]);
    }
private:
    std::vector<double> vec;
};

#endif /* INCLUDE_RADBOT_PROCESSOR_PSO_H_ */
