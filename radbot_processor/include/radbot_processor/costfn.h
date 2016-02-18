/*
 * costfn.h
 *
 *  Created on: Jan 27, 2016
 *      Author: mike
 */

#ifndef INCLUDE_RADBOT_PROCESSOR_COSTFN_H_
#define INCLUDE_RADBOT_PROCESSOR_COSTFN_H_

#include "radbot_processor/util.h"
#include <math.h>
#include <vector>

class costfn {
public:
    inline costfn(std::vector<sample> readings) :
            obs_(readings) {
    }
    inline costfn() {}
    //space for costfn with map for raytracing.

    inline double operator()(std::vector<double> predict) {
        ROS_ASSERT(obs_.size()>0);
        int num_src = predict.size() / 3;
        int num_obs = obs_.size();
        double cost = 0;

        std::vector<double> intAt(obs_.size(), 0);
        double radius = 0;

        for (int i = 0; i < num_obs; i++) { //for each reading
            for (int j = 0; j < num_src; j++) { //for each src
                radius = sqrt(
                        pow(obs_[i].x - predict[j * 3], 2)
                                + pow(obs_[i].y - predict[j * 3 + 1], 2));
                intAt[i] += predict[j * 3 + 2] / pow(radius, 2);
            }
        }
        for (int i = 0; i < num_obs; i++) {
            cost += pow(intAt[i] - obs_[i].counts, 2);
        }
        return pow(cost / num_obs, 0.5);
    }

    inline void addSample(sample samp) {
        obs_.push_back(samp);
    }
private:
    std::vector<sample> obs_;

};

#endif /* INCLUDE_RADBOT_PROCESSOR_COSTFN_H_ */
