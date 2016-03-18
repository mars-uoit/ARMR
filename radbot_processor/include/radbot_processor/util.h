/*
 * util.h
 *
 *  Created on: Jan 21, 2016
 *      Author: mike
 */

#ifndef INCLUDE_RADBOT_PROCESSOR_UTIL_H_
#define INCLUDE_RADBOT_PROCESSOR_UTIL_H_

#include "ros/ros.h"
#include <iostream>
#define INFLATE 0.15

using namespace std;

typedef struct sample {
    double x;
    double y;
    double counts;
} sample;

inline bool cmpX(sample a, sample b) {
    return (a.x < b.x);
}

inline bool cmpY(sample a, sample b) {
    return (a.y < b.y);
}

inline bool cmpCounts(sample a, sample b) {
    return (a.counts < b.counts);
}

inline ostream&
operator<<(ostream& os, sample a) {
    os << "X val: " << a.x << " Y val: " << a.y << " Counts: " << a.counts;
    return os;
}
inline void minimax(vector<sample> &measurements, sample * max, sample * min) {
    vector<sample>::iterator itr;
    itr = max_element(measurements.begin(), measurements.end(), cmpX);
    max->x = (*itr).x;
    itr = max_element(measurements.begin(), measurements.end(), cmpY);
    max->y = (*itr).y;
    itr = min_element(measurements.begin(), measurements.end(), cmpX);
    min->x = (*itr).x;
    itr = min_element(measurements.begin(), measurements.end(), cmpY);
    min->y = (*itr).y;
    double inflate;
    inflate = ((max->x - min->x) / 2) * INFLATE;
    min->x -= inflate;
    max->x += inflate;
    inflate = ((max->y - min->y) / 2) * INFLATE;
    min->y -= inflate;
    max->y += inflate;

    min->counts = 0;
    max->counts = 10000000;
    ROS_INFO_STREAM(
            "Max Vals:" << *max << endl << "Min Vals:" << *min << endl);
}

#endif /* INCLUDE_RADBOT_PROCESSOR_UTIL_H_ */
