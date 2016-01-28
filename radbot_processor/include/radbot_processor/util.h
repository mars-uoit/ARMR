/*
 * util.h
 *
 *  Created on: Jan 21, 2016
 *      Author: mike
 */

#ifndef INCLUDE_RADBOT_PROCESSOR_UTIL_H_
#define INCLUDE_RADBOT_PROCESSOR_UTIL_H_

#include <iostream>

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

#endif /* INCLUDE_RADBOT_PROCESSOR_UTIL_H_ */
