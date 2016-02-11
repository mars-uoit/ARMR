/*
 * main.cc
 *
 *  Created on: Jun 26, 2015
 *      Author: hosmar
 */
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <ctime>
using namespace std;
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "radbot_processor/util.h"
#include "radbot_processor/pso.h"

#define INFLATE 0.15

vector<sample> measurements;
sample max_val, min_val;

ifstream infile;

void
openFile();

int main(int argc, char **argv) {
    ros::init(argc, argv, "radbot_processor");
    ros::NodeHandle nh;

    openFile();

    costfn my_cost(
            vector<sample>(measurements.begin(), measurements.begin() + 11));
    //costfn my_cost(measurements);
    pso my_pso(my_cost, min_val, max_val, 100, 5000, 2);
    std::vector<double> result;

    std::ostream_iterator<double> out_it(std::cout, ", ");

    clock_t t0 = clock(), t1;
    for (vector<sample>::iterator i = measurements.begin() + 11;
            i != measurements.end(); i++) {
        my_cost.addSample(*i);
        my_pso.setCostFn(my_cost);
        result = my_pso.run();
        std::copy(result.begin(), result.end(), out_it);
        cout << endl;
    }
    t1 = clock() - t0;
    cerr << (float) t1 / CLOCKS_PER_SEC << endl;

    std::vector<tf::StampedTransform> locations;
    //add in service listeners

    tf::TransformListener listener;

//    while (ros::ok()) {
//
//        ros::spinOnce();
//    }

}

inline void openFile() {
    try {
        infile.open("datawval.csv", ifstream::in);
    } catch (ifstream::failure * e) {
        cerr << "Exception opening/reading/closing file\n";
    }
    if (infile.is_open()) {
        while (infile.good()) {
            string line;
            getline(infile, line);
            if (infile.eof())
                break;

            stringstream ss(line);
            sample reading;
            reading.x = 0.0;
            reading.y = 0.0;
            reading.counts = 0.0;
            try {
                string num;
                getline(ss, num, ',');
                reading.x = atof(num.c_str());
                num = "";

                getline(ss, num, ',');
                reading.y = atof(num.c_str());
                num = "";

                getline(ss, num, ',');
                reading.counts = atof(num.c_str());
                cout << "x val: " << reading.x << " y val: " << reading.y
                        << " counts: " << reading.counts << endl;
            } catch (ifstream::failure * e) {
                cerr << "Exception opening/reading/closing file\n" << e->what();
            }
            measurements.push_back(reading);
        }
        infile.close();
    } else {
        cerr << "Error opening file";
    }
    cerr << "Size of point list:" << measurements.size() << endl;

    vector<sample>::iterator itr;
    itr = max_element(measurements.begin(), measurements.end(), cmpX);
    max_val.x = (*itr).x;
    itr = max_element(measurements.begin(), measurements.end(), cmpY);
    max_val.y = (*itr).y;
    itr = min_element(measurements.begin(), measurements.end(), cmpX);
    min_val.x = (*itr).x;
    itr = min_element(measurements.begin(), measurements.end(), cmpY);
    min_val.y = (*itr).y;
    double inflate;
    inflate = ((max_val.x - min_val.x) / 2) * INFLATE;
    min_val.x -= inflate;
    max_val.x += inflate;
    inflate = ((max_val.y - min_val.y) / 2) * INFLATE;
    min_val.y -= inflate;
    max_val.y += inflate;

    min_val.counts = 0;
    max_val.counts = 10000000;
    cerr << "Max Vals:" << max_val << endl << "Min Vals:" << min_val << endl;
}
