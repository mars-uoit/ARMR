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
using namespace std;
#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "radbot_processor/util.h"

#define INFLATE 0.15

vector<sample> measurements;
sample max_val, min_val;

int i = 0;

ifstream infile;

int main(int argc, char **argv) {
	ros::init(argc, argv, "radbot_processor");
	ros::NodeHandle nh;

	try {
		infile.open("data.csv", ifstream::in);
	} catch (ifstream::failure e) {
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
			} catch (ifstream::failure e) {
				cerr << "Exception opening/reading/closing file\n" << e.what();
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

	min_val.counts = 10000000;
	max_val.counts = 0;
	cerr << "Max Vals:" << max_val << endl << "Min Vals:" << min_val;

	std::vector<tf::StampedTransform> locations;
	//add in service listeners

	tf::TransformListener listener;

	while (ros::ok()) {

		ros::spinOnce();
	}

}
