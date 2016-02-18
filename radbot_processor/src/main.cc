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
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "ursa_driver/ursa_counts.h"
#include <actionlib/server/simple_action_server.h>
#include "radbot_processor/sampleAction.h"
#include "radbot_processor/util.h"
#include "radbot_processor/pso.h"

#define INFLATE 0.15

ros::MultiThreadedSpinner spinner(4);

vector<sample> measurements;
sample max_val, min_val;

//sample action variables
string global_frame;
string rad_topic;
actionlib::SimpleActionServer<radbot_processor::sampleAction> * sampleAs;
radbot_processor::sampleFeedback sampleFb;
ros::Subscriber sample_sub;
unsigned int sample_count;
unsigned int sample_goal;
double sample_sum;

//pso action variables

void sampleGoalCB();
void samplePreemptCB();
void sampleCB(const ursa_driver::ursa_countsConstPtr msg);

ifstream infile;
costfn * my_cost;

void
openFile();

int main(int argc, char **argv) {
    ros::init(argc, argv, "radbot_processor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh(nh, "~");

    pnh.param<std::string>("global_frame", global_frame, "map");
    pnh.param<std::string>("topic", rad_topic, "counts");

    my_cost = new costfn();
    sampleAs =
            new actionlib::SimpleActionServer<radbot_processor::sampleAction>(
                    nh, "process_sampler", false);
    sampleAs->registerGoalCallback(&sampleGoalCB);
    sampleAs->registerPreemptCallback(&samplePreemptCB);
    sample_sub = nh.subscribe(rad_topic, 1, &sampleCB);



#ifdef DEBUG
    openFile();
    my_cost = new costfn(
            vector<sample>(measurements.begin(), measurements.begin() + 11));
    pso my_pso(*my_cost, min_val, max_val, 100, 5000, 2);
    std::vector<double> result;
    std::ostream_iterator<double> out_it(std::cout, ", ");
    //clock_t t0 = clock(), t1;
    for (vector<sample>::iterator i = measurements.begin() + 11;
            i != measurements.end(); i++) {
        my_cost->addSample(*i);
        my_pso.setCostFn(*my_cost);
        result = my_pso.run();
        std::copy(result.begin(), result.end(), out_it);
        cout << endl;
    }
    //t1 = clock() - t0;
    //cerr << (float) t1 / CLOCKS_PER_SEC << endl;
#endif

    spinner.spin();

}

inline void sampleCB(const ursa_driver::ursa_countsConstPtr msg) {
    if (!sampleAs->isActive())
        return;
    sample_sum += msg->counts;
    sample_count++;
    sampleFb.sample = sample_count;
    sampleAs->publishFeedback(sampleFb);
    if (sample_count >= sample_goal) {
        tf::TransformListener tf_listener;
        tf::StampedTransform transform;
        if (!tf_listener.waitForTransform(global_frame, msg->header.frame_id,
                ros::Time::now(), ros::Duration(10))) {
            ROS_ERROR_STREAM(
                    "Couldn't transform from "<<global_frame<<" to "<< "base_link");
            sampleAs->setAborted();
            return;
        }
        while (1) {
            try {
                tf_listener.lookupTransform(global_frame, msg->header.frame_id,
                        ros::Time(0), transform);
                break;
            } catch (tf::TransformException * ex) {
                ROS_ERROR("%s", ex->what());
                ros::Duration(1.0).sleep();
            }
        }
        sample temp;
        temp.x = transform.getOrigin().x();
        temp.y = transform.getOrigin().y();
        temp.counts = sample_sum / (float) sample_count;
        my_cost->addSample(temp);
        sampleAs->setSucceeded();
    }

}
inline void sampleGoalCB() {
    sample_count = 0;
    sample_sum = 0;
    sample_goal = sampleAs->acceptNewGoal()->samples;
}
inline void samplePreemptCB() {
    ROS_INFO("Sampling: Preempted");
    // set the action state to preempted
    sampleAs->setPreempted();
}

inline void openFile() {
    try {
        infile.open("datawval.csv", ifstream::in);
    } catch (ifstream::failure * e) {
        ROS_ERROR("Exception opening/reading/closing file");
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
                ROS_DEBUG_STREAM(
                        "x val: " << reading.x << " y val: " << reading.y << " counts: " << reading.counts);
            } catch (ifstream::failure * e) {
                ROS_ERROR_STREAM(
                        "Exception opening/reading/closing file\n" << e->what());
            }
            measurements.push_back(reading);
        }
        infile.close();
    } else {
        ROS_ERROR("Error opening file");
    }
    ROS_INFO_STREAM("Size of point list:" << measurements.size());

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
    ROS_INFO_STREAM(
            "Max Vals:" << max_val << endl << "Min Vals:" << min_val << endl);
}
