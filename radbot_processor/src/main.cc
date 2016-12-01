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
#include <std_srvs/Empty.h>
#include <actionlib/server/simple_action_server.h>
#include "radbot_processor/sampleAction.h"
#include "radbot_processor/psoAction.h"
#include "radbot_processor/util.h"
#include "radbot_processor/pso.h"

ros::MultiThreadedSpinner spinner(4);

vector<sample> measurements;
sample max_val, min_val;

//sample action variables
string global_frame;
string rad_topic;
actionlib::SimpleActionServer<radbot_processor::sampleAction> * sampleAs;
radbot_processor::sampleFeedback sampleFb;
radbot_processor::sampleResult sampleRs;
ros::Subscriber sample_sub;
unsigned int sample_count;
unsigned int sample_goal;
double sample_sum;

void sampleGoalCB();
void samplePreemptCB();
void sampleCB(const ursa_driver::ursa_countsConstPtr msg);

//pso action variables
actionlib::SimpleActionServer<radbot_processor::psoAction> * psoAs;
void psoExecuteCB(const radbot_processor::psoGoalConstPtr &goal);

//clearSamples variables
bool clearSamplesCB(std_srvs::Empty::Request& request,
                    std_srvs::Empty::Response& response);

ifstream infile;
costfn * my_cost;
pso * my_pso;

tf::TransformListener * tf_listener;

void
openFile();

int main(int argc, char **argv) {
    ros::init(argc, argv, "radbot_processor");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    tf_listener = new tf::TransformListener(nh);

    pnh.param<std::string>("global_frame", global_frame, "map");
    pnh.param<std::string>("topic", rad_topic, "counts");

    my_cost = new costfn();
    sampleAs =
            new actionlib::SimpleActionServer<radbot_processor::sampleAction>(
                    nh, "process_sampler", false);
    sampleAs->registerGoalCallback(&sampleGoalCB);
    sampleAs->registerPreemptCallback(&samplePreemptCB);
    sample_sub = nh.subscribe(rad_topic, 1, &sampleCB);

    psoAs = new actionlib::SimpleActionServer<radbot_processor::psoAction>(
            nh, "process_pso", &psoExecuteCB, false);

    ros::ServiceServer clrSamplesSrv = nh.advertiseService("clear_samples",
                                                           clearSamplesCB);

    my_pso = new pso(*my_cost, min_val, max_val, 250, 3000, 2);

#ifdef DEBUG
    openFile();
    my_cost = new costfn(
            vector<sample>(measurements.begin(), measurements.end()));
    pso my_pso2(*my_cost, min_val, max_val, 250, 3000, 2);
    std::vector<double> result;
    std::ostream_iterator<double> out_it(std::cout, ", ");
    clock_t t0 = clock(), t1, t2;
    /*for (vector<sample>::iterator i = measurements.begin() + 11;
            i != measurements.end(); i++)
    {
        my_cost->addSample(*i);
        my_pso.setCostFn(*my_cost);
        result = my_pso.run();
        std::copy(result.begin(), result.end(), out_it);
        cout << endl;
    }*/
    for(int i = 0; i<10; i++){
        t2 = clock();
        result = my_pso2.run();
        std::copy(result.begin(), result.end(), out_it);
        cout << endl;
        t1 = clock() - t2;
        cout << (float) t1 / CLOCKS_PER_SEC << endl;
    }

    t1 = clock() - t0;
    cerr << (float) t1 / CLOCKS_PER_SEC << endl;
#endif

    sampleAs->start();
    psoAs->start();
    ROS_INFO("PSO Ready");
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
        tf::StampedTransform transform;
        int tries = 0;
        while (!tf_listener->waitForTransform(global_frame,
                                              msg->header.frame_id,
                                              msg->header.stamp,
                                              ros::Duration(10.0))) {
            ROS_ERROR_STREAM(
                    "Couldn't transform from \"" << global_frame << "\" to \""
                            << msg->header.frame_id << "\"");
            if (tries > 4) {
                sampleAs->setAborted();
                return;
            }
            tries++;
        }

        while (1) {
            try {
                tf_listener->lookupTransform(global_frame, msg->header.frame_id,
                                             msg->header.stamp, transform);
                break;
            }
            catch (tf::TransformException * ex) {
                ROS_ERROR("%s", ex->what());
                ros::Duration(1.0).sleep();
            }
        }
        sample temp;
        temp.x = sampleRs.x = transform.getOrigin().x();
        temp.y = sampleRs.y = transform.getOrigin().y();
        temp.counts = sample_sum / (float) sample_count;
        ROS_INFO_STREAM("PSO: Newest Sample: " << temp);
        my_cost->addSample(temp);
        sampleAs->setSucceeded(sampleRs);
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

void psoExecuteCB(const radbot_processor::psoGoalConstPtr &goal) {
    vector<sample> temp(my_cost->getObs());
    minimax(temp, &max_val, &min_val);
    my_pso->setCostFn(*my_cost);
    my_pso->setParticles(goal->particles);
    my_pso->setSources(goal->numSrc);
    my_pso->setBounds(max_val, min_val);

    radbot_processor::psoResult res;
    ROS_WARN("PSO: About to Run");
    res.params = my_pso->run();
    res.cost = my_pso->getGMin();
    psoAs->setSucceeded(res);
}

bool clearSamplesCB(std_srvs::Empty::Request& request,
                    std_srvs::Empty::Response& response) {
    my_cost->clearAll();
    ROS_INFO("PSO Samples Reset");
}

inline void openFile() {
    try {
        infile.open("data_4_extra.csv", ifstream::in);
    }
    catch (ifstream::failure * e) {
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
                        "x val: " << reading.x << " y val: " << reading.y
                                << " counts: " << reading.counts);
            }
            catch (ifstream::failure * e) {
                ROS_ERROR_STREAM(
                        "Exception opening/reading/closing file\n"
                                << e->what());
            }
            measurements.push_back(reading);
        }
        infile.close();
    }
    else {
        ROS_ERROR("Error opening file");
    }
    ROS_INFO_STREAM("Size of point list:" << measurements.size());

    minimax(measurements, &max_val, &min_val);

}
