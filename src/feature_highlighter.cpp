#include <iostream>
#include <mutex>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Vector3.h"
#include "cv_bridge/cv_bridge.h"

using namespace std;

ros::NodeHandle *node;
ros::Subscriber *sub_image_source;
ros::Subscriber *sub_feature_source;
ros::Publisher *pub_image_output;

string nodename;
vector<geometry_msgs::Vector3::ConstPtr> features;
mutex features_mutex;

void receive_image(const sensor_msgs::Image::ConstPtr &msg) {
    cv_bridge::CvImagePtr work;

    work = cv_bridge::toCvCopy(msg);

    features_mutex.lock();
    cout << "Highlighting " << features.size() << " features." << endl;
    for(uint32_t i = 0; i < features.size(); i++) {
        cv::Point centre(features[i]->x + msg->width / 2, features[i]->y + msg->height / 2);
        double radius = 1 / features[i]->z;
        cv::Size size(radius, radius);
        cv::Scalar color(0, 255, 0);

        cv::ellipse(work->image, centre, size, 0, 0, 360, color, 4);
    };
    features.clear();
    features_mutex.unlock();

    work->header.seq = msg->header.seq;
    work->header.stamp = msg->header.stamp;
    pub_image_output->publish(work->toImageMsg());
};

void receive_feature(const geometry_msgs::Vector3::ConstPtr &msg) {
    features_mutex.lock();
    cout << "Received feature! New size is " << features.size() << endl;
    if(features.size() < 256)
        features.push_back(msg);
    features_mutex.unlock();
};
    
int main(int argc, char **argv) {
    ros::init(argc, argv, "feature_highlighter", ros::init_options::AnonymousName);

    nodename = ros::this_node::getName();
    node = new ros::NodeHandle();
    if(! node->hasParam(nodename + "/image_source"))
        node->setParam(nodename + "/image_source", "image");
    if(! node->hasParam(nodename + "/feature_source"))
        node->setParam(nodename + "/feature_source", "features");
    node->setParam(nodename + "/image_source__meta/type", "string");
    node->setParam(nodename + "/image_source__meta/defines", "topic");
    node->setParam(nodename + "/image_source__meta/topic_type", "sensor_msgs/Image");
    node->setParam(nodename + "/feature_source__meta/type", "string");
    node->setParam(nodename + "/feature_source__meta/defines", "topic");
    node->setParam(nodename + "/feature_source__meta/topic_type", "geometry_msgs/Vector3");

    pub_image_output = new ros::Publisher(node->advertise<sensor_msgs::Image>(nodename + "/highlighted", 1));
    string image_source;
    string feature_source;
    ros::Rate r(100);
    while(ros::ok()) {
        string new_image_source;
        if(node->getParam(nodename + "/image_source", new_image_source)) {
            if(new_image_source != image_source) {
                image_source = new_image_source;
                if(sub_image_source) {
                    delete sub_image_source;
                    sub_image_source = NULL;
                };
                sub_image_source = new ros::Subscriber(node->subscribe(image_source, 1, receive_image));
            };
        };

        string new_feature_source;
        if(node->getParam(nodename + "/feature_source", new_feature_source)) {
            if(new_feature_source != feature_source) {
                feature_source = new_feature_source;
                if(sub_feature_source) {
                    delete sub_feature_source;
                    sub_feature_source = NULL;
                };
                sub_feature_source = new ros::Subscriber(node->subscribe(feature_source, 128, receive_feature));
            };
        };

        ros::spinOnce();
        r.sleep();
    };

    node->deleteParam(nodename);

    return 0;
};

