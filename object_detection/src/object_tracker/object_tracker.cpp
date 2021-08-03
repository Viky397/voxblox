// Author: Keenan Burnett
#include "object_tracker/object_tracker_node.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <string>
#include <vector>
#include <map>

int main(int argc, char **argv) {
    ros::init(argc, argv, "kernel_publisher");
    ros::NodeHandle nh;
    // Get parameters
    std::string det_topic, pose_topic;
    std::string node_name = ros::this_node::getName();
    nh.getParam(node_name + "/pose_topic", pose_topic);
    nh.getParam(node_name + "/det_topic", det_topic);

    // Subscribers
    message_filters::Subscriber<zeus_msgs::Detections3D> sub_det(nh, det_topic, 10);
    message_filters::Subscriber<nav_msgs::Odometry> sub_odom(nh, pose_topic, 100);

    // Initialize kalman tracker node object
    KalmanTrackerNode myNode = KalmanTrackerNode(nh);

    typedef message_filters::sync_policies::ApproximateTime<zeus_msgs::Detections3D, nav_msgs::Odometry> sync_policy;
    message_filters::Synchronizer<sync_policy> sync(sync_policy(100), sub_det, sub_odom);

    sync.registerCallback(boost::bind(&KalmanTrackerNode::callback, myNode, _1, _2));

    ROS_INFO_STREAM("[OBJ] kalman tracker running!");
    ros::spin();
    return 0;
}

