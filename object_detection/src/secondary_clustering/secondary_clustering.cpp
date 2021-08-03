// Author: Keenan Burnett
#include <secondary_clustering/secondary_clustering_node.h>
#include <ros/console.h>
#include <string>

int main(int argc, char **argv) {
    ros::init(argc, argv, "secondary_clustering");
    ros::NodeHandle nh;
    // Get parameters
    std::string output_append, lidar_topic;
    std::string node_name = ros::this_node::getName();
    nh.getParam(node_name + "/lidar_topic",     lidar_topic);
    // Initialize cluster publisher node object
    SecondaryClusteringNode myNode(nh);
    // Subscribers
    ros::Subscriber sub = nh.subscribe(lidar_topic, 1, &SecondaryClusteringNode::callback, &myNode);
    ROS_INFO("[OBJ] secondary_clustering running!");
    ros::spin();
    return 0;
}
