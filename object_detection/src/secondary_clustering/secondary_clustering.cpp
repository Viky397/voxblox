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
    nh.getParam(node_name + "/output_append",   output_append);
    // Publishers
    ros::Publisher det_pub = nh.advertise<zeus_msgs::Detections3D>("/Object/RawDetections3D" + output_append, 1);
    ros::Publisher gp_pub = nh.advertise<sensor_msgs::PointCloud2>("/Object/GroundPlane3D" + output_append, 1);
    ros::Publisher gp_prior_pub = nh.advertise<sensor_msgs::PointCloud2>(
            "/Object/GroundPlanePrior3D" + output_append, 1);
    // Initialize cluster publisher node object
    SecondaryClusteringNode myNode(det_pub, gp_pub, gp_prior_pub, nh);
    myNode.set_node_name();
    myNode.get_ros_parameters();
    //myNode.initialize_transforms();
    // Subscribers
    ros::Subscriber sub = nh.subscribe(lidar_topic, 1, &SecondaryClusteringNode::callback, &myNode);
    ROS_INFO("[OBJ] secondary_clustering running!");
    ros::spin();
    return 0;
}
