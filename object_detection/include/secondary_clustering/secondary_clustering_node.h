// Author: Keenan Burnett
// Copyright (C) 2020 aUToronto
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef SECONDARY_CLUSTERING_SECONDARY_CLUSTERING_NODE_H
#define SECONDARY_CLUSTERING_SECONDARY_CLUSTERING_NODE_H

#include <ros/ros.h>
#include <zeus_msgs/BoundingBox3D.h>
#include <zeus_msgs/Detections3D.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <chrono>  // NOLINT [build/c++11]
#include <string>
#include <vector>
#include <mutex>
#include "types/colors.hpp"
#include "types/zeus_pcl_types.hpp"
#include "utils/transform_utils.hpp"
#include <visualization_msgs/Marker.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace object_detection {
//* SecondaryClusteringNode
/**
* \brief This ROS node receives 3D LIDAR pointclouds and outputs 3D object centroids and cuboids using
* Euclidean clustering.
*
*/
class SecondaryClusteringNode {
 public:
	SecondaryClusteringNode(ros::NodeHandle nh);

    void set_node_name() {node_name = ros::this_node::getName();}

    /*!
       \brief Initializes the static transforms so that they only need to be queried once.
       \pre Make sure to run get_ros_parameters() before this.
    */
    void initialize_transforms();

    /*!
       \brief Retrieve rosparams that apply to this node.
       \pre Make sure to run set_node_name() before this function.
    */
    void get_ros_parameters();

    /*!
       \brief This callback first extracts the ground plane using RANSAC and then performs Euclidean clustering
       using a KdTree to extract object clusters. For each cluster of points, a centroid and 3D cuboid is computed
       which is published as a list in a ROS message.
       \param scan A 3D pointcloud output by a velodyne LIDAR.
    */
    zeus_msgs::Detections3D cluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scan, const Eigen::Matrix4d& T_oc);

    void callback(const sensor_msgs::PointCloud2ConstPtr& scan) {
    	//auto ret = cluster(scan);
    }

 private:
    ros::NodeHandle nh_;
    ros::Publisher det_pub_;
    ros::Publisher gp_pub_;
    ros::Publisher gp_prior_pub_;
    ros::Publisher normal_line_pub_;

    std::string node_name = "secondary_clustering";

    Eigen::Matrix4d Tc2_v = Eigen::Matrix4d::Identity();

    std::vector<double> point_cloud_range = {0, 50.0, -15.0, 15.0, -2.0, 0.0};
    // gp param {1.0, 40.0, -10, 10.0, -2.75, -1.75}
    std::vector<double> ground_point_range = {1.0, 40.0, -10, 10.0, -2.75, -1.75};
    std::vector<double> max_object_size = {4.0, 4.0, 4.0};
    std::vector<double> min_object_size = {0.0, 0.0, 0.0};

    float randomDownSampleThreshold = 0.5;  // stock 0.5
    bool doRandomDownsample = false;
    float min_object_z = -2.0, max_object_z = 0.5;  // filter out object that gp is not perfect
    float max_distance_ransac = 0.25;       /*!< Points closer than this are an inlier to the RANSAC ground plane */
    int max_iterations_ransac = 25;         /*!< Maximum number of iterations to run RANSAC */
    int min_samples = 30;                   /*!< Minimum number of points required to keep a cluster */
    float cluster_tolerance = 0.25;         /*!< Points closer than this distance will be clustered together. */
    int unknown_type = 3;
    Eigen::Matrix4f K = Eigen::Matrix4f::Identity();
    Eigen::Vector4f gp_params = Eigen::Vector4f::Zero();

    std::shared_ptr<Color> color_ = nullptr;
    std::vector<zeus_pcl::PointCloudPtr> pcl_pcds;
    std::mutex pcd_mtx;

    void secondary_clustering(zeus_pcl::PointCloudPtr pc, zeus_msgs::Detections3D &outputDetections, int type);

    void MergeBoxes(zeus_msgs::Detections3D &dets, size_t target, size_t source);
    void NMSBoxes(zeus_msgs::Detections3D &dets);
};

}

#endif  // SECONDARY_CLUSTERING_SECONDARY_CLUSTERING_NODE_H
