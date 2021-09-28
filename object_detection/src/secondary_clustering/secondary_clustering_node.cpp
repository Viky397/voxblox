// Author: Keenan Burnett
#include "secondary_clustering/secondary_clustering_node.h"
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include "utils/zeus_pcl.hpp"
#include "utils/line_of_sight_filter.hpp"

zeus_msgs::Detections3D SecondaryClusteringNode::cluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scan, const Eigen::Matrix4d& T_oc) {
	zeus_msgs::Detections3D outputDetections;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_scan_los(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_scan(new pcl::PointCloud<pcl::PointXYZRGB>);

	bool vis_normal = false;
	kalman::LOSFilter los_filter(75);
	auto normal_msg = los_filter.filter(scan, filtered_scan_los, vis_normal);
	if (vis_normal) normal_line_pub_.publish(normal_msg);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(filtered_scan_los);
	sor.setMeanK(50);
	sor.setStddevMulThresh(0.3);
	sor.filter(*filtered_scan);

	sensor_msgs::PointCloud2 gp_prior_msg;

    zeus_pcl::PointCloudPtr pc(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr gp_prior(new zeus_pcl::PointCloud());
    zeus_pcl::fromPCLRGB(*filtered_scan, pc);

    Eigen::Matrix4d C_oc = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d r_oc_xy = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d r_oc_z = Eigen::Matrix4d::Identity();
    C_oc.block<3,3>(0,0) = T_oc.block<3,3>(0,0);
    r_oc_z.block<1,1>(2,3) = T_oc.block<1,1>(2,3);
    zeus_pcl::transform_cloud(pc, C_oc);
    zeus_pcl::transform_cloud(pc, r_oc_z);
    zeus_pcl::passthrough(pc, gp_prior, -100, 100, -100, 100, 0.1, 3.5);

    r_oc_xy.block<2,1>(0,3) = T_oc.block<2,1>(0,3);
    zeus_pcl::transform_cloud(gp_prior, r_oc_xy);

    pcl_conversions::fromPCL(scan->header, outputDetections.header);
    outputDetections.header.frame_id = "map";
    outputDetections.camera = 1;
    outputDetections.bbs.clear();
    // Secondary Clustering
    secondary_clustering(gp_prior, outputDetections);
    // Publish ROS Message
    // det_pub_.publish(outputDetections);

    zeus_pcl::toROSMsg(gp_prior, gp_prior_msg, "map");
    gp_prior_pub_.publish(gp_prior_msg);

    return outputDetections;
}

void SecondaryClusteringNode::get_ros_parameters() {
    nh_.getParam(node_name + "/secondary_cloud_range",   point_cloud_range);
    nh_.getParam(node_name + "/secondary_ground_point_range",   ground_point_range);
    nh_.getParam(node_name + "/MAX_DISTANCE_RANSAC",     max_distance_ransac);
    nh_.getParam(node_name + "/MAX_ITERATIONS_RANSAC",   max_iterations_ransac);
    nh_.getParam(node_name + "/CLUSTER_TOLERANCE_SECONDARY", cluster_tolerance);
    nh_.getParam(node_name + "/MIN_SAMPLES_SECONDARY",   min_samples);
    nh_.getParam(node_name + "/secondary_max_object_size",         max_object_size);
    nh_.getParam(node_name + "/secondary_min_object_size",         min_object_size);
    nh_.getParam(node_name + "/secondary_max_object_z",            max_object_z);
    nh_.getParam(node_name + "/secondary_min_object_z",            min_object_z);
    nh_.getParam(node_name + "/secondary_randomDownSampleThreshold",       randomDownSampleThreshold);
    nh_.getParam(node_name + "/secondary_doRandomDownsample",       doRandomDownsample);
    nh_.getParam(node_name + "/unknown_type",            unknown_type);
    float kalman_gain = 0.5;
    nh_.getParam(node_name + "/GROUND_KALMAN_GAIN",      kalman_gain);
    K = Eigen::Matrix4f::Identity() * kalman_gain;
    gp_params.matrix() << 0.0, 0.0, -1.0, -1.70;
}

void SecondaryClusteringNode::initialize_transforms() {
    //tfBuffer = zeus_tf::tfBufferPtr(new zeus_tf::tfBuffer());
    //tfListener = zeus_tf::tfListenerPtr(new zeus_tf::tfListener(*tfBuffer));
    //zeus_tf::get_transform(tfBuffer, "velodyne", "c2", Tc2_v);
}

void SecondaryClusteringNode::secondary_clustering(zeus_pcl::PointCloudPtr pc,
    zeus_msgs::Detections3D &outputDetections) {
    std::vector<std::vector<int>> clusters;
    if (pc->size() > 0) {
        zeus_pcl::cluster_point_cloud(pc, min_samples, cluster_tolerance, clusters);
        std::cout << "Number of clusters: " << clusters.size() << std::endl;
    }
    for (uint i = 0; i < clusters.size(); i++) {
        zeus_pcl::PointCloudPtr cluster_pts (new zeus_pcl::PointCloud());
        zeus_pcl::extract_points(pc, cluster_pts, clusters[i]);

        if (cluster_pts->size() == 0) continue;

        zeus_msgs::BoundingBox3D detection;
        sensor_msgs::PointCloud2 cluster_msg;

        zeus_pcl::toROSMsg(cluster_pts, cluster_msg, "map");
        detection.cloud = cluster_msg;

        zeus_pcl::randomDownSample(cluster_pts, 0.5);

        auto bbox = zeus_pcl::getBBox(cluster_pts);

        detection.x = bbox.at(0);
        detection.y = bbox.at(1);
        detection.z = bbox.at(2);
        detection.l = bbox.at(3);
        detection.w = bbox.at(4);
        detection.h = bbox.at(5);
        detection.yaw = bbox.at(6);
        detection.type = unknown_type;
        detection.confidence = 0.85;
        detection.camera = 1;

        outputDetections.bbs.push_back(detection);
    }
}
