// Author: Keenan Burnett
#include "secondary_clustering/secondary_clustering_node.h"
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include <pcl/filters/statistical_outlier_removal.h>
#include "utils/zeus_pcl.hpp"
#include "utils/line_of_sight_filter.hpp"
#include "utils/planar_filter.hpp"
#include "utils/association.hpp"

namespace object_detection {

zeus_msgs::Detections3D SecondaryClusteringNode::cluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scan, const Eigen::Matrix4d& T_oc) {
	std::lock_guard<std::mutex> lock(pcd_mtx);

	zeus_msgs::Detections3D outputDetections;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_scan_los(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_scan(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl_pcds.clear();

	bool vis_normal = false;
	kalman::LOSFilter los_filter(80);
	auto normal_msg = los_filter.filter(scan, filtered_scan_los, vis_normal);
	if (vis_normal) normal_line_pub_.publish(normal_msg);

	pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
	sor.setInputCloud(filtered_scan_los);
	sor.setMeanK(50);
	sor.setStddevMulThresh(0.3);
	sor.filter(*filtered_scan);

	kalman::PlanarFilter planar_filter;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> planes;
	planar_filter.filter(filtered_scan, planes);
	std::cout << "[JQ9] Number of planes found: " << planes.size() << std::endl;
	for (size_t pi(0); pi<planes.size(); pi++) {
		std::cout << "[JQ9]   Number of pts: " << planes[pi]->size() << std::endl;
	}


	sensor_msgs::PointCloud2 gp_prior_msg;

    zeus_pcl::PointCloudPtr pc(new zeus_pcl::PointCloud());

    pc = zeus_pcl::filterAndCombinePlanes(planes, color_);

    zeus_pcl::PointCloudPtr gp_prior(new zeus_pcl::PointCloud());
    //zeus_pcl::fromPCLRGB(*filtered_scan, pc);

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

    zeus_pcl::PointCloudPtr pcd0(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr pcd1(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr pcd2(new zeus_pcl::PointCloud());
    std::vector<zeus_pcl::PointCloudPtr> pcds{pcd0,pcd1,pcd2};
    zeus_pcl::divideByDynamicness(gp_prior, pcds, color_);


    pcl_conversions::fromPCL(scan->header, outputDetections.header);
    outputDetections.header.frame_id = "map";
    outputDetections.camera = 1;
    outputDetections.bbs.clear();
    // Secondary Clustering
    for (int type(0); type<pcds.size(); type++) {
    	secondary_clustering(pcds[type], outputDetections, type);
    }
    NMSBoxes(outputDetections);
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
    zeus_msgs::Detections3D &outputDetections, int type) {
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
        detection.type = type;
        detection.confidence = 1.0;
        detection.camera = 1;

        if (detection.l > 0.2 && detection.w > 0.0 && detection.h > 0.3) {
        	outputDetections.bbs.push_back(detection);
        	pcl_pcds.push_back(cluster_pts);
        }
    }
}

void SecondaryClusteringNode::MergeBoxes(zeus_msgs::Detections3D &dets, size_t target, size_t source) {
	zeus_pcl::PointCloudPtr target_pts = pcl_pcds.at(target);
	zeus_pcl::PointCloudPtr source_pts = pcl_pcds.at(source);
	zeus_pcl::PointCloudPtr merged_pts(new zeus_pcl::PointCloud());

	zeus_pcl::applyDynamicness(source_pts, target_pts->points.front().dynamicness);
	zeus_pcl::append(merged_pts, target_pts);
	zeus_pcl::append(merged_pts, source_pts);

	sensor_msgs::PointCloud2 merged_pcd_msg;
	zeus_pcl::toROSMsg(merged_pts, merged_pcd_msg, "map");

	//zeus_pcl::randomDownSample(merged_pts, max(target_pts->size()/merged_pts->size(), source_pts->size()/merged_pts->size()));
	auto bbox = zeus_pcl::getBBox(merged_pts);

	dets.bbs[target].x = bbox.at(0);
	dets.bbs[target].y = bbox.at(1);
	dets.bbs[target].z = bbox.at(2);
	dets.bbs[target].l = bbox.at(3);
	dets.bbs[target].w = bbox.at(4);
	dets.bbs[target].h = bbox.at(5);
	dets.bbs[target].yaw = bbox.at(6);
	dets.bbs[target].cloud = merged_pcd_msg;
}

void SecondaryClusteringNode::NMSBoxes(zeus_msgs::Detections3D &dets) {
	std::cout << "[JQ10] Dets before NMS " << dets.bbs.size() << std::endl;
	std::vector<int> merged(dets.bbs.size(), 0);

	for (size_t i(0); i<dets.bbs.size(); i++) {
		for (size_t j(0); j<dets.bbs.size(); j++) {
			if (i == j) continue;
			if (merged[i]+merged[j] > 0) continue;
			if (pcl_pcds[i]->points.empty() || pcl_pcds[j]->points.empty()) continue;

			zeus_msgs::BoundingBox3D& box_i = dets.bbs[i];
			double area_i = box_i.l * box_i.w;
			zeus_msgs::BoundingBox3D& box_j = dets.bbs[j];
			double area_j = box_j.l * box_j.w;

			double inter_a_ij = kalman::calculate_intersection_rotated(box_i.x, box_i.y, box_i.l, box_i.w, box_i.yaw,
					box_j.x, box_j.y, box_j.l, box_j.w, box_j.yaw);
			double inter_p_ij = max(inter_a_ij/area_i, inter_a_ij/area_j);
			if (inter_p_ij > 0.5) {
				if (area_i > area_j) {
					MergeBoxes(dets, i, j);
					merged[j] = 1;
				} else {
					MergeBoxes(dets, j, i);
					merged[i] = 1;
				}
			}
		}
	}

	std::vector<zeus_msgs::BoundingBox3D> filtered_bbs;
	for (size_t i(0); i<dets.bbs.size(); i++) {
		if (merged[i] == 0) filtered_bbs.push_back(dets.bbs[i]);
	}
	dets.bbs = filtered_bbs;
	std::cout << "[JQ10] Dets after NMS " << dets.bbs.size() << std::endl;
}

}
