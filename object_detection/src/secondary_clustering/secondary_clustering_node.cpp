// Author: Keenan Burnett
#include "secondary_clustering/secondary_clustering_node.h"
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include "utils/zeus_pcl.hpp"
#include <pcl/filters/voxel_grid.h>

zeus_msgs::Detections3D SecondaryClusteringNode::cluster(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& scan, const Eigen::Matrix4d& T_oc) {
	zeus_msgs::Detections3D outputDetections;
	sensor_msgs::PointCloud2 gp_prior_msg;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr scan_filtered (new pcl::PointCloud<pcl::PointXYZRGB>());

	pcl::VoxelGrid<pcl::PointXYZRGB> filter;
	filter.setLeafSize(0.05, 0.05, 0.05);
	filter.setInputCloud(scan);
	filter.filter(*scan_filtered);

    zeus_pcl::PointCloudPtr pc(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr gp_prior(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr gp(new zeus_pcl::PointCloud());
    zeus_pcl::fromPCLRGB(*scan_filtered, pc);

    Eigen::Matrix4d C_oc = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d r_oc = Eigen::Matrix4d::Identity();
    C_oc.block<3,3>(0,0) = T_oc.block<3,3>(0,0);
    zeus_pcl::transform_cloud(pc, C_oc);
    zeus_pcl::passthrough(pc, gp_prior, -2.5, 2.5, -2.5, 2.5, 0.1, 3.5);

    r_oc.block<3,1>(0,3) = T_oc.block<3,1>(0,3);
    zeus_pcl::transform_cloud(gp_prior, r_oc);

    pcl_conversions::fromPCL(scan->header, outputDetections.header);
    outputDetections.header.frame_id = "map";
    outputDetections.camera = 1;
    outputDetections.bbs.clear();
    // Secondary Clustering
    secondary_clustering(gp_prior, outputDetections);
    // Publish ROS Message
    det_pub_.publish(outputDetections);

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
        zeus_pcl::randomDownSample(cluster_pts, 0.5);
        if (cluster_pts->size() == 0) continue;

        zeus_pcl::PointCloudPtr cluster_pts_2d (new zeus_pcl::PointCloud());
        zeus_pcl::to_2d(cluster_pts, cluster_pts_2d);



        cv::Mat data_pts = cv::Mat(cluster_pts_2d->size(), 2, CV_64F);
        for (int j = 0; j < data_pts.rows; j++) {
            data_pts.at<double>(j, 0) = cluster_pts_2d->points[j].x;
            data_pts.at<double>(j, 1) = cluster_pts_2d->points[j].y;
        }
        cv::PCA pca_analysis(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW, 2);

        std::vector<Eigen::Vector2d> eigen_vecs(2);
        std::vector<double> eigen_vals(2);
        for (int k = 0; k < 2; k++) {
            eigen_vecs[k][0] = pca_analysis.eigenvectors.at<double>(k, 0);
            eigen_vecs[k][1] = pca_analysis.eigenvectors.at<double>(k, 1);
            eigen_vals[k] = pca_analysis.eigenvalues.at<double>(k);
        }

        Eigen::Vector4d centroid; // = zeus_pcl::get_centroid(cluster_pts, cuboid);
        centroid(0,0) = static_cast<float>(pca_analysis.mean.at<double>(0, 0));
        centroid(1,0) = static_cast<float>(pca_analysis.mean.at<double>(0, 1));
        centroid(3,0) = 1.0;

        size_t major_axis = eigen_vals[0] > eigen_vals[1] ? 0 : 1;
        size_t minor_axis = 1 - major_axis;
        double yaw = atan2(eigen_vecs[major_axis][0], eigen_vecs[major_axis][1]);
       // if (yaw < 0) {
       // 	yaw += M_PI;
       // } else if (yaw > M_PI) {
       // 	yaw -= M_PI;
       // }

        double max_x = 0, max_y = 0, max_z = 0;
        for (size_t p = 0; p < cluster_pts->size(); p++) {
        	Eigen::Vector2d centered_pt(cluster_pts->points[p].x - centroid(0,0), cluster_pts->points[p].y - centroid(1,0));

        	double proj_major_axis = abs(centered_pt[0] * eigen_vecs[major_axis][0] + centered_pt[1] * eigen_vecs[major_axis][1]);
        	double proj_minor_axis = abs(centered_pt[0] * eigen_vecs[minor_axis][0] + centered_pt[1] * eigen_vecs[minor_axis][1]);
        	Eigen::Vector2d pt_pca(proj_major_axis, proj_minor_axis);


        	if (proj_major_axis > max_x) {
        		max_x = proj_major_axis;
        	}
        	if (proj_minor_axis > max_y) {
        		max_y = proj_minor_axis;
        	}
        	if (abs(cluster_pts->points[p].z/2) > max_z) max_z = abs(cluster_pts->points[p].z/2);
        }
        centroid(2, 0) = max_z;
        double w, l, h;
        // cuboid: {min_x, min_y, min_z, max_x, max_y, max_z}
        l = max_x*2*1.1;
        w = max_y*2*1.1;
        h = max_z*2*1.1;
        //if (l > max_object_size[0] || w > max_object_size[1] || h > max_object_size[2] ||
       //    l < min_object_size[0] || w < min_object_size[1] || h < min_object_size[2])
       //     continue;
        //if (centroid(2, 0) < min_object_z || centroid(2, 0) > max_object_z)
        //    continue;
        zeus_msgs::BoundingBox3D detection;
        // centroid = Tc2_v * centroid;
        detection.x = centroid(0, 0);
        detection.y = centroid(1, 0);
        detection.z = centroid(2, 0);
        detection.l = l;
        detection.w = w;
        detection.h = h;
        detection.yaw = yaw;
        detection.type = unknown_type;
        detection.confidence = 0.9;
        detection.camera = 1;
        outputDetections.bbs.push_back(detection);
    }
}
