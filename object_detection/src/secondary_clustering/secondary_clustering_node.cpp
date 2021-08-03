// Author: Keenan Burnett
#include "secondary_clustering/secondary_clustering_node.h"
#include <vector>
#include <chrono>
#include <opencv2/core.hpp>
#include "utils/zeus_pcl.hpp"

void SecondaryClusteringNode::callback(const sensor_msgs::PointCloud2ConstPtr& scan) {
	std::cout << "New scan" << std::endl;
	std::cout << "   scan size before conversion " << scan->width << std::endl;
    zeus_pcl::PointCloudPtr pc(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr gp_prior(new zeus_pcl::PointCloud());
    zeus_pcl::PointCloudPtr gp(new zeus_pcl::PointCloud());
    zeus_pcl::fromROSMsg(scan, pc);
    std::cout << "   scan size after conversion " << pc->size() << std::endl;
    //for (size_t i(0); i<100; i++) {
    //	std::cout << "    sample point " << pc->points[i] << std::endl;
    //}
    zeus_pcl::passthrough(pc, point_cloud_range[0], point_cloud_range[1], point_cloud_range[2],
        point_cloud_range[3], point_cloud_range[4], point_cloud_range[5]);
    std::cout << "   pcr[0] " << point_cloud_range[0] << " pcr[1] " << point_cloud_range[1]  << " pcr[2] " << point_cloud_range[2]  << " pcr[3] " << point_cloud_range[3]<< std::endl;
    std::cout << "   scan size after passthrough " << pc->size() << std::endl;
    // radialDownSample(pc);
    // zeus_pcl::copyCloud(pc, gp);
    zeus_pcl::passthrough(pc, gp_prior, ground_point_range[0], ground_point_range[1], ground_point_range[2],
        ground_point_range[3], ground_point_range[4], ground_point_range[5]);
    // zeus_pcl::removeGroundPlane(gp_prior, pc, gp_params, K, max_distance_ransac,
    //     max_iterations_ransac, true, gp);
    std::cout << "   scan size of gp prior " << gp_prior->size() << std::endl;
    zeus_pcl::copyCloud(gp_prior, gp);
    // alpha, tolerance, Tm, Tm_small, Tb, Trmse, Tdprev
    //auto ls = zeus_pcl::removeGroundPlane2(gp_prior, 3/180.0*M_PI, 0.25, 0.4, 0.2, 0.8, 2, 2, true, gp);
    //std::cout << "   scan size of gp " << gp->size() << std::endl;
    // Create ROS message
    zeus_msgs::Detections3D outputDetections;
    outputDetections.header.stamp = scan->header.stamp;
    outputDetections.header.frame_id = "map";
    outputDetections.camera = 1;
    outputDetections.bbs.clear();
    // Secondary Clustering
    secondary_clustering(gp_prior, outputDetections);
    // Publish ROS Message
    det_pub_.publish(outputDetections);
    sensor_msgs::PointCloud2 gp_msg;
    //zeus_pcl::toROSMsg(gp, gp_msg, "map");
    //gp_pub_.publish(gp_msg);
    sensor_msgs::PointCloud2 gp_prior_msg;
    zeus_pcl::toROSMsg(gp_prior, gp_prior_msg, "map");
    gp_prior_pub_.publish(gp_prior_msg);

/***
    visualization_msgs::Marker lines;
    lines.header.stamp = ros::Time::now();
    lines.header.frame_id = "velodyne_local";
    lines.ns = "gp_removal";
    lines.action = visualization_msgs::Marker::ADD;
    lines.type = visualization_msgs::Marker::LINE_LIST;
    lines.pose.orientation.w = 1;
    lines.id = 0;
    lines.scale.x = 0.05;
    lines.color.a = 1;
    lines.color.g = 1;
    for (auto& l : ls) {
        geometry_msgs::Point p;
        p.x = l.first[0];
        p.y = l.first[1];
        p.z = l.first[2];

        lines.points.push_back(p);

        p.x = l.second[0];
        p.y = l.second[1];
        p.z = l.second[2];

        lines.points.push_back(p);
    }
    gt_line_pub_.publish(lines);
    ***/
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
    if (pc->size() > 0)
        zeus_pcl::cluster_point_cloud(pc, min_samples, cluster_tolerance, clusters);
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
