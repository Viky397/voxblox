#pragma once

#include <iostream>
#include <math.h>
#include <cmath>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>

#include <visualization_msgs/Marker.h>

namespace kalman {

class PlanarFilter {
public:
	PlanarFilter() {

	}

	void filter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_in,
			std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &plane_pcds_out) {

		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZRGB>);
		copyPointCloud(*pcd_in, *cloud_filtered);

		pcl::SACSegmentation<pcl::PointXYZRGB> seg;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_PLANE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setMaxIterations(20);
		seg.setDistanceThreshold(0.25);

		int nr_points = (int) cloud_filtered->size();
		while (cloud_filtered->size() > 0.05 * nr_points) {
			// Segment the largest planar component from the remaining cloud
			seg.setInputCloud(cloud_filtered);
			seg.segment(*inliers, *coefficients);
			if (inliers->indices.size() == 0) {
				break;
			}

			// Extract the planar inliers from the input cloud
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(cloud_filtered);
			extract.setIndices(inliers);
			extract.setNegative(false);

			// Get the points associated with the planar surface
			extract.filter(*cloud_plane);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZRGB>);
			copyPointCloud(*cloud_plane, *cloud_p);
			plane_pcds_out.push_back(cloud_p);

			// Remove the planar inliers, extract the rest
			extract.setNegative(true);
			extract.filter(*cloud_temp);
			*cloud_filtered = *cloud_temp;
		}
	}

private:

};
}
