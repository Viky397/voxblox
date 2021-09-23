#pragma once

#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PCLPointCloud2.h>

namespace kalman {

class NormEstimator{
public:
	NormEstimator() {}

	void estimate(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd,
				  pcl::PointCloud<pcl::PointNormal>::Ptr output_PointNormal,
				  int k_neighbors,
				  bool skip_inf) {
		pcl::PointCloud<pcl::Normal> normals;
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
		normal_estimation.setInputCloud(pcd);
		normal_estimation.setSearchSurface(pcd);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
		normal_estimation.setSearchMethod(search_tree);
		normal_estimation.setKSearch(k_neighbors);
		normal_estimation.compute(normals);
		for (int i = 0; i < normals.size(); i++) {
			pcl::PointXYZ pt = pcd->at(i);
			pcl::Normal nl = normals[i];
			if (pcl_isinf(nl.normal_x) || pcl_isinf(nl.normal_y) || pcl_isinf(nl.normal_z)) {
				if (skip_inf) continue;
			}
			pcl::PointNormal pt_nl;
			pt_nl.x = pt.x;
			pt_nl.y = pt.y;
			pt_nl.z = pt.z;
			pt_nl.normal_x = nl.normal_x;
			pt_nl.normal_y = nl.normal_y;
			pt_nl.normal_z = nl.normal_z;
			output_PointNormal->push_back(pt_nl);
		}
	}
};
}
