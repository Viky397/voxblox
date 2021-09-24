#pragma once

#include <iostream>
#include <fstream>
#include <math.h>
#include <cmath>

#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/PCLPointCloud2.h>

#include "utils/normal_estimation.hpp"

namespace kalman {

class LOSFilter{
public:
	LOSFilter(float max_angle_deg) {
		max_angle_deg_ = fabs(max_angle_deg);
		max_angle_rad_ = max_angle_deg_ / 180.0 * M_PI;
	}

	void filter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_out) {
		pcd_out->header = pcd_in->header;
		pcl::PointCloud<pcl::PointXYZ>::Ptr pcd(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::copyPointCloud(*pcd_in, *pcd);

		pcl::PointCloud<pcl::PointNormal>::Ptr pcd_normal(new pcl::PointCloud<pcl::PointNormal>);
		normal_estmation(pcd, pcd_normal);

		if (pcd_in->points.size() != pcd_normal->points.size()) {
			throw std::runtime_error("Size inconsistent after normal calculation!");
		}

		for (size_t idx(0); idx<pcd_in->points.size(); idx++) {
			const auto& pt_normal = pcd_normal->points.at(idx);
			float cos_theta = 0;
			float theta = 90;

			Eigen::Vector3f los(pt_normal.x, pt_normal.y, pt_normal.z);
			los /= los.norm();
			Eigen::Vector3f normal(pt_normal.normal_x, pt_normal.normal_y, pt_normal.normal_z);
			normal /= normal.norm();

			cos_theta = los[0] * normal[0] + los[1] * normal[1] + los[2] * normal[2];
			theta = fabs(wrapAngle(acos(cos_theta)));

			//std::cout << "Point " << idx << " los: " << los[0] << "  " << los[1] << std::endl;
			//std::cout << "      " << " normal: " << normal[0] << "  " << normal[1] << std::endl;
			//std::cout << "      " << " cos_theta: " << cos_theta << std::endl;
			//std::cout << "      " << " theta: " << theta << std::endl;

			if (theta <= max_angle_rad_ || theta >= M_PI-max_angle_rad_) {
				pcd_out->push_back(pcd_in->points.at(idx));
				//std::cout << "      " << " Push" << std::endl;
			}
		}
	}

private:	
	void normal_estmation(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd, pcl::PointCloud<pcl::PointNormal>::Ptr output_PointNormal) {
		NormEstimator normal_estimator;
		normal_estimator.estimate(pcd, output_PointNormal, 10, false);
	}

	double wrapAngle(float angle) {
	    if (angle <= (float)-M_PI) return wrapAngle(angle + 2.0 * (float)M_PI);
	    if (angle > (float)M_PI) return wrapAngle(angle - 2.0 * (float)M_PI);
	    return angle;
	}

	float max_angle_deg_;
	float max_angle_rad_;

};
}
