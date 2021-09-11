#pragma once

#include <iostream>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>


namespace kalman {

class Alignment{
public:
	Alignment()
	{
		max_iter = 100;
		far_dist = 5;
		outlier_dist = 0.1;
		Pose_ini.setIdentity();

		pointCloudSource = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		pointCloudTarget = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	}

	Eigen::Matrix4f align_point2plane(float ne_nearest_K, bool limit_to_2d = true)
	{
		pcl::transformPointCloud(*pointCloudSource, *pointCloudSource, Pose_ini);


		pcl::PointCloud<pcl::PointNormal>::Ptr target_PointNormal(new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointNormal>::Ptr source_PointNormal(new pcl::PointCloud<pcl::PointNormal>);
		normal_estmation(pointCloudSource, source_PointNormal);
		normal_estmation(pointCloudTarget, target_PointNormal);


		pcl::registration::TransformationEstimation<pcl::PointNormal, pcl::PointNormal, float>::Ptr
			te(new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal, float>);
		pcl::PointCloud<pcl::PointNormal> pcd_refine;
		pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
		icp.setInputTarget(target_PointNormal);
		icp.setInputSource(source_PointNormal);
		icp.setTransformationEstimation(te);
		icp.setMaxCorrespondenceDistance(outlier_dist);
		icp.setMaximumIterations(max_iter);
		icp.align(pcd_refine);

		Eigen::Matrix4f Pose_final;
		Pose_final = icp.getFinalTransformation() * Pose_ini;

		source_PointNormal->clear();
		target_PointNormal->clear();
		pointCloudSource->clear();
		pointCloudTarget->clear();

		if (limit_to_2d) {
			Pose_final(0,2) = 0.0;
			Pose_final(1,2) = 0.0;
		    Pose_final(2,0) = 0.0;
		    Pose_final(2,1) = 0.0;
		    Pose_final(2,2) = 1.0;
		    Pose_final(2,3) = 0.0;
		}

		return Pose_final;
	}

	// require surface normal estimation, using nearst K neighbors for computation
	Eigen::Matrix4f align_point2plane_2(float ne_nearest_K)
	{

		pcl::registration::TransformationEstimation<pcl::PointNormal, pcl::PointNormal, float>::Ptr
						te(new pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal, float>);
		pcl::PointCloud<pcl::PointNormal>::Ptr target_PointNormal(new pcl::PointCloud<pcl::PointNormal>);
					pcl::PointCloud<pcl::PointNormal>::Ptr source_PointNormal(new pcl::PointCloud<pcl::PointNormal>);


		for (int i(0); i<max_iter; i++) {

			pcl::transformPointCloud(*pointCloudSource, *pointCloudSource, Pose_ini);
			normal_estmation(pointCloudSource, source_PointNormal);
			normal_estmation(pointCloudTarget, target_PointNormal);
			pcl::PointCloud<pcl::PointNormal> pcd_refine;
			pcl::IterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> icp;
			icp.setInputTarget(target_PointNormal);
			icp.setInputSource(source_PointNormal);
			icp.setTransformationEstimation(te);
			icp.setMaxCorrespondenceDistance(outlier_dist);
			icp.setMaximumIterations(1);

			icp.align(pcd_refine);
			Eigen::Matrix4f transform = icp.getFinalTransformation();
			transform(0,2) = 0.0;
			transform(1,2) = 0.0;
			transform(2,0) = 0.0;
			transform(2,1) = 0.0;
			transform(2,2) = 1.0;
			transform(2,3) = 0.0;

			Pose_ini = transform * Pose_ini;
			source_PointNormal->clear();
			target_PointNormal->clear();
		}

		return Pose_ini;
	}

	Eigen::Matrix4f align_point2point()
	{
		removeFarPoints(pointCloudSource);
		removeFarPoints(pointCloudTarget);
		pcl::transformPointCloud(*pointCloudSource, *pointCloudSource, Pose_ini);

		pcl::PointCloud<pcl::PointXYZ> pcd_refine;
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputTarget(pointCloudTarget);
		icp.setInputSource(pointCloudSource);
		icp.setMaxCorrespondenceDistance(outlier_dist);
		icp.setMaximumIterations(max_iter);
		icp.align(pcd_refine);

		Eigen::Matrix4f Pose_final;
		Pose_final = icp.getFinalTransformation() * Pose_ini;

		pointCloudSource->clear();
		pointCloudTarget->clear();

		return Pose_final;
	}

	//Eigen::Matrix4f align_point2plane();

	void setSourcePointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_source)
	{
		for (int i = 0; i < pcd_source->size(); i++)
		{
			pcl::PointXYZ pt;
			pt.x = pcd_source->at(i).x;
			pt.y = pcd_source->at(i).y;
			pt.z = pcd_source->at(i).z;
			pointCloudSource->push_back(pt);
		}
		removeFarPoints(pointCloudSource);
	}

	void setTargetPointCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_target)
	{
		for (int i = 0; i < pcd_target->size(); i++)
		{
			pcl::PointXYZ pt;
			pt.x = pcd_target->at(i).x;
			pt.y = pcd_target->at(i).y;
			pt.z = pcd_target->at(i).z;
			pointCloudTarget->push_back(pt);
		}
		removeFarPoints(pointCloudTarget);
	}

	void setSourceInitialPose(Eigen::Matrix4f Pose_ini_)
	{
		Pose_ini = Pose_ini_;
	}

	void setOutlierRejectionDist(float dist_)
	{
		outlier_dist = dist_;
	}

	void setFarDist(float dist_)
	{
		far_dist = dist_;
	}

	void setMaxIter(float iter)
	{
		max_iter = iter;
	}

private:	
	void removeFarPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd)
	{
		pcl::PointCloud<pcl::PointXYZ> pcd_tmp;
		for (int i = 0; i < pcd->size(); i++)
		{
			pcl::PointXYZ pt = pcd->at(i);

			if (pcl_isinf(pcd->at(i).x) || pcl_isinf(pcd->at(i).y) || pcl_isinf(pcd->at(i).z))
			{
				continue;
			}
			if (pcd->at(i).z > far_dist)
			{
				continue;
			}
			pcd_tmp.push_back(pt);
		}
		pcd->clear();
		for (int i = 0; i < pcd_tmp.size(); i++)
		{
			pcl::PointXYZ pt = pcd_tmp[i];
			pcd->push_back(pt);
		}
	}

	void normal_estmation(pcl::PointCloud<pcl::PointXYZ>::Ptr pcd, pcl::PointCloud<pcl::PointNormal>::Ptr output_PointNormal)
	{
		pcl::PointCloud<pcl::Normal> normals;
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
		normal_estimation.setInputCloud(pcd);
		normal_estimation.setSearchSurface(pcd);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
		normal_estimation.setSearchMethod(search_tree);
		normal_estimation.setKSearch(50);
		normal_estimation.compute(normals);
		for (int i = 0; i < normals.size(); i++)
		{
			pcl::PointXYZ pt = (*pcd)[i];
			pcl::Normal nl = normals[i];
			if (pcl_isinf(nl.normal_x) || pcl_isinf(nl.normal_y) || pcl_isinf(nl.normal_z))
			{
				continue;
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


	Eigen::Matrix4f Pose_ini;
	float outlier_dist;
	float far_dist;
	int max_iter;

	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudSource;
	pcl::PointCloud<pcl::PointXYZ>::Ptr pointCloudTarget;
	
};
}
