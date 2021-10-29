// Author: Keenan Burnett
#include <deque>
#include <algorithm>
#include <vector>
#include <random>
#include <chrono>
#include <string>
#include <utility>
#include <opencv2/core.hpp>
#include "utils/zeus_pcl.hpp"

namespace zeus_pcl {

static float getFloatFromByteArray(auto *byteArray, uint index) {
    return *( (float *)(byteArray + index));
}

static uint16_t getUint16FromByteArray(auto *byteArray, uint index) {
    return *( (uint16_t *)(byteArray + index));
}

int fromPCLRGB(const pcl::PointCloud<pcl::PointXYZRGB>& pclIn, PointCloudPtr cloudOut, std::shared_ptr<Color> color) {
	std::vector<int> dyn_score{0,0,0};
	cloudOut->resize(pclIn.size());
	for (size_t i = 0; i < pclIn.size(); ++i) {
		PointXYZ pt(pclIn.points[i].x, pclIn.points[i].y, pclIn.points[i].z,
				pclIn.points[i].r, pclIn.points[i].g, pclIn.points[i].b);
		if (color) {
			pt.dynamicness = color->getDynamicTypeFromRGB(pt.r, pt.g, pt.b);
			dyn_score.at(pt.dynamicness)++;
		}
		cloudOut->points[i] = pt;
	 }
	return std::distance(dyn_score.begin(),std::max_element(dyn_score.begin(), dyn_score.end()));
}

int fromROSMsg(const sensor_msgs::PointCloud2ConstPtr& ros_msg, PointCloudPtr cloudOut, std::shared_ptr<Color> color) {
	std::vector<int> dyn_score{0,0,0};
    cloudOut->resize(ros_msg->width);
    std::cout << "size: " << ros_msg->width << std::endl;
    uint32_t point_step = ros_msg->point_step;
    uint32_t x_offset = ros_msg->fields[0].offset;  // float (32)
    uint32_t y_offset = ros_msg->fields[1].offset;  // float (32)
    uint32_t z_offset = ros_msg->fields[2].offset;  // float (32)
    uint32_t b_offset = ros_msg->fields[3].offset;
    uint32_t g_offset = ros_msg->fields[3].offset+1;
    uint32_t r_offset = ros_msg->fields[3].offset+2;
    uint j = 0;
    auto *data = ros_msg->data.data();
    uint total_size = ros_msg->width * ros_msg->height;
    for (uint i = 0; i < total_size * point_step; i += point_step) {
        float pt_x = getFloatFromByteArray(data, i + x_offset);
        float pt_y = getFloatFromByteArray(data, i + y_offset);
        float pt_z = getFloatFromByteArray(data, i + z_offset);
        int pt_b = (int)ros_msg->data[i + b_offset];
        int pt_g = (int)ros_msg->data[i + g_offset];
        int pt_r = (int)ros_msg->data[i + r_offset];

        PointXYZ pt(pt_x, pt_y, pt_z, pt_r, pt_g, pt_b);
        if (color) {
        	pt.dynamicness = color->getDynamicTypeFromRGB(pt.r, pt.g, pt.b);
        	dyn_score.at(pt.dynamicness)++;
        }
        cloudOut->points[i] = pt;

        j++;
    }
    return std::distance(dyn_score.begin(),std::max_element(dyn_score.begin(), dyn_score.end()));
}

void fromROSMsg(const sensor_msgs::PointCloud2ConstPtr& ros_msg, IPointCloudPtr cloudOut) {
    cloudOut->resize(ros_msg->width);
    uint32_t point_step = ros_msg->point_step;
    uint32_t x_offset = ros_msg->fields[0].offset;  // float (32)
    uint32_t y_offset = ros_msg->fields[1].offset;  // float (32)
    uint32_t z_offset = ros_msg->fields[2].offset;  // float (32)
    uint32_t i_offset = ros_msg->fields[3].offset;  // float (32)
    // uint32_t r_offset = ros_msg->fields[4].offset;  // uint16_t
    uint j = 0;
    auto *data = ros_msg->data.data();
    for (uint i = 0; i < ros_msg->width * point_step; i += point_step) {
        cloudOut->points[j].x = getFloatFromByteArray(data, i + x_offset);
        cloudOut->points[j].y = getFloatFromByteArray(data, i + y_offset);
        cloudOut->points[j].z = getFloatFromByteArray(data, i + z_offset);
        cloudOut->points[j].intensity = getFloatFromByteArray(data, i + i_offset);
        // cloudOut->points[j].ring = getUint16FromByteArray(data, i + r_offset);
        j++;
    }
}

static void getByteArrayFromFloat(uint8_t byteArray[4], float f) {
    std::memcpy(byteArray, (uint8_t*)(&f), 4);
}

void toROSMsg(PointCloudPtr pc, sensor_msgs::PointCloud2& msg, std::string frame) {
    uint32_t offset = 8;  // float32
    uint32_t point_step = offset * 4;  // x, y, z, intensity
    uint32_t x_offset = 0;
    uint32_t y_offset = 4;
    uint32_t z_offset = 8;
    uint32_t b_offset = 16;
    uint32_t g_offset = 17;
    uint32_t r_offset = 18;
    msg.point_step = point_step;
    msg.width = pc->size();
    msg.height = 1;
    msg.row_step = pc->size() * point_step;
    msg.is_dense = true;
    msg.data.resize(pc->size() * point_step);
    msg.is_bigendian = false;
    uint j = 0;
    for (uint i = 0; i < msg.width * point_step; i += point_step) {
        getByteArrayFromFloat(&msg.data[i + x_offset], pc->points[j].x);
        getByteArrayFromFloat(&msg.data[i + y_offset], pc->points[j].y);
        getByteArrayFromFloat(&msg.data[i + z_offset], pc->points[j].z);
        msg.data[i + b_offset] = pc->points[j].b;
        msg.data[i + g_offset] = pc->points[j].g;
        msg.data[i + r_offset] = pc->points[j].r;
        j++;
    }

    sensor_msgs::PointField field;
    field.name = "x";
    field.offset = x_offset;
    field.datatype = 7;
    field.count = 1;
    msg.fields.push_back(field);

    field.name = "y";
    field.offset = y_offset;
    field.datatype = 7;
    field.count = 1;
    msg.fields.push_back(field);

    field.name = "z";
    field.offset = z_offset;
    field.datatype = 7;
    field.count = 1;
    msg.fields.push_back(field);

    field.name = "rgb";
    field.offset = b_offset;
    field.datatype = 7;
    field.count = 1;
    msg.fields.push_back(field);

    msg.header.frame_id = frame;
}

void applyDynamicness(PointCloudPtr cloudIn, std::shared_ptr<Color> color) {
	std::cout << "[JQ7] Apply dynamic type from RGB" << std::endl;
	if (!cloudIn || !color) throw std::runtime_error("Color null");
    for (uint i = 0; i < cloudIn->size(); i++) {
    	PointXYZ& pt = cloudIn->points[i];
    	int dynamicness = color->getDynamicTypeFromRGB(pt.r, pt.g, pt.b);
    	pt.dynamicness = dynamicness;
    }
}

void applyDynamicness(PointCloudPtr cloudIn, int type) {
	if (!cloudIn) throw std::runtime_error("Color null");
	int r, g, b;
	switch(type) {
	case 0:
		r = 0;
		g = 0.75 * 255;
		b = 0;
		break;
	case 1:
		r = 0.7 * 255;
		g = 0.6 * 255;
		b = 0;
		break;
	case 2:
		r = 0.8 * 255;
		g = 0;
		b = 0;
		break;
	}
    for (uint i = 0; i < cloudIn->size(); i++) {
    	cloudIn->points[i].dynamicness = type;
    	// cloudIn->points[i].r = r;
    	// cloudIn->points[i].g = g;
    	// cloudIn->points[i].b = b;
    }
}

void divideByDynamicness(const PointCloudPtr cloudIn, std::vector<PointCloudPtr>& cloudsOut, std::shared_ptr<Color> color) {
    for (uint i = 0; i < cloudIn->size(); i++) {
    	PointXYZ& pt = cloudIn->points[i];
    	int dynamicness = color->getDynamicTypeFromRGB(pt.r, pt.g, pt.b);
    	pt.dynamicness = dynamicness;
    	cloudsOut[dynamicness]->points.push_back(pt);
    }
}

PointCloudPtr filterAndCombinePlanes(const std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>& pcl_planes, std::shared_ptr<Color> color) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_pcl(new pcl::PointCloud<pcl::PointXYZRGB>);

	PointCloudPtr pcd(new PointCloud);

	for (size_t i(0); i<pcl_planes.size(); i++) {
		auto& pcl_plane_ptr = pcl_planes.at(i);
		PointCloudPtr plane_temp(new PointCloud);
		int dynamicness = fromPCLRGB(*pcl_plane_ptr, plane_temp, color);
		applyDynamicness(plane_temp, dynamicness);
		pcd->points.insert(pcd->points.begin(), plane_temp->points.begin(),plane_temp->points.end());
	}
	return pcd;
}

void append(PointCloudPtr cloud_a, PointCloudPtr cloud_b) {
    cloud_a->points.insert(cloud_a->points.end(), cloud_b->points.begin(), cloud_b->points.end());
}

void copyCloud(PointCloudPtr cloudIn, PointCloudPtr cloudOut) {
    cloudOut->resize(cloudIn->size());
    for (uint i = 0; i < cloudIn->size(); i++) {
        cloudOut->points[i] = cloudIn->points[i];
    }
}

void extract_points(PointCloudPtr cloudIn, PointCloudPtr cloudOut, std::vector<int> &indices) {
    cloudOut->resize(indices.size());
    for (uint i = 0; i < indices.size(); i++) {
        cloudOut->points[i] = cloudIn->points[indices[i]];
    }
}

void extract_points(PointCloudPtr cloud, std::vector<int> &indices) {
    for (uint i = 0; i < indices.size(); i++) {
        cloud->points[i] = cloud->points[indices[i]];
    }
    cloud->resize(indices.size());
}

void extract_negative(PointCloudPtr cloud, std::vector<int> &indices) {
    uint j = 0;
    for (uint i = 0; i < cloud->size(); i++) {
        if (!std::binary_search(indices.begin(), indices.end(), i)){
            cloud->points[j] = cloud->points[i];
            j++;
        }
    }
    cloud->resize(j);
}

void passthrough(PointCloudPtr cloudIn, PointCloudPtr cloudOut, float min_x,
    float max_x, float min_y, float max_y, float min_z, float max_z) {
    std::vector<int> indices;
    for (uint i = 0; i < cloudIn->size(); i++) {
        if (cloudIn->points[i].x >= min_x && cloudIn->points[i].x <= max_x &&
            cloudIn->points[i].y >= min_y && cloudIn->points[i].y <= max_y &&
            cloudIn->points[i].z >= min_z && cloudIn->points[i].z <= max_z) {
            indices.push_back(i);
        }
    }
    extract_points(cloudIn, cloudOut, indices);
}

void passthrough(PointCloudPtr cloud, float min_x, float max_x, float min_y,
    float max_y, float min_z, float max_z) {
    std::vector<int> indices;
    for (uint i = 0; i < cloud->size(); i++) {
        if (cloud->points[i].x >= min_x && cloud->points[i].x <= max_x &&
            cloud->points[i].y >= min_y && cloud->points[i].y <= max_y &&
            cloud->points[i].z >= min_z && cloud->points[i].z <= max_z) {
            indices.push_back(i);
        }
    }
    extract_points(cloud, indices);
}

void passthrough(PointCloudPtr cloudIn, PointCloudPtr cloudOut, float min_x,
    float max_x, float min_y, float max_y, std::vector<int> &indices) {
    indices.clear();
    for (uint i = 0; i < cloudIn->size(); i++) {
        if (cloudIn->points[i].x >= min_x && cloudIn->points[i].x <= max_x &&
            cloudIn->points[i].y >= min_y && cloudIn->points[i].y <= max_y) {
            indices.push_back(i);
        }
    }
    extract_points(cloudIn, cloudOut, indices);
}

void get_subset_unclustered(std::vector<int> &input, std::vector<int> &so_far) {
    if (so_far.size() == 0 || input.size() == 0)
        return;
    std::sort(so_far.begin(), so_far.end());
    std::vector<int> output;
    for (uint i = 0; i < input.size(); i++) {
        if (!std::binary_search(so_far.begin(), so_far.end(), input[i]))
            output.push_back(input[i]);
    }
    input = output;
}

void sort_bbs_by_size(std::vector<zeus_msgs::BoundingBox2D> &bbs) {
    std::sort(bbs.begin(), bbs.end(), greater_than_bb());
}

Eigen::Vector4d get_centroid(PointCloudPtr pc, std::vector<float> &cuboid_parameters) {
    Eigen::Vector4d centroid;
    centroid << 0.0, 0.0, 0.0, 0.0;
    int bignum = 1000000;
    float min_x = bignum, min_y = bignum, min_z = bignum;
    float max_x = -1 * bignum, max_y = -1 * bignum, max_z = -1 * bignum;
    for (uint i = 0; i < pc->size(); i++) {
        if (pc->points[i].x < min_x)
            min_x = pc->points[i].x;
        if (pc->points[i].y < min_y)
            min_y = pc->points[i].y;
        if (pc->points[i].z < min_z)
            min_z = pc->points[i].z;
        if (pc->points[i].x > max_x)
            max_x = pc->points[i].x;
        if (pc->points[i].y > max_y)
            max_y = pc->points[i].y;
        if (pc->points[i].z > max_z)
            max_z = pc->points[i].z;
        centroid(0, 0) += pc->points[i].x;
        centroid(1, 0) += pc->points[i].y;
        centroid(2, 0) += pc->points[i].z;
    }
    centroid = centroid / double(pc->size());
    centroid(3, 0) = 1.0;
    cuboid_parameters = {min_x, min_y, min_z, max_x, max_y, max_z};
    return centroid;
}

Eigen::Vector4d get_centroid(PointCloudPtr pc, std::vector<int> &indices) {
    Eigen::Vector4d centroid;
    centroid << 0.0, 0.0, 0.0, 1.0;
    if (indices.size() == 0)
        return centroid;
    for (uint i = 0; i < indices.size(); i++) {
        centroid(0, 0) += pc->points[indices[i]].x;
        centroid(1, 0) += pc->points[indices[i]].y;
        centroid(2, 0) += pc->points[indices[i]].z;
    }
    centroid = centroid / double(indices.size());
    centroid(3, 0) = 1.0;
    return centroid;
}

void to_2d(PointCloudPtr cloudIn, PointCloudPtr cloudOut) {
    cloudOut->resize(cloudIn->size());
    for (uint i = 0; i < cloudIn->size(); i++) {
        cloudOut->points[i] = cloudIn->points[i];
        cloudOut->points[i].z = 0;
    }
}

std::vector<double> getBBox(const PointCloudPtr cluster_pts) {
	if (cluster_pts->size() == 0)
		throw std::runtime_error("Empty array, cannot generate bounding box!");
	std::vector<double> bbox(7); //x, y, z, l, w, h, yaw

	PointCloudPtr cluster_pts_2d (new PointCloud());
	to_2d(cluster_pts, cluster_pts_2d);

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
	centroid(0, 0) = static_cast<float>(pca_analysis.mean.at<double>(0, 0));
	centroid(1, 0) = static_cast<float>(pca_analysis.mean.at<double>(0, 1));
	centroid(3, 0) = 1.0;

	size_t major_axis = eigen_vals[0] > eigen_vals[1] ? 0 : 1;
	size_t minor_axis = 1 - major_axis;
	double yaw = atan2(eigen_vecs[major_axis][0], eigen_vecs[major_axis][1]);

	double max_x = 0, max_y = 0, max_z = 0;
	for (size_t p = 0; p < cluster_pts->size(); p++) {
		Eigen::Vector2d centered_pt(cluster_pts->points[p].x - centroid(0, 0),
				cluster_pts->points[p].y - centroid(1, 0));

		double proj_major_axis = abs(
				centered_pt[0] * eigen_vecs[major_axis][0]
						+ centered_pt[1] * eigen_vecs[major_axis][1]);
		double proj_minor_axis = abs(
				centered_pt[0] * eigen_vecs[minor_axis][0]
						+ centered_pt[1] * eigen_vecs[minor_axis][1]);
		Eigen::Vector2d pt_pca(proj_major_axis, proj_minor_axis);

		if (proj_major_axis > max_x) {
			max_x = proj_major_axis;
		}
		if (proj_minor_axis > max_y) {
			max_y = proj_minor_axis;
		}
		if (abs(cluster_pts->points[p].z / 2) > max_z)
			max_z = abs(cluster_pts->points[p].z / 2);
	}
	centroid(2, 0) = max_z;

	bbox.at(0) = centroid(0, 0);
	bbox.at(1) = centroid(1, 0);
	bbox.at(2) = centroid(2, 0);
	bbox.at(3) = max_x * 2 * 1.0;;
	bbox.at(4) = max_y * 2 * 1.0;;
	bbox.at(5) = max_z * 2 * 1.0;;
	bbox.at(6) = yaw;

	return bbox;
}

void removeGroundPlane(PointCloudPtr groundPrior, PointCloudPtr pc, Eigen::Vector4f &gp_params,
        Eigen::Matrix4f &K, float dist_thres, unsigned int max_iters, bool ret_gp, PointCloudPtr gp) {
    if (pc->size() < 3 || groundPrior->size() < 3)
        return;
    Ransac ransac(groundPrior, dist_thres, max_iters);
    ransac.computeModel();
    Eigen::Vector4f model_coefficients;
    ransac.getModelCoefficients(model_coefficients);
    if (model_coefficients(3, 0) > 0)
        model_coefficients = model_coefficients * -1;
    // Temporal Smoothing for Ground Plane Parameters
    gp_params = gp_params + K * (model_coefficients - gp_params);
    std::vector<int> outliers;
    ransac.setPointCloud(pc);
    ransac.get_outliers(gp_params, outliers);
    extract_points(pc, outliers);
    if (ret_gp) extract_negative(gp, outliers);
}

std::vector<std::pair<std::vector<float>, std::vector<float> > > removeGroundPlane2(PointCloudPtr pc, float alpha,
        float tolerance, float Tm, float Tm_small,
        float Tb, float Trmse, float Tdprev, bool ret_gp, PointCloudPtr gp) {
    if (pc->size() < 3)
        return std::vector<std::pair<std::vector<float>, std::vector<float> > >();
    Himmelsbach himmelsbach(pc);
    himmelsbach.set_alpha(alpha);
    himmelsbach.set_tolerance(tolerance);
    himmelsbach.set_thresholds(Tm, Tm_small, Tb, Trmse, Tdprev);
    std::vector<int> inliers;
    auto ls = himmelsbach.compute_model_and_get_inliers(inliers);
    std::sort(inliers.begin(), inliers.end());
    extract_negative(pc, inliers);
    if (ret_gp) extract_points(gp, inliers);
    return ls;
}

void project_points(PointCloudPtr cloud, PointCloudPtr uv, Eigen::Matrix4f P) {
    copyCloud(cloud, uv);
    for (uint i = 0; i < uv->size(); i++) {
        Eigen::Vector4f x = {uv->points[i].x, uv->points[i].y, uv->points[i].z, 1.0};
        x = P * x;
        uv->points[i].x = x(0, 0) / x(2, 0);
        uv->points[i].y = x(1, 0) / x(2, 0);
        uv->points[i].z = x(2, 0);
    }
}

void transform_cloud(PointCloudPtr cloud, Eigen::Matrix4d T) {
    for (uint i = 0; i < cloud->size(); i++) {
        Eigen::Vector4d x = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1.0};
        x = T * x;
        cloud->points[i].x = x(0, 0);
        cloud->points[i].y = x(1, 0);
        cloud->points[i].z = x(2, 0);
    }
}

void cluster_point_cloud(PointCloudPtr pc, int min_samples, float tolerance, std::vector<std::vector<int>> &labels) {
    Cluster c(pc, tolerance, min_samples, 3);
    c.extract(labels);
}


int filter_clusters(std::vector<std::vector<int>> &clusters, PointCloudPtr bb_pts, zeus_msgs::BoundingBox2D bbox,
    Eigen::Matrix4f CAM, Eigen::Vector4d &centroid_out, float min_height, float max_height) {
    int idx = -1;
    float min = 1000;
    centroid_out << 0, 0, 0, 1;
    for (uint i = 0; i < clusters.size(); i++) {
        Eigen::Vector4d centroid;
        centroid = get_centroid(bb_pts, clusters[i]);
        float h = centroid(0, 0) * bbox.h / CAM(1, 1);
        if (min_height <= h && h <= max_height && 0 <= centroid(0, 0) && centroid(0, 0) < min) {
            min = centroid(0, 0);
            idx = i;
        }
    }
    if (idx >= 0)
        centroid_out = get_centroid(bb_pts, clusters[idx]);
    return idx;
}

void randomDownSample(PointCloudPtr pc, float prob_keep) {
    if (prob_keep < 0 || 1 < prob_keep) {
        std::cerr << "ERROR: prob_keep must be within [0, 1]" << std::endl;
        return;
    }
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    std::vector<int> indices;
    for (int i = 0; i < (int)pc->size(); i++) {
        float x = distribution(generator);
        if (x <= prob_keep)
            indices.push_back(i);
    }
    for (uint i = 0; i < indices.size(); i++) {
        pc->points[i] = pc->points[indices[i]];
    }
    pc->resize(indices.size());
}

void radialDownSample(PointCloudPtr pc) {
    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    std::vector<int> indices;
    double r = 0;
    double prob_keep = 1;
    double x = 0;
    for (int i = 0; i < (int)pc->size(); i++) {
        if (pc->points[i].z < -0.5 && pc->points[i].z > -1.0 && fabs(pc->points[i].y) < 3.0) {
            indices.push_back(i);
        } else {
            r = sqrt(pow(pc->points[i].x, 2) + pow(pc->points[i].y, 2));
            prob_keep = std::min(1.0, 0.15 + 0.03*r);
            x = distribution(generator);
            if (x <= prob_keep) indices.push_back(i);
        }
    }
    for (uint i = 0; i < indices.size(); i++) {
        pc->points[i] = pc->points[indices[i]];
    }
    pc->resize(indices.size());
}

KdTree::KdTree(PointCloudPtr pc_, int dimensionality_) {
    if (dimensionality_ < 0 || dimensionality_ > 3) {
        std::cerr << "Invalid point cloud dimensionality: " << dimensionality_ << std::endl;
        return;
    }
    if (pc_->size() == 0)
        std::cout << "WARNING: Empty point cloud passed to KdTree" << std::endl;
    dimensionality = dimensionality_;
    pc = pc_;
    std::srand(std::time(NULL));
    std::vector<int> indices(pc->size(), 0);
    for (uint i = 0; i < pc->size(); i++) {
        indices[i] = i;
    }
    int comparedim = 0;
    int median_index = sample_median(indices, comparedim);
    if (median_index < 0)
        return;
    root = NodePtr(new Node(median_index));
    root->comparedim = comparedim;
    construct(root, indices);  // Recursively construct KdTree
}

void KdTree::print_tree(NodePtr n) {
    std::cout << n->index << std::endl;
    if (n->hasleft)
        print_tree(n->left);
    if (n->hasright)
        print_tree(n->right);
}

int KdTree::count_tree(NodePtr n) {
    int count = 1;
    if (n->hasleft)
        count += count_tree(n->left);
    if (n->hasright)
        count += count_tree(n->right);
    return count;
}

void KdTree::construct(NodePtr parent, std::vector<int> &indices) {
    if (indices.size() == 0)
        return;
    std::vector<int> lindices;
    std::vector<int> rindices;
    sort_points(parent, indices, lindices, rindices);
    int next_dim = 0;
    if (lindices.size() > 0) {
        int lindex = sample_median(lindices, next_dim);
        parent->left = NodePtr(new Node(lindex));
        parent->hasleft = true;
        parent->left->comparedim = next_dim;
        construct(parent->left, lindices);
    }
    if (rindices.size() > 0) {
        int rindex = sample_median(rindices, next_dim);
        parent->right = NodePtr(new Node(rindex));
        parent->hasright = true;
        parent->right->comparedim = next_dim;
        construct(parent->right, rindices);
    }
}

// Estimate median using only a sample of the input points
// returns the index of the median point
int KdTree::sample_median(std::vector<int> &indices, int &comparedim) {
    if (indices.size() < 1)
        return -1;
    if (indices.size() == 1)
        return indices[0];
    uint size = sample_size;
    if (indices.size() < sample_size)
        size = indices.size();
    std::vector<int> sub_indices(size, -1);
    float xmin = 1000, ymin = 1000, zmin = 1000, xmax = -1000, ymax = -1000, zmax = -1000;
    for (uint i = 0; i < size; i++) {
        if (indices.size() > sample_size)
            sub_indices[i] = indices[std::rand() % indices.size()];
        else
            sub_indices[i] = indices[i];
        if (pc->points[sub_indices[i]].x < xmin)
            xmin = pc->points[sub_indices[i]].x;
        if (pc->points[sub_indices[i]].x > xmax)
            xmax = pc->points[sub_indices[i]].x;
        if (dimensionality > 1) {
            if (pc->points[sub_indices[i]].y < ymin)
                ymin = pc->points[sub_indices[i]].y;
            if (pc->points[sub_indices[i]].y > ymax)
                ymax = pc->points[sub_indices[i]].y;
        }
        if (dimensionality > 2) {
            if (pc->points[sub_indices[i]].z < zmin)
                zmin = pc->points[sub_indices[i]].z;
            if (pc->points[sub_indices[i]].z > zmax)
                zmax = pc->points[sub_indices[i]].z;
        }
    }
    // Find most informative dimension to split on:
    float X = fabs(xmax - xmin);
    float Y = 0;
    if (dimensionality > 1)
        Y = fabs(ymax - ymin);
    float Z = 0;
    if (dimensionality > 2)
        Z = fabs(zmax - zmin);
    comparedim = 0;
    float mean = (xmax + xmin) / 2;
    if (Y > X && Y > Z) {
        comparedim = 1;
        mean = (ymax + ymin) / 2;
    } else if (Z > Y && Z > X) {
        comparedim = 2;
        mean = (zmax + zmin) / 2;
    }
    int closest = 0;
    float min = 1000;
    for (uint i = 0; i < size; i++) {
        float d = min;
        if (comparedim == 0)
            d = fabs(pc->points[sub_indices[i]].x - mean);
        else if (comparedim == 1)
            d = fabs(pc->points[sub_indices[i]].y - mean);
        else if (comparedim == 2)
            d = fabs(pc->points[sub_indices[i]].z - mean);
        if (d < min) {
            min = d;
            closest = i;
        }
    }
    return sub_indices[closest];
}

// sort points into left vs. right side of parent
void KdTree::sort_points(NodePtr parent, std::vector<int> indices, std::vector<int> &lindices,
    std::vector<int> &rindices) {
    for (uint idx = 0; idx < indices.size(); idx++) {
        if (indices[idx] == parent->index)
            continue;
        if (parent->comparedim == 0) {
            if (pc->points[indices[idx]].x <= pc->points[parent->index].x)
                lindices.push_back(indices[idx]);
            else if (pc->points[indices[idx]].x > pc->points[parent->index].x)
                rindices.push_back(indices[idx]);
        } else if (parent->comparedim == 1) {
            if (pc->points[indices[idx]].y <= pc->points[parent->index].y)
                lindices.push_back(indices[idx]);
            else if (pc->points[indices[idx]].y > pc->points[parent->index].y)
                rindices.push_back(indices[idx]);
        } else if (parent->comparedim == 2) {
            if (pc->points[indices[idx]].z <= pc->points[parent->index].z)
                lindices.push_back(indices[idx]);
            else if (pc->points[indices[idx]].z > pc->points[parent->index].z)
                rindices.push_back(indices[idx]);
        }
    }
}

std::vector<int> KdTree::radiusSearch(int idx, float radius_squared) {
    std::vector<int>indices;
    float radius = sqrt(radius_squared);
    xmin = pc->points[idx].x - radius;
    xmax = pc->points[idx].x + radius;
    ymin = pc->points[idx].y - radius;
    ymax = pc->points[idx].y + radius;
    zmin = pc->points[idx].z - radius;
    zmax = pc->points[idx].z + radius;
    radiusSearchHelper(indices, root, idx, radius_squared);
    return indices;
}

void KdTree::radiusSearchHelper(std::vector<int> &indices, NodePtr node, int idx, float radius_squared) {
    if (!node->assigned) {
        if (distance(idx, node->index) < radius_squared) {
            node->assigned = true;
            if (idx != node->index)
                indices.push_back(node->index);
        }
    }
    if (node->comparedim == 0) {
        if (xmin <= pc->points[node->index].x && node->hasleft)
            radiusSearchHelper(indices, node->left, idx, radius_squared);
        if (xmax >= pc->points[node->index].x && node->hasright)
            radiusSearchHelper(indices, node->right, idx, radius_squared);
    } else if (node->comparedim == 1) {
        if (ymin <= pc->points[node->index].y && node->hasleft)
            radiusSearchHelper(indices, node->left, idx, radius_squared);
        if (ymax >= pc->points[node->index].y && node->hasright)
            radiusSearchHelper(indices, node->right, idx, radius_squared);
    } else if (node->comparedim == 2) {
        if (zmin <= pc->points[node->index].z && node->hasleft)
            radiusSearchHelper(indices, node->left, idx, radius_squared);
        if (zmax >= pc->points[node->index].z && node->hasright)
            radiusSearchHelper(indices, node->right, idx, radius_squared);
    }
}

float KdTree::distance(int id1, int id2) {
    return (pc->points[id1] - pc->points[id2]).sq_norm();
}

float KdTree::distance(NodePtr n1, NodePtr n2) {
    return (pc->points[n1->index] - pc->points[n2->index]).sq_norm();
}

Cluster::Cluster(PointCloudPtr pc_, float tolerance_, int minClusterSize_, int dimensionality) :
    pc(pc_), minClusterSize(minClusterSize_) {
    if (pc->size() == 0) {
        std::cout << "WARNING: given pointcloud pc_ is empty, returning!" << std::endl;
        return;
    }
    tolerance_squared = tolerance_ * tolerance_;
    if (pc->size() > 30)
        kdt =  KdTreePtr(new KdTree(pc, dimensionality));
    assigned = std::vector<bool>(pc->size(), false);
    indices = std::vector<int>(pc->size(), -1);
    for (uint i = 0; i < pc->size(); i++) {
        indices[i] = i;
    }
}

void Cluster::extract(std::vector<std::vector<int>> &clusters) {
    clusters.clear();
    for (uint i = 0; i < pc->size(); i++) {
        int seed = nextSeedIndex();
        if (seed < 0)
            break;
        if (assigned[seed])
            continue;
        std::vector<int> cluster;
        std::deque<int> q;
        q.push_back(seed);
        while (q.size() > 0) {
            int idx = q[0];
            q.pop_front();
            cluster.push_back(idx);
            assigned[idx] = true;
            std::vector<int> neighbors;
            if (pc->size() > 30)
                neighbors = kdt->radiusSearch(idx, tolerance_squared);
            else
                neighbors = linearRadiusSearch(idx, tolerance_squared);
            for (int neighbor : neighbors) {
                q.push_back(neighbor);
            }
        }
        clusters.push_back(cluster);
    }
    // prune clusters that don't reach the minimum size
    for (int i = clusters.size() - 1; i >= 0; i--) {
        if ((int)clusters[i].size() < minClusterSize)
            clusters.erase(clusters.begin() + i);
    }
}

std::vector<int> Cluster::linearRadiusSearch(int idx, float radius_squared) {
    std::vector<int> indices;
    for (uint i = 0; i < pc->size(); i++) {
        if (!assigned[i]) {
            if (distance(idx, i) < radius_squared) {
                indices.push_back(i);
                assigned[i] = true;
            }
        }
    }
    return indices;
}

float Cluster::distance(int id1, int id2) {
    return (pc->points[id1] - pc->points[id2]).sq_norm();
}

int Cluster::nextSeedIndex() {
    current_index++;
    if (current_index >= (int)indices.size())
        return -1;
    return current_index;
}

template <class PointType>
int Ransac::get_num_inliers(PointType n, PointType p) {
    int num_inliers = 0;
    for (uint i = 0; i < pc->size(); i++) {
        float d = fabs(n.dot(pc->points[i] - p));
        if (d < tolerance)
            num_inliers++;
    }
    return num_inliers;
}

void Ransac::get_outliers(Eigen::Vector4f model, std::vector<int> &outliers) {
    outliers.clear();
    for (uint i = 0; i < pc->size(); i++) {
        float d = fabs(model(0) * pc->points[i].x + model(1) * pc->points[i].y + model(2) * pc->points[i].z + model(3));
        d = d / sqrt(pow(model(0), 2) + pow(model(1), 2) + pow(model(2), 2));
        if (d > tolerance)
            outliers.push_back(i);
    }
}

/*!
   \brief Returns a random subset of indices, where 0 <= indices[i] <= max_index. indices are non-repeating.
*/
static std::vector<int> random_subset(int max_index, int subset_size) {
    std::vector<int> subset;
    if (max_index < 0 || subset_size < 0)
        return subset;
    if (max_index < subset_size)
        subset_size = max_index;
    subset = std::vector<int>(subset_size, -1);
    for (uint i = 0; i < subset.size(); i++) {
        while (subset[i] < 0) {
            int idx = std::rand() % max_index;
            if (std::find(subset.begin(), subset.begin() + i, idx) == subset.begin() + i)
                subset[i] = idx;
        }
    }
    return subset;
}

void Ransac::computeModel() {
    int max_inliers = 0;
    for (int i = 0; i < iterations; i++) {
        std::vector<int> subset = random_subset(pc->size(), 3);
        if (subset.size() < 3)
            continue;
        auto a = pc->points[subset[0]] - pc->points[subset[1]];
        auto b = pc->points[subset[2]] - pc->points[subset[1]];
        // check for collinearity
        float cos_theta = a.dot(b) / (a.sq_norm() * b.sq_norm());
        if (cos_theta > collinear_angle_threshold)
            continue;
        auto n = a.cross(b);
        n.normalize();
        int num_inliers = get_num_inliers(n, pc->points[subset[1]]);
        if (num_inliers > max_inliers) {
            best_model = {n[0], n[1], n[2], 0.0};
            best_model(3) = -1 * n.dot(pc->points[subset[1]]);
            max_inliers = num_inliers;
        }
        if (float(num_inliers) / float(pc->size()) > inlier_ratio)
            break;
    }
}

Line::Line(PointCloudPtr pc, std::vector<int> line_set, float m_, float b_) {
    int idx = line_set[0];
    start_idx = idx;
    start = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
    idx = line_set[line_set.size() - 1];
    end_idx = idx;
    end = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
    m = m_;
    b = b_;
}

void Himmelsbach::sort_points_segments(std::vector<std::vector<int>> &segments) {
    for (uint i = 0; i < pc->size(); i++) {
        float angle = atan2(pc->points[i].y, pc->points[i].x);
        if (angle < 0)
            angle = angle + 2 * M_PI;
        int segment = int(angle / alpha);
        segments[segment].push_back(i);
    }
}

void Himmelsbach::sort_points_bins(const std::vector<int>& segment, std::vector<int> &bins_out) {
    std::vector<std::vector<int>> bins(num_bins_small + num_bins_large);
    float rsmall = rmin + bin_size_small * num_bins_small;
    for (int idx : segment) {
        float r = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
        int bin = -1;
        if (rmin <= r && r < rsmall)
            bin = (r - rmin) / bin_size_small;
        if (rsmall <= r && r < rmax)
            bin = num_bins_small + (r - rsmall) / bin_size_large;
        if (bin >= 0)
            bins[bin].push_back(idx);
    }
    // The point with the lowest z-coordinate in each bin becomes the representative point
    bins_out = std::vector<int>(num_bins_small + num_bins_large, -1);
    int i = 0;
    for (std::vector<int> bin_points : bins) {
        float zmin = 0.5;
        int lowest = -1;
        for (int idx : bin_points) {
            if (pc->points[idx].z < zmin) {
                zmin = pc->points[idx].z;
                lowest = idx;
            }
        }
        bins_out[i] = lowest;
        i++;
    }
}

float Himmelsbach::fitline(const std::vector<int>& line_set, float &m, float &b) {
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(line_set.size(), 2);
    Eigen::VectorXf B = Eigen::VectorXf::Zero(line_set.size());
    for (uint i = 0; i < line_set.size(); i++) {
        int idx = line_set[i];
        A(i, 0) = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
        A(i, 1) = 1;
        B(i) = pc->points[idx].z;
    }
    Eigen::VectorXf x = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(B);
    m = x(0);
    b = x(1);
    Eigen::VectorXf error = A * x - B;
    return sqrt(error.dot(error) / float(line_set.size()));
}

float Himmelsbach::fitline(std::vector<int> line_set, int idx, float &m, float &b) {
    line_set.push_back(idx);
    return fitline(line_set, m, b);
}

float Himmelsbach::distpointline(const Line& line, int idx) {
    float r = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
    return fabs(pc->points[idx].z - line.m * r - line.b) / sqrt(1 + pow(line.m, 2));
}

std::vector<std::pair<std::vector<float>, std::vector<float>>> Himmelsbach::compute_model_and_get_inliers(
        std::vector<int> &inliers) {
    if (pc->size() == 0)
        return std::vector<std::pair<std::vector<float>, std::vector<float> > >();
    int num_segments = ceil((2 * M_PI) / alpha) + 1;
    std::vector<std::vector<int>> segments(num_segments);
    // Sort points into segments
    sort_points_segments(segments);
    std::vector<std::pair<std::vector<float>, std::vector<float> > > ls;
    for (auto& segment : segments) {
        // Sort points into bins
        if (segment.size() == 0)
            continue;
        std::vector<int> bins;
        sort_points_bins(segment, bins);
        std::vector<Line> lines;
        std::vector<int> line_set;
        int c = 0;
        for (int idx : bins) {
            if (idx < 0) {
                continue;
            } else if (line_set.size() >= 2) {
                float m = 1.0;
                float b = 0.0;
                float rmse = fitline(line_set, idx, m, b);
                if (fabs(m) <= Tm && (fabs(m) > Tm_small || fabs(b) <= Tb)
                        && rmse <= Trmse) {
                    line_set.push_back(idx);
                } else {
                    fitline(line_set, m, b);
                    auto line = Line(pc, line_set, m, b);
                    lines.push_back(line);
                    line_set.clear();
                    c++;
                    idx--;
                }
            } else {
                float dprev = 10000;
                if (lines.size() > 0 && (c - 1) >= 0)
                    dprev = distpointline(lines[c - 1], idx);
                if (dprev <= Tdprev || c == 0 || line_set.size() != 0)
                    line_set.push_back(idx);
            }
        }
        std::cout << "num of lines " << lines.size() << std::endl;
        // Assign points as inliers if they are within a threshold of the ground model
        for (int idx : segment) {
            // get line that's closest to the candidate point based on distance to endpoints
            int closest = -1;
            float dmin = 10000;
            float r = sqrt(pow(pc->points[idx].x, 2) + pow(pc->points[idx].y, 2));
            for (uint i = 0; i < lines.size(); i++) {
                float d1 = fabs(lines[i].start - r);
                float d2 = fabs(lines[i].end - r);
                if ((d1 < dmin || d2 < dmin) && (fabs(lines[i].m) < Tm)) {
                    dmin = std::min(d1, d2);
                    closest = i;
                }
            }
            if (closest >= 0) {
                auto e = distpointline(lines[closest], idx);
                if (e < tolerance) {
                    inliers.push_back(idx);
                } else {
                	std::cout << "outlier " << e << std::endl;
                }
            } else {
            	//std::cout << "no closest " << std::endl;
            }
        }

        for (const auto& l : lines) {
            float angle = atan2(pc->points[l.start_idx].y, pc->points[l.start_idx].x);
            if (angle < 0)
                angle = angle + 2 * M_PI;
            float r = l.end - l.start;
            float spos_x = l.start * cos(angle);
            float spos_y = l.start * sin(angle);
            float spos_z = l.start * l.m + l.b;
            float epos_x = l.end * cos(angle);
            float epos_y = l.end * sin(angle);
            float epos_z = l.end * l.m + l.b;

            ls.push_back(std::make_pair(std::vector<float>{spos_x,spos_y,spos_z},std::vector<float>{epos_x,epos_y,epos_z}));
        }

    }
    return ls;
}
}  // namespace zeus_pcl
