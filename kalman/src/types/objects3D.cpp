// Author: Keenan Burnett
#include <pcl/filters/uniform_sampling.h>

#include "types/objects3D.hpp"
#include <iostream>
#include <vector>

int Object::getType() {
    return hmm->get_type();
}

bool Object::checkFlashing(int type, float lower, float upper) {
    float ratio  = 0;
    float count = 0;
    if ((int)past_types.size() == filter_length) {
        for (uint i = 0; i < past_types.size(); i++) {
            if (past_types[i] == type)
                count += 1;
        }
        ratio = count / float(filter_length);
        if (lower < ratio && ratio < upper)
            return true;
    }
    return false;
}

void Object::push_type(int t, std::vector<float> class_confidences) {
    hmm->filter(t, class_confidences);
    past_types.push_back(t);
    if ((int)past_types.size() > filter_length) {
        past_types.pop_front();
    }
}

double Object::getLostTime(double t) {
    if (t > last_observed_time)
        return t - last_observed_time;
    else
        return 0.0;
}

double Object::getAge(double t) {
    if (t > first_observed_time)
        return t - first_observed_time;
    else
        return 0.0;
}

std::vector<double> Object::mergeNewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl) {
	auto cloud_merged = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB> >(*cloud + *cloud_pcl);
	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::UniformSampling<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud_merged);
	filter.setRadiusSearch(0.05f);
	filter.filter(*cloud);

	zeus_pcl::PointCloudPtr cloud_merged_zpcl(new zeus_pcl::PointCloud());
	zeus_pcl::fromPCLRGB(*cloud, cloud_merged_zpcl);

	auto bbox = zeus_pcl::getBBox(cloud_merged_zpcl);

	return bbox;
}
