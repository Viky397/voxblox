// Author: Keenan Burnett
#include <pcl/filters/uniform_sampling.h>

#include "types/objects3D.hpp"
#include <iostream>
#include <vector>

#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/uniform.hpp>

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

void Object::updateProbability(double change, double std_change) {
	std::cout << "[JQ] Object " << ID << " amount of change " << change << std::endl;
	double s_sq = 1.0 / (1.0 / (pow(sig, 2)) + 1.0 / (pow(std_change, 2)));
	double m = s_sq * (mu / (pow(sig, 2)) + change / (pow(std_change, 2)));

	boost::math::normal_distribution<double> norm_dist(mu, sig);
	boost::math::uniform_distribution<double> uniform_dist(0.0, 1.0);

	// double C1 = (a / (a + b)) * boost::math::pdf(norm_dist, change);
	double C1 = (a / (a + b)) * std::max(boost::math::pdf(norm_dist, change), 0.01);
	double C2 = 0.0;
	if (change <= 1.0) C2 = (b / (a + b)) * boost::math::pdf(uniform_dist, change);

	double C_norm = C1 + C2;
	C1 /= C_norm;
	C2 /= C_norm;

	double mu_prime = C1 * m + C2 * mu;
	sig = sqrt(C1 * (s_sq + pow(m, 2)) + C2 * (pow(sig, 2) + pow(mu, 2)) - pow(mu_prime, 2));

	double f = C1 * (a + 1.0) / (a + b + 1.0) + C2 * a / (a + b + 1.0);
	double e = C1 * (a + 1.0) * (a + 2.0) / ((a + b + 1.0) * (a + b + 2.0))
			  + C2 * a * (a + 1.0) / ((a + b + 1.0) * (a + b + 2.0));

	mu = mu_prime;

	a = (e - f) / (f - e / f);
	b = a * (1.0 - f) / f ;

	confidence = a / (a + b);

	std::cout << "[JQ]   updated static prob of " << confidence << std::endl;
}
