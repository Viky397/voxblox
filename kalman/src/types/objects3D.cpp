// Author: Keenan Burnett
#include <pcl/filters/uniform_sampling.h>

#include "types/objects3D.hpp"
#include <iostream>
#include <vector>
#include <math.h>

#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/uniform.hpp>

int Object::getType() {
    return type;
}

void Object::set_type(int type_) {
    type = type_;
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
	zeus_pcl::PointCloudPtr cloud_merged_temp(new zeus_pcl::PointCloud());
	zeus_pcl::PointCloudPtr cloud_prev(new zeus_pcl::PointCloud());
	zeus_pcl::fromPCLRGB(*cloud_merged, cloud_merged_temp);
	zeus_pcl::fromPCLRGB(*cloud, cloud_prev);
	auto bbox_new = zeus_pcl::getBBox(cloud_merged_temp);

	if (bbox_new[3] > 3.5 || bbox_new[4] > 3.5) {
		return zeus_pcl::getBBox(cloud_prev);
	}

	cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

    pcl::UniformSampling<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud_merged);
	filter.setRadiusSearch(0.03f);
	filter.filter(*cloud);

	return bbox_new;
}

void Object::updateProbability(double change, double std_change) {
	std::cout << "[JQ12] Object " << ID << " amount of change " << change << " std " << std_change <<std::endl;

	double s_weight = 1;

	if (type == 0 && inlier == false) s_weight = 3;  // dynamic outlier, drop fast
	if (type == 0 && inlier == true)  s_weight = 0;  // dynamic inlier, rise slow
	if (type == 1 && inlier == false) s_weight = 0;  // static outlier, drop slow
	if (type == 1 && inlier == true)  s_weight = 3;  // static inlier, rise fast
	if (type == 2 && inlier == false) s_weight = 5;  // disappeared outlier, drop very fast
	if (type == 2 && inlier == true)  s_weight = 0;  // disappeared inlier, rise slow

	type = std::min(type, 1);

	//s_weight = 0;

	std::cout << "[JQ12]   inlier " << inlier << std::endl;
	std::cout << "[JQ12]   s_weight " << s_weight << std::endl;

	double tolerance = 20.0 * std_change;

	double s_sq = 1.0 / (1.0 / (pow(sig, 2)) + 1.0 / (pow(std_change, 2)));
	double m = s_sq * (mu / (pow(sig, 2)) + change / (pow(std_change, 2)));

	auto Kvals = K(s_weight);
	double K1 = Kvals.first;
	double K2 = Kvals.second;

	std::cout << "[JQ12]   K1 " << K1 << "  K2 " << K2 << std::endl;

	boost::math::normal_distribution<double> norm_dist(mu, sig);
	boost::math::uniform_distribution<double> uniform_dist(0.0, tolerance);

	// double C1 = (a / (a + b)) * boost::math::pdf(norm_dist, change);
	double C1 = K1 * std::max(boost::math::pdf(norm_dist, change), eps);
	double C2 = fabs(change) >= tolerance-eps ? K2 * boost::math::pdf(uniform_dist, tolerance) : K2 * boost::math::pdf(uniform_dist, fabs(change));
	C1 = max(eps, C1);
	C2 = max(eps, C2);

	double C_norm = C1 + C2;
	C1 /= C_norm;
	C2 /= C_norm;


	std::cout << "[JQ12]   C1 " << C1 << "  C2 " << C2 << std::endl;

	inlier = C1>=C2 ? true : false;

	double mu_prime = C1 * m + C2 * mu;
	sig = sqrt(C1 * (s_sq + pow(m, 2)) + C2 * (pow(sig, 2) + pow(mu, 2)) - pow(mu_prime, 2));

    double gamma = (a + type*s_weight + 1)/(a + b + s_weight + 1);
    double eta = (a + type*s_weight)/(a + b + s_weight + 1);
    double theta = C1*gamma + C2*eta;
    double alpha = ( (a+type*s_weight+2)*(a+type*s_weight+1)/( (a+b+s_weight+1)*(a+b+s_weight+2) ) );
    double beta = ( (a+type*s_weight+1)*(a+type*s_weight)/( (a+b+s_weight+1)*(a+b+s_weight+2) ) );

    mu = mu_prime;
    a = (C1*theta*alpha + beta*C2*theta - pow(theta,2))/(pow(theta,2) - C1*alpha - C2*beta);
    b = ( (C1*theta*alpha + beta*C2*theta - pow(theta,2))*(1 - C1*gamma - C2*eta) / ( (pow(theta,2) - C1*alpha - C2*beta)* (C1*gamma + C2*eta)) );

    double cap = 25;
    if (a>cap || b>cap) {
        double ratio = max(a,b) / cap;
        a /= ratio;
        b /= ratio;
    }

    //std::cout << "[JQ12]   a " << a << "  b " << b << std::endl;

	confidence = a / (a + b);

	//std::cout << "[JQ12]   confidence " << confidence << std::endl;

	life++;

	//std::cout << "[JQ]   updated static prob of " << confidence << std::endl;
}

bool Object::expectedToObserve(Pose2 cam_pose, float fov) {
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr samples(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::UniformSampling<pcl::PointXYZRGB> filter;
	filter.setInputCloud(cloud);
	filter.setRadiusSearch(0.25f);
	filter.filter(*samples);

	if (samples->points.size() == 0) return false;

	float num_pts_expected = 0;
	for (const auto& p : samples->points) {
		Pose2 lm(p.x, p.y, 0);
		Pose2 T_c_l = lm - cam_pose;
		if (T_c_l.x() > 0.1 && T_c_l.norm() < 6 && fabs(atan2(T_c_l.y(), T_c_l.x()))/M_PI*180 < fabs(fov*0.85/2.0)) {
			num_pts_expected++;
		}
	}

	if ((num_pts_expected / samples->points.size()) > 0.35) return true;

	return false;
}

std::pair<double, double> Object::K(double k) const {
    double lk1 = (lgamma(a+b) + lgamma(a+k*type+1) + lgamma(b+k-k*type)) - (lgamma(a) + lgamma(b) + lgamma(a+b+k+1));
    double lk2 = (lgamma(a+b) + lgamma(a+k*type) + lgamma(b+k-k*type+1)) - (lgamma(a) + lgamma(b) + lgamma(a+b+k+1));

    double k1 = exp(lk1);
    double k2 = exp(lk2);

    double ks = k1+k2;

    k1 /= ks;
    k2 /= ks;

    k1 = max(eps, k1);
    k2 = max(eps, k2);

    return std::make_pair(k1, k2);
}





