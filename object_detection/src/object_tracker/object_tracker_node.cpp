// Author: Keenan Burnett
#include "object_tracker/object_tracker_node.h"
#include <vector>

void convertToRosMessage(std::vector<Object> object_list,
		zeus_msgs::Detections3D &outputDetections, float yaw);

void KalmanTrackerNode::init() {
	float scale, relative, obj_conf_;

	XmlRpc::XmlRpcValue Q_, R_;

	std::string node_name = ros::this_node::getName();
	nh_.getParam(node_name + "/scale", scale);
	nh_.getParam(node_name + "/relative", relative);
	nh_.getParam(node_name + "/static_obj_conf", obj_conf_);

	nh_.getParam(node_name + "/Q", Q_);
	nh_.getParam(node_name + "/R", R_);

	int xdim = 5;
	int ydim = 3;
	float T = 0.1;
	Eigen::MatrixXd A = Eigen::MatrixXd::Identity(xdim, xdim);   // Motion model
	Eigen::MatrixXd C = Eigen::MatrixXd::Zero(ydim, xdim);  // Observation model

	Eigen::MatrixXd Q_all = Eigen::MatrixXd::Identity(xdim, xdim); // Process noise
	Eigen::MatrixXd R_all = Eigen::MatrixXd::Identity(ydim, ydim); // Measurement noise
	Eigen::MatrixXd Q_dynamic = Eigen::MatrixXd::Identity(xdim, xdim); // Process noise
	Eigen::MatrixXd R_dynamic = Eigen::MatrixXd::Identity(ydim, ydim); // Measurement noise

	Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(xdim, xdim); // Initial Covariance guess
	A(0, 3) = T;
	A(1, 4) = T;
	C(0, 0) = 1;
	C(1, 1) = 1;
	C(2, 2) = 1;
	for (int i = 0; i < 5; i++) {
		Q_all(i, i) = Q_[i];  // Process Noise
		Q_dynamic(i, i) = Q_[i];
	}
	for (int i = 0; i < 3; i++) {
		R_all(i, i) = R_[i];  // Measurement Noise
		R_dynamic(i, i) = R_[i];
	}

	Q_all = Q_all * scale;
	Q_dynamic = Q_dynamic * scale;
	R_all = R_all * scale * relative;
	R_dynamic = R_dynamic * scale * relative;
	P0 = Q_all * 10;

	std::map<std::string, Eigen::MatrixXd> Q { { "all", Q_all }, { "dynamic",
			Q_dynamic } };
	std::map<std::string, Eigen::MatrixXd> R { { "all", R_all }, { "dynamic",
			R_dynamic } };

	kalmantracker = KalmanPtr(new kalman::KalmanTracker(A, C, Q, R, P0));
	std::vector<double> one_d_kalman_gains, point_cloud_range;
	double confidence_drop, metricGate, metric_thres, delete_time, cost_alpha,
			cost_beta;
	int pedestrian_type, unknown_type, unknown_dynamic_type;
	nh_.getParam(node_name + "/one_d_kalman_gains", one_d_kalman_gains);
	nh_.getParam(node_name + "/point_cloud_range", point_cloud_range);
	nh_.getParam(node_name + "/confidence_drop", confidence_drop);
	nh_.getParam(node_name + "/metricGate", metricGate);
	nh_.getParam(node_name + "/metric_thres", metric_thres);
	nh_.getParam(node_name + "/delete_time", delete_time);
	nh_.getParam(node_name + "/cost_alpha", cost_alpha);
	nh_.getParam(node_name + "/cost_beta", cost_beta);
	nh_.getParam(node_name + "/pedestrian_type", pedestrian_type);
	nh_.getParam(node_name + "/unknown_type", unknown_type);
	nh_.getParam(node_name + "/unknown_dynamic_type", unknown_dynamic_type);
	kalmantracker->set1DKalmanGains(one_d_kalman_gains);
	kalmantracker->setPointCloudRange(point_cloud_range);
	kalmantracker->setConfidenceDrop(confidence_drop);
	kalmantracker->setMetricGate(metricGate);
	kalmantracker->setMetricThres(metric_thres);
	kalmantracker->setDeleteTime(delete_time);
	kalmantracker->setCostParameters(cost_alpha, cost_beta);
	kalmantracker->setPedestrianType(pedestrian_type);
	kalmantracker->setUnknownType(unknown_type);
	kalmantracker->setUnknownDynamicType(unknown_dynamic_type);
	kalmantracker->setStaticObjConf(obj_conf_);
	kalmantracker->setDynamicObjConf(obj_conf_);

	std::vector<double> zeros { 0, 0 };
	kalmantracker->setDeerDims(zeros, zeros, zeros);
	kalmantracker->setBarrelDims(zeros, zeros, zeros);
	kalmantracker->setPedDims(zeros, zeros, zeros);

	// HMM Parameters
	std::vector<int> types;
	XmlRpc::XmlRpcValue A_, C_, pi_;
	nh_.getParam(node_name + "/types", types);
	nh_.getParam(node_name + "/A_hmm", A_);
	nh_.getParam(node_name + "/C_hmm", C_);
	nh_.getParam(node_name + "/pi_hmm", pi_);
	int num_types = types.size();
	Eigen::MatrixXd A_hmm, C_hmm, pi_hmm;
	A_hmm = Eigen::MatrixXd::Zero(num_types, num_types);
	C_hmm = Eigen::MatrixXd::Zero(num_types, num_types);
	pi_hmm = Eigen::MatrixXd::Zero(num_types, 1);
	for (int i = 0; i < num_types; i++) {
		pi_hmm(i, 0) = pi_[i];
		for (int j = 0; j < num_types; j++) {
			A_hmm(i, j) = A_[num_types * i + j];
			C_hmm(i, j) = C_[num_types * i + j];
		}
	}
	kalmantracker->setHMMParameters(types, A_hmm, C_hmm, pi_hmm);

	initialize_transforms();
}

zeus_msgs::Detections3D KalmanTrackerNode::track(const zeus_msgs::Detections3D &det,
		const Eigen::Matrix4d &T_oc) {
	std::cout << "Raw det received " << det.bbs.size() << std::endl;
	auto start = std::chrono::high_resolution_clock::now();
	std::vector<zeus_msgs::BoundingBox3D> dets = det.bbs;
	//Eigen::Matrix4d Toi = Eigen::Matrix4d::Identity();
	//zeus_tf::get_odom_tf(*odom, Toi);
	//Eigen::Matrix4d Toc = Toi * Tic2;
	kalmantracker->setCurrentTime(det.header.stamp.toSec());
	//! Perform data association between the new detections (dets) and the existing object tracks
	kalmantracker->association(dets, T_oc);
	//! Linear Kalman filter update
	kalmantracker->filter(dets, T_oc);
	//! Prune objects that are closer than metricGate to each other or objects outside point_cloud_range.
	kalmantracker->prune(zeus_tf::get_inverse_tf(T_oc));
	//! Publish ROS message:
	zeus_msgs::Detections3D outputDetections;
	outputDetections.header.stamp = det.header.stamp;
	outputDetections.header.frame_id = world_frame_id;
	outputDetections.bbs.clear();
	std::vector<Object> object_list = kalmantracker->get_object_list();
	Eigen::Vector3d rpy = T_oc.block<3,3>(0,0).transpose().eulerAngles(0, 1, 2);
	//zeus_tf::get_rpy_from_odom(*odom, rpy);
	convertToRosMessage(object_list, outputDetections, (float)rpy[2]);
	//convertToRosMessage(object_list, outputDetections, (float)0);
	det_pub_.publish(outputDetections);
	auto stop = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = stop - start;
	ROS_DEBUG_STREAM("[OBJ] KALMAN TRACKER TIME: " << elapsed.count());
	return outputDetections;
}

void KalmanTrackerNode::initialize_transforms() {
	//tfBuffer = zeus_tf::tfBufferPtr(new zeus_tf::tfBuffer());
	//tfListener = zeus_tf::tfListenerPtr(new zeus_tf::tfListener(*tfBuffer));
	//zeus_tf::get_transform(tfBuffer, "c2", "imu_link", Tic2);
	//ROS_INFO_STREAM("[OBJ] object_tracker transforms initialized");
}

// Given an object list, formats the appropriate ROS output message for object detection
void convertToRosMessage(std::vector<Object> object_list,
		zeus_msgs::Detections3D &outputDetections, float yaw) {
	for (int i = 0; i < (int) object_list.size(); i++) {
		zeus_msgs::BoundingBox3D detection;
		detection.x = object_list[i].x_hat(0, 0);
		detection.y = object_list[i].x_hat(1, 0);
		detection.z = object_list[i].x_hat(2, 0);
		detection.x_dot = object_list[i].x_hat(3, 0);
		detection.y_dot = object_list[i].x_hat(4, 0);
		detection.w = object_list[i].w;
		detection.l = object_list[i].l;
		detection.h = object_list[i].h;
		detection.yaw = object_list[i].yaw + yaw;
		detection.ID = object_list[i].ID;
		detection.type = object_list[i].type;
		detection.camera = object_list[i].camera;
		detection.confidence = object_list[i].confidence;
		outputDetections.bbs.push_back(detection);
	}
}
