// Author: Keenan Burnett
#include <limits>
#include <map>
#include <algorithm>
#include <vector>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

#include "utils/kalman.hpp"
#include "utils/transform_utils.hpp"
#include "utils/association.hpp"
#include "utils/zeus_pcl.hpp"
#include "utils/hmm.hpp"
#include "utils/alignment.hpp"

namespace kalman {

/*!
   \brief Returns the euclidean ground plane distance between an object (x) and a detection (det)
*/
static double dist(Eigen::VectorXd x, zeus_msgs::BoundingBox3D det) {
    return sqrt(pow((x(0, 0) - det.x), 2) + pow((x(1, 0) - det.y), 2));
}

/*!
   \brief Returns the euclidean ground plane distance between two objects (x) and (y)
*/
static double dist(Eigen::VectorXd x, Eigen::VectorXd y) {
    return sqrt(pow((x(0, 0) - y(0, 0)), 2) + pow((x(1, 0) - y(1, 0)), 2));
}

const std::vector<Object>& KalmanTracker::get_object_list() {
    return X;
}

std::vector<Object>& KalmanTracker::get_mutable_object_list() {
    return X;
}

void KalmanTracker::print() {
    for (uint i = 0; i < X.size(); i++) {
        std::cout << X[i] << std::endl;
    }
}

/*!
   \brief Transforms detections from the sensor frame to odom frame according to Toc.
*/
static void transform_dets(std::vector<zeus_msgs::BoundingBox3D> &dets, Eigen::Matrix4d Toc) {
    if (dets.size() == 0)
        return;
    Eigen::MatrixXd xbar = Eigen::MatrixXd::Ones(4, 1);
    for (uint i = 0; i < dets.size(); i++) {
        xbar << dets[i].x, dets[i].y, dets[i].z, 1.0;
        xbar = Toc * xbar;
        dets[i].x = xbar(0, 0);
        dets[i].y = xbar(1, 0);
        dets[i].z = xbar(2, 0);
    }
}

static bool is_member(std::vector<int> v, int x) {
    return std::find(v.begin(), v.end(), x) != v.end();
}

void KalmanTracker::association(const std::vector<zeus_msgs::BoundingBox3D>& dets, Eigen::Matrix4f robot_pose) {
	for (auto& obj : X) {
		obj.is_observed = false;
		obj.is_new = false;
	}
    indices = std::vector<int>(X.size(), -1);
    auto change_signs = std::vector<float>(X.size(), 0);
    std::vector<int> notassoc;
    for (uint j = 0; j < dets.size(); j++) {
        notassoc.push_back(j);
    }
    // Check if detections correspond to one of our existing tracks
    // indices[i] = det_index or (-1)
    if (X.size() > 0)
        optimalAssociationSM(dets, indices, notassoc, change_signs, robot_pose, current_time);
    // Initialize a new object for each unassigned detection
    int M2 = int(notassoc.size());
    for (int j = 0; j < M2; j++) {
        X.push_back(create_new(dets[notassoc[j]], objectID, current_time));
        indices.push_back(-1);
        //std::cout << "[JQ10] Add object " << X.back().ID << std::endl;
    }

    for (uint i = 0; i < X.size(); i++) {
        if (indices[i] >= 0) {
            if (is_object_tracker) {
                std::vector<float> confidences(1, 0.8);
                X[i].set_type(dets[indices[i]].type);
            }

            X[i].camera = dets[indices[i]].camera;
            if (current_time < X[i].last_observed_time) {
                indices[i] = -1;
                continue;
            }
            X[i].delta_t = current_time - X[i].last_observed_time;
            X[i].last_observed_time = current_time;
            X[i].is_observed = true;
            X[i].change_sign = 1;
            //std::cout << "[JQ30]   object: " << i << " change is " <<  change_signs[i] << std::endl;
        }
    }
}

std::vector<pcl::PointCloud<pcl::PointXYZRGB> > KalmanTracker::filter(std::vector<zeus_msgs::BoundingBox3D> &dets, Eigen::Matrix4f robot_pose) {
    if (X.size() == 0)
        return std::vector<pcl::PointCloud<pcl::PointXYZRGB> >{};
    if (indices.size() < X.size()) {
        ROS_INFO_STREAM("ERROR: There are less indices than objects, returning!");
        return std::vector<pcl::PointCloud<pcl::PointXYZRGB> >{};
    }
    Pose2 robot_pose2(robot_pose);

    int N = X.size();
    Eigen::MatrixXd x_check = Eigen::MatrixXd::Zero(xdim, 1);
    Eigen::MatrixXd P_check = Eigen::MatrixXd::Zero(xdim, xdim);
    Eigen::MatrixXd Q_adj, R_adj;
    double dist_thresh = 30.0;   // distance threshold for inflating measurement noise
    bool adjust = true;          // turn off/on covarinace adjustment

    pcl::PointCloud<pcl::PointXYZRGB> sources;
    pcl::PointCloud<pcl::PointXYZRGB> targets;
    pcl::PointCloud<pcl::PointXYZRGB> refines;

    for (int i = 0; i < N; i++) {
        X[i].type = X[i].getType();     // Get the most likely type based on the HMM
        Eigen::MatrixXd Q, R;
        if (is_object_tracker) {
            Q = Q_dynamic;
            R = R_dynamic;
        } else {
            Q = Q_all;
            R = R_all;
        }
        int j = indices[i];
        if (j >= 0) {
            Q_adj = Q;
            R_adj = R;

            /**
            A(0, 3) = X[i].delta_t;
            A(1, 4) = X[i].delta_t;
            x_check = A * X[i].x_hat;
            P_check = A * X[i].P_hat * A.transpose() + Q_adj;
            Eigen::Vector4d ybar = {dets[j].x, dets[j].y, dets[j].z, 1.0};
            ybar = Toc * ybar;
            Eigen::Vector3d y = {ybar(0, 0), ybar(1, 0), ybar(2, 0)};
            Eigen::MatrixXd temp = C * P_check * C.transpose() + R_adj;
            Eigen::MatrixXd K = (P_check * C.transpose()) * temp.inverse();
            X[i].x_hat = x_check + K * (y - C * x_check);
            X[i].P_hat = (Eigen::MatrixXd::Identity(xdim, xdim) - K * C) * P_check;
            X[i].w = X[i].w + one_d_kalman_gains[0] * (dets[j].w - X[i].w);
            X[i].l = X[i].l + one_d_kalman_gains[1] * (dets[j].l - X[i].l);
            X[i].h = X[i].h + one_d_kalman_gains[2] * (dets[j].h - X[i].h);
            //std::cout << "X[i].yaw before " << X[i].yaw << std::endl;
            //std::cout << "dets[j].yaw " << dets[j].yaw << std::endl;
            //X[i].yaw = X[i].yaw + one_d_kalman_gains[3] * (dets[j].yaw - X[i].yaw);
            X[i].yaw = dets[j].yaw;
            //std::cout << "X[i].yaw after " << X[i].yaw << std::endl;
            X[i].confidence = X[i].confidence + one_d_kalman_gains[4] * (dets[j].confidence - X[i].confidence);
            **/

            A(0, 3) = X[i].delta_t;
            A(1, 4) = X[i].delta_t;
            x_check = A * X[i].x_hat;
            P_check = A * X[i].P_hat * A.transpose() + Q_adj;
            Eigen::Vector4d ybar = {dets[j].x, dets[j].y, dets[j].z, 1.0};
            //ybar = Eigen::Matrix4d::Identity() * ybar;
            Eigen::Vector3d y = {ybar(0, 0), ybar(1, 0), ybar(2, 0)};
            Eigen::MatrixXd temp = C * P_check * C.transpose() + R_adj;
            Eigen::MatrixXd K = (P_check * C.transpose()) * temp.inverse();

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_refine(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::fromROSMsg(dets[j].cloud, *cloud_pcl);

            X[i].new_obs.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
            *X[i].new_obs = *cloud_pcl;

            X[i].y_prev(0,0) = dets[j].x;
            X[i].y_prev(1,0) = dets[j].y;
            X[i].y_prev(2,0) = dets[j].z;
            X[i].P_hat = Eigen::MatrixXd::Identity(xdim, xdim);

            sources += *X[i].new_obs;
            targets += *X[i].cloud;

            pcl::transformPointCloud(*cloud_pcl, *pcd_refine, sc_tfs.at(std::make_pair(i, j)));
            refines += *pcd_refine;
        } else {
        	std::cout << "[JQ8] Checking unassociated object " << X[i].ID << std::endl;
        	if (X[i].expectedToObserve(robot_pose2+Pose2(0.43,0,0), 75) && !X[i].is_new) {
        		X[i].set_type(0);
        		X[i].updateProbability(100, 0.00001);
        		X[i].new_obs.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
        		std::cout << "[JQ10] Lower confidence for object (not in FOV) " << X[i].ID << " with conf " << X[i].confidence << std::endl;
        	}
        }
        X[i].last_updated = current_time;
    }
    return std::vector<pcl::PointCloud<pcl::PointXYZRGB> >{sources, targets, refines};
}

void KalmanTracker::mergeObservation(int ID) {
	for (auto& obj : X) {
		if (obj.ID != ID) continue;
		auto bbox = obj.mergeNewCloud(obj.new_obs);
		obj.x_hat(0,0) = bbox[0];
		obj.x_hat(1,0) = bbox[1];
		obj.x_hat(2,0) = bbox[2];
		obj.P_hat = Eigen::MatrixXd::Identity(xdim, xdim);
		obj.l = bbox[3];
		obj.w = bbox[4];
		obj.h = bbox[5];
		obj.yaw = bbox[6];
	}
}

void KalmanTracker::prune(Eigen::Matrix4f robot_pose, bool prune_by_confidence) {
	std::vector<Object> Xout;
	std::vector<int> delete_indices;

    // Then prune objects that are too close to each other or overlap
    for (uint i = 0; i < X.size(); i++) {
        for (uint j = i + 1; j < X.size(); j++) {
            double interArea = calculate_intersection_rotated(X[i], X[j]);
            if ((interArea / X[i].get2DArea() > ioa_thres) || (interArea / X[j].get2DArea() > ioa_thres)) {
                // bigger one wins
                if (X[i].get2DArea() < X[j].get2DArea()) {
                    delete_indices.push_back(i);
                    X[j].mergeNewCloud(X[i].cloud);
                    //X[j].ID = min(X[i].ID,X[j].ID);
                    X[j].is_observed = true;
                    std::cout << "[JQ10] Merge object " << X[i].ID << " to object " << X[j].ID << " inter: " << interArea/X[i].get2DArea() << std::endl;
                } else {
                    delete_indices.push_back(j);
                    X[i].mergeNewCloud(X[j].cloud);
                    //X[i].ID = min(X[i].ID,X[j].ID);
                    X[i].is_observed = true;
                    std::cout << "[JQ10] Merge objects " << X[j].ID << " to object " << X[i].ID << " inter: " << interArea/X[j].get2DArea() << std::endl;
                }

            }

        }
        if (prune_by_confidence && X[i].confidence < 0.4 && X[i].life > 2) {
            delete_indices.push_back(i);
            std::cout << "[JQ3] Remove object for low confidence " << X[i].ID << std::endl;
        }
    }
    Xout.clear();

    for (uint i = 0; i < X.size(); i++) {
        if (!is_member(delete_indices, i)) {
            Xout.push_back(X[i]);
        }
    }
    X = Xout;
}

/*!
   \brief Get the Intersection over union between two bounding boxes.
   x1,y1,x2,y2 correspond to the top-left and bottom-right corners of the first box
*/
static double calculate_iou(double x1, double y1, double x2, double y2, double x1t, double y1t,
                            double x2t, double y2t) {
    if (x2 < x1t || x1 > x2t || y2 < y1t || y1 > y2t)
        return 0;
    double xa = std::max(x1, x1t);
    double ya = std::max(y1, y1t);
    double xb = std::min(x2, x2t);
    double yb = std::min(y2, y2t);
    double intersection = (xb - xa + 1) * (yb - ya + 1);
    double area = (x2 - x1 + 1) * (y2 - y1 + 1);
    double areat = (x2t - x1t + 1) * (y2t - y1t + 1);
    double un = area + areat - intersection;
    double iou = intersection / un;
    return iou;
}

/*!
   \brief Converts 3D point xin (in odom frame) into 2D image plane location (u,v)
*/
static void convert3dto2d(Eigen::VectorXd xin, int &u, int &v, float wx, float hx, int &w, int &h,
                          Eigen::Matrix4d CAM, Eigen::Matrix4d Tco) {
    Eigen::MatrixXd x_w = Eigen::MatrixXd::Ones(4, 1);
    Eigen::MatrixXd x_c = Eigen::MatrixXd::Ones(4, 1);
    x_w(0, 0) = xin(0, 0);
    x_w(1, 0) = xin(1, 0);
    x_w(2, 0) = xin(2, 0);
    x_c = Tco * x_w;
    double z = x_c(0, 0);
    double x = -x_c(1, 0);
    double y = -x_c(2, 0);
    u = CAM(0, 0) * (x / z) + CAM(0, 2);
    v = CAM(1, 1) * (y / z) + CAM(1, 2);
    w = CAM(0, 0) * wx / z;
    h = CAM(1, 1) * hx / z;
}

/*!
   \brief returns (1 - IOU) as a distance metric
*/
static double distIOU(Eigen::VectorXd x, float wx, float hx, Eigen::VectorXd xdet, float wdet, float hdet,
                      Eigen::Matrix4d CAM, Eigen::Matrix4d Tco) {
    double x1t = 0, y1t = 0, x2t = 0, y2t = 0;
    double x1 = 0, y1 = 0, x2 = 0, y2 = 0;
    int u = 0, v = 0, w = 0, h = 0;
    convert3dto2d(xdet, u, v, wdet, hdet, w, h, CAM, Tco);
    x1t = u - w / 2;
    y1t = v - h / 2;
    x2t = u + w / 2;
    y2t = v + h / 2;
    convert3dto2d(x, u, v, wx, hx, w, h, CAM, Tco);
    x1 = u - w / 2;
    y1 = v - h / 2;
    x2 = u + w / 2;
    y2 = v + h / 2;
    double iou = calculate_iou(x1, y1, x2, y2, x1t, y1t, x2t, y2t);
    double d = 1 - iou;
    if (d < 0)
        d = 0.0;
    return d;
}

Object KalmanTracker::create_new(const zeus_msgs::BoundingBox3D &det, int &objectID, double current_time) {
    Object x;
    x.x_hat(0, 0) = det.x;
    x.x_hat(1, 0) = det.y;
    x.x_hat(2, 0) = det.z;
    x.x_hat(3, 0) = 0.0;
    x.x_hat(4, 0) = 0.0;
    x.w = det.w;
    x.l = det.l;
    x.h = det.h;
    x.yaw = det.yaw;
    x.hmm = HMMPtr(new HiddenMarkovModel(A_hmm, C_hmm, pi_hmm, types));
    x.filter_length = filter_length;
    // auto cloud = boost::make_shared<sensor_msgs::PointCloud2>(det.cloud);
    pcl::fromROSMsg(det.cloud, *x.cloud);
    if (det.class_confidences.size() > 0) {
        x.set_type(det.type);
    } else {
        std::vector<float> confidences(1, det.confidence);
        x.set_type(det.type);
    }
    x.type = x.getType();
    x.ID = objectID;
    objectID++;
    x.P_hat = P0;
    x.camera = det.camera;
    x.is_observed = true;
    x.is_new = true;
    x.last_observed_time = current_time;
    x.first_observed_time = current_time;
    x.last_updated = current_time;
    x.delta_t = 0.1;
    x.initPrior();
    return x;
}

bool KalmanTracker::checkIfIDPresent(int ID) {
    for (uint i = 0; i < X.size(); i++) {
        if (ID == X[i].ID)
            return true;
    }
    return false;
}
void KalmanTracker::add_new(const zeus_msgs::BoundingBox3D &det, int ID, double current_time) {
    Object obj = create_new(det, ID, current_time);
    X.push_back(obj);
}

void KalmanTracker::removeObjectsNotInList(std::vector<int> keepIDs) {
    std::vector<Object> Xout;
    for (uint i = 0; i < X.size(); i++) {
        if (is_member(keepIDs, X[i].ID))
            Xout.push_back(X[i]);
    }
    X = Xout;
}

void KalmanTracker::setIndices(std::map<int64_t, int> associations) {
    indices = std::vector<int>(X.size(), -1);
    for (uint i = 0; i < X.size(); i++) {
        indices[i] = associations[X[i].ID];
    }
}

void KalmanTracker::pushTypes(std::vector<zeus_msgs::BoundingBox3D> dets, double current_time) {
    for (uint i = 0; i < X.size(); i++) {
        if (indices[i] >= 0) {
            if (dets[indices[i]].class_confidences.size() > 0) {
                X[i].set_type(dets[indices[i]].type);
            } else {
                std::vector<float> confidences(1, dets[indices[i]].confidence);
                X[i].set_type(dets[indices[i]].type);
            }
            X[i].last_observed_time = current_time;
        }
    }
}

void KalmanTracker::pruneOld(double current_time) {
    std::vector<Object> Xout;
    for (uint i = 0; i < X.size(); i++) {
        if (X[i].getLostTime(current_time) < delete_time) {
            Xout.push_back(X[i]);
        }
    }
    X = Xout;
}

Eigen::MatrixXd KalmanTracker::generateCostMatrixSM(const std::vector<zeus_msgs::BoundingBox3D> &dets,
                                                  std::vector<int> dets_indices,
                                                  std::vector<int> object_indices,
                                                  double &infeasible_cost,
												  const Eigen::Matrix4f& T,
												  Eigen::MatrixXd& changeSignMatrix) {
    uint N = object_indices.size();
    uint M = dets_indices.size();
    sc_tfs.clear();
    Eigen::MatrixXd costMatrix = Eigen::MatrixXd::Zero(N, M);
    changeSignMatrix = Eigen::MatrixXd::Zero(N, M);
    if (N == 0 || M == 0)
        return costMatrix;

    double EPS = 1e-10;
    double INF = std::numeric_limits<double>::infinity();
    infeasible_cost = INF;
    double max_value = 0;
    for (uint i = 0; i < N; i++) {
    	Eigen::Vector3d x = A * X[object_indices[i]].x_hat;
    	//std::cout << "[JQ10] Object: " << X[i].ID << std::endl;
        for (uint j = 0; j < M; j++) {
        	double d_euclid = dist(x, dets[dets_indices[j]]);
        	if (d_euclid > 10*metricGate) {
        		costMatrix(i, j) = INF;
        		continue;
        	}

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::fromROSMsg(dets[dets_indices[j]].cloud, *cloud_pcl);

            Eigen::Matrix4f sc_tf;
            float conf = matchPCDs(cloud_pcl, X[i], 60, sc_tf);
            double d_sm = norm(sc_tf);

            Eigen::Matrix4f sc_body = T.inverse() * sc_tf * T;

            //std::cout << "[JQ30]   i  j: " << i << "  " <<  j << std::endl;
            //std::cout << "[JQ30]      bdx  bdy: " << sc_body(0,3) << "  " <<  sc_body(1,3) << std::endl;
            //std::cout << "[JQ30]      dsm: " << d_sm << std::endl;

            int sign = 1;
            if (abs(sc_body(0,3)) < 0.05) sign = 1;
            else if (sc_body(0,3) > 0) sign = 1;
            else sign = -1;
            changeSignMatrix(i, j) = sign;

            //std::cout << "[JQ10]    j: " << j << std::endl;
            //std::cout << "[JQ10]       d_sm: " << d_sm << "   conf: " << conf << std::endl;


            if (d_sm > metricGate || d_sm < EPS || conf < 0.9) {
                costMatrix(i, j) = INF;
                continue;
            }

            costMatrix(i, j) = d_sm + cost_alpha * abs(dets[dets_indices[j]].type - X[i].type);// + (1 - conf);
            sc_tfs[std::make_pair(i,j)] = sc_tf;
            if (costMatrix(i, j) > max_value && costMatrix(i, j) != INF)
                max_value = costMatrix(i, j);
        }
    }
    if (max_value > 0) {
        // Replace INF with maximum non-infinite cost in costMatrix:
        for (uint i = 0; i < costMatrix.rows(); i++) {
            for (uint j = 0; j < costMatrix.cols(); j++) {
                if (costMatrix(i, j) == INF) {
                    costMatrix(i, j) = max_value + 100;
                    infeasible_cost = max_value + 99;
                }
            }
        }
    }
    return costMatrix;
}

Eigen::MatrixXd KalmanTracker::generateCostMatrix(const std::vector<zeus_msgs::BoundingBox3D> &dets,
                                                  std::vector<int> dets_indices,
                                                  std::vector<int> object_indices,
                                                  double &infeasible_cost) {
    uint N = object_indices.size();
    uint M = dets_indices.size();
    Eigen::MatrixXd costMatrix;
    if (N == 0 || M == 0)
        return costMatrix;
    costMatrix = Eigen::MatrixXd::Zero(N, M);
    double EPS = 1e-15;
    double INF = std::numeric_limits<double>::infinity();
    infeasible_cost = INF;
    double max_value = 0;
    for (uint i = 0; i < N; i++) {
        Eigen::MatrixXd x = X[object_indices[i]].x_hat;
        x = A * x;
        for (uint j = 0; j < M; j++) {
            double d_euclid = dist(x, dets[dets_indices[j]]);
            double mgate = metricGate;
            if (d_euclid > mgate) {
                costMatrix(i, j) = INF;
                continue;
            }
            d_euclid = d_euclid / mgate;
            double d_size = 0.33 * (fabs(X[i].h - dets[dets_indices[j]].h) / (X[i].h + EPS) +
                fabs(X[i].w - dets[dets_indices[j]].w) / (X[i].w + EPS) +
                fabs(X[i].l - dets[dets_indices[j]].l) / (X[i].l + EPS));
            if (d_size > 1)
                d_size = 1;
            double d_conf = 1 - dets[dets_indices[j]].confidence;
            double d_type = fabs(X[i].type - dets[dets_indices[j]].type);
            costMatrix(i, j) = d_euclid + cost_alpha * d_type;
            if (costMatrix(i, j) > max_value && costMatrix(i, j) != INF)
                max_value = costMatrix(i, j);
        }
    }
    if (max_value > 0) {
        // Replace INF with maximum non-infinite cost in costMatrix:
        for (uint i = 0; i < costMatrix.rows(); i++) {
            for (uint j = 0; j < costMatrix.cols(); j++) {
                if (costMatrix(i, j) == INF) {
                    costMatrix(i, j) = max_value + 1;
                    infeasible_cost = max_value + 1;
                }
            }
        }
    }
    return costMatrix;
}

/*!
   \brief Given a vector of objects and a vector of detections, this method determines the euclidean mean
   (the centroid) of all the objects and points. This is used to prevent numerical rounding errors during association.
*/
static void get_centroid(const std::vector<Object>& X, const std::vector<zeus_msgs::BoundingBox3D>& dets,
                         std::vector<double> &centroid) {
    centroid = std::vector<double>(3, 0.0);
    for (uint i = 0; i < X.size(); i++) {
        centroid[0] += X[i].x_hat(0, 0);
        centroid[1] += X[i].x_hat(1, 0);
        centroid[2] += X[i].x_hat(2, 0);
    }
    for (uint i = 0; i < dets.size(); i++) {
        centroid[0] += dets[i].x;
        centroid[1] += dets[i].y;
        centroid[2] += dets[i].z;
    }
    centroid[0] /= double(X.size() + dets.size());
    centroid[1] /= double(X.size() + dets.size());
    centroid[2] /= double(X.size() + dets.size());
}

/*!
   \brief During data association, it is convenient to stack the object indices and detection indices into a single
   vector. This function converts a single vector of object and detection indices into two separate vectors of each.
   \pre Objects correspond to indices 0 to N -1 and dets to N through N + M -1
*/
static void disentangle_indices(std::vector<int> &cluster, int M, int N, std::vector<int> &dets,
                                std::vector<int> &objs) {
    dets.clear();
    objs.clear();
    for (uint k = 0; k < cluster.size(); k++) {
        if (cluster[k] < N)
            objs.push_back(cluster[k]);
        if (cluster[k] >= N)
            dets.push_back(cluster[k] - N);
    }
}

float KalmanTracker::matchPCDs(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, const Object& target, int max_iter, Eigen::Matrix4f& final_pose) {
	kalman::Alignment2 matcher;
	matcher.setFarDist(100000);
	// set ICP iterations
	matcher.setMaxIter(max_iter);
	// set ourlier rejection
	matcher.setOutlierRejectionDist(1.0);
	// set input point cloud
	matcher.setSourcePointCloud(source);
	matcher.setTargetPointCloud(target.cloud);
	// match
	final_pose = matcher.align_point2point();

	return matcher.getConfidence();
}

void KalmanTracker::optimalAssociation(const std::vector<zeus_msgs::BoundingBox3D> &dets, std::vector<int> &indices,
                                       std::vector<int> &notassoc, double current_time) {
    uint N = X.size();
    uint M = dets.size();
    if (N == 0) {
        indices.clear();
        notassoc = std::vector<int>(M, -1);
        return;
    }
    indices = std::vector<int>(N, -1);
    if (M == 0) {
        indices = std::vector<int>(N, -1);
        notassoc.clear();
        return;
    }
    // Cluster objects and detections together
    zeus_pcl::PointCloudPtr pc(new zeus_pcl::PointCloud());
    pc->points.resize(N + M);
    std::vector<double> centroid;
    get_centroid(X, dets, centroid);
    for (uint i = 0; i < N; i++) {
        pc->points[i].x = float(X[i].y_prev(0, 0) - centroid[0]);
        pc->points[i].y = float(X[i].y_prev(1, 0) - centroid[1]);
        pc->points[i].z = float(X[i].y_prev(2, 0) - centroid[2]);
    }
    for (uint i = N; i < N + M; i++) {
        pc->points[i].x = float(dets[i - N].x - centroid[0]);
        pc->points[i].y = float(dets[i - N].y - centroid[1]);
        pc->points[i].z = float(dets[i - N].z - centroid[2]);
    }
    std::vector<std::vector<int>> clusters;
    zeus_pcl::cluster_point_cloud(pc, 1, (float) metricGate, clusters);


    notassoc.clear();
    // Loop over the object-detection clusters
    for (uint i = 0; i < clusters.size(); i++) {
        std::vector<int> cluster = clusters[i];
        bool has_det = false;
        bool has_obj = false;
        for (uint k = 0; k < cluster.size(); k++) {
            if (cluster[k] < (int) N)
                has_obj = true;
            if (cluster[k] >= (int) N)
                has_det = true;
        }
        // tracks without detections
        if (!has_det)
            continue;
        // detections not associated to a track
        if (!has_obj) {
            for (uint k = 0; k < cluster.size(); k++) {
                notassoc.push_back(cluster[k] - N);
            }
            continue;
        }
        // Check for isolated object-detection pair
        if (cluster.size() == 2) {
            int obj = -1;
            int det = -1;
            for (uint k = 0; k < cluster.size(); k++) {
                if (cluster[k] < (int) N)
                    obj = cluster[k];
                if (cluster[k] >= (int) N)
                    det = cluster[k] - N;
            }
            indices[obj] = det;
            continue;
        }
        // We have a cluster with >= 1 detections and >= 1 objects
        // Generate cost matrix and use an optimal solver
        std::vector<int> object_indices;
        std::vector<int> dets_indices;
        disentangle_indices(cluster, M, N, dets_indices, object_indices);
        double infeasible_cost = std::numeric_limits<double>::infinity();
        Eigen::MatrixXd costMatrix = generateCostMatrix(dets, dets_indices, object_indices, infeasible_cost);
        std::vector<int> assignments = kalman::association(costMatrix, 0, infeasible_cost);
        for (uint i = 0; i < assignments.size(); i++) {
            if (assignments[i] < 0)
                indices[object_indices[i]] = -1;
            else
                indices[object_indices[i]] = dets_indices[assignments[i]];
        }
    }
    for (uint j = 0; j < M; j++) {
        if (!is_member(indices, j))
            notassoc.push_back(j);
    }
}


void KalmanTracker::optimalAssociationSM(const std::vector<zeus_msgs::BoundingBox3D> &dets, std::vector<int> &indices,
                                       std::vector<int> &notassoc, std::vector<float>& change_signs, const Eigen::Matrix4f& T, double current_time) {
    uint N = X.size();
    uint M = dets.size();
    if (N == 0) {
        indices.clear();
        notassoc = std::vector<int>(M, -1);
        return;
    }
    indices = std::vector<int>(N, -1);
    if (M == 0) {
        indices = std::vector<int>(N, -1);
        notassoc.clear();
        return;
    }

    Eigen::MatrixXd changeSignMatrix;

    notassoc.clear();
    std::vector<int> object_indices;
    for (uint i(0); i<N; i++) object_indices.push_back(i);

    std::vector<int> dets_indices;
    for (uint j(0); j<M; j++) dets_indices.push_back(j);

    double infeasible_cost = std::numeric_limits<double>::infinity();

    Eigen::MatrixXd costMatrix = generateCostMatrixSM(dets, dets_indices, object_indices, infeasible_cost, T , changeSignMatrix);
    std::vector<int> assignments = kalman::association(costMatrix, 0, infeasible_cost);

    for (uint i = 0; i < assignments.size(); i++) {
        if (assignments[i] < 0) {
            indices[object_indices[i]] = -1;
            change_signs[object_indices[i]] = 0;
        } else {
        	double cost = costMatrix(object_indices[i], dets_indices[assignments[i]]);
            if (cost <= infeasible_cost) {
            	indices[object_indices[i]] = dets_indices[assignments[i]];
            	change_signs[object_indices[i]] = changeSignMatrix(object_indices[i], dets_indices[assignments[i]]);
            } else {
            	indices[object_indices[i]] = -1;
            	change_signs[object_indices[i]] = 0;
            }
        }
    }
    for (uint j = 0; j < M; j++) {
        if (!is_member(indices, j))
            notassoc.push_back(j);
    }
}

void KalmanTracker::drawBBs() {
	cv::Mat image(1800, 1800, CV_8UC3, cv::Scalar(0));
	double scale = 500; // 100
	double x_shift = 0; // 1400
	double y_shift = 0; //400
	for (const auto& obj : X) {
		double x_coord = -obj.x_hat(0) * scale + x_shift;
		double y_coord = -obj.x_hat(1) * scale + y_shift;
		cv::RotatedRect rect(cv::Point2f(y_coord,x_coord), cv::Size2f(obj.l*scale,obj.w*scale), obj.yaw/M_PI*180);
		cv::Point2f v[4];
		rect.points(v);
		for (int i(0); i<4; i++){
			if (obj.type == 0)
				cv::line(image, v[i], v[(i+1)%4], cv::Scalar(0,255,0));
			else if (obj.type == 1)
				cv::line(image, v[i], v[(i+1)%4], cv::Scalar(0,150,150));
			else if (obj.type == 2)
				cv::line(image, v[i], v[(i+1)%4], cv::Scalar(0,0,255));
		}
	}
	//cv::imshow("BBs", image);
	//cv::waitKey(100);
}

}  // namespace kalman
