// Author: Keenan Burnett
#include "object_tracker/object_tracker_node.h"
#include <vector>

void convertToRosMessage(std::vector<Object> object_list, zeus_msgs::Detections3D &outputDetections, float yaw);

void KalmanTrackerNode::callback(const zeus_msgs::Detections3D::ConstPtr & det,
    const nav_msgs::OdometryConstPtr& odom) {
	std::cout << "Raw det received " << det->bbs.size() << std::endl;
    auto start = std::chrono::high_resolution_clock::now();
    std::vector<zeus_msgs::BoundingBox3D> dets = det->bbs;
    //Eigen::Matrix4d Toi = Eigen::Matrix4d::Identity();
    //zeus_tf::get_odom_tf(*odom, Toi);
    //Eigen::Matrix4d Toc = Toi * Tic2;
    kalmantracker->setCurrentTime(det->header.stamp.toSec());
    //! Perform data association between the new detections (dets) and the existing object tracks
    kalmantracker->association(dets, Eigen::Matrix4d::Identity());
    //! Linear Kalman filter update
    kalmantracker->filter(dets, Eigen::Matrix4d::Identity());
    //! Prune objects that are closer than metricGate to each other or objects outside point_cloud_range.
    kalmantracker->prune(zeus_tf::get_inverse_tf(Eigen::Matrix4d::Identity()));
    //! Publish ROS message:
    zeus_msgs::Detections3D outputDetections;
    outputDetections.header.stamp = det->header.stamp;
    outputDetections.header.frame_id = world_frame_id;
    outputDetections.bbs.clear();
    std::vector<Object> object_list = kalmantracker->get_object_list();
    std::vector<double> rpy;
    zeus_tf::get_rpy_from_odom(*odom, rpy);
    convertToRosMessage(object_list, outputDetections, 0);
    det_pub_.publish(outputDetections);
    auto stop = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = stop - start;
    ROS_DEBUG_STREAM("[OBJ] KALMAN TRACKER TIME: " << elapsed.count());
}

void KalmanTrackerNode::initialize_transforms() {
    tfBuffer = zeus_tf::tfBufferPtr(new zeus_tf::tfBuffer());
    tfListener = zeus_tf::tfListenerPtr(new zeus_tf::tfListener(*tfBuffer));
    zeus_tf::get_transform(tfBuffer, "c2", "imu_link", Tic2);
    ROS_INFO_STREAM("[OBJ] object_tracker transforms initialized");
}

// Given an object list, formats the appropriate ROS output message for object detection
void convertToRosMessage(std::vector<Object> object_list, zeus_msgs::Detections3D &outputDetections, float yaw) {
    for (int i = 0; i < (int)object_list.size(); i++) {
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
