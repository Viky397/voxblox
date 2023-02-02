// Author: Keenan Burnett
// Copyright (C) 2020 aUToronto
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef OBJECT_TRACKER_OBJECT_TRACKER_NODE_H
#define OBJECT_TRACKER_OBJECT_TRACKER_NODE_H
#include <Eigen/Geometry>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <zeus_msgs/BoundingBox3D.h>
#include <zeus_msgs/Detections3D.h>
#include <nav_msgs/Odometry.h>
#include <chrono>  // NOLINT [build/c++11]
#include <vector>
#include <string>
#include "utils/kalman.hpp"
#include "utils/transform_utils.hpp"
#include <cmath>
#include <math.h>
#include <boost/bind.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/math/distributions/uniform.hpp>

namespace object_detection {

typedef boost::shared_ptr<kalman::KalmanTracker> KalmanPtr;

//* KalmanTrackerNode
/**
* \brief This ROS node receives raw detections output in the "c2" frame (dets) and odomety messages (odom) and outputs
* filtered object tracks in the world frame "odom".
*
* A linear Kalman Filter is used to update an object track's position and velocity
* A HiddenMarkovModel is used to filter the object states temporally.
* We maintain a confidence level for each object track.
*/
class KalmanTrackerNode {
 public:
    KalmanTrackerNode(ros::NodeHandle& nh) : nh_(nh) {
    	det_pub_ = nh_.advertise<zeus_msgs::Detections3D>("/Object/Detections3D", 10);
    	sm_target_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/SM/Target", 10);
    	sm_source_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/SM/Source", 10);
    	sm_refined_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/SM/Refined", 10);
    	init();
    }

    void init();

    void setKalmanTracker(KalmanPtr kalmantracker_) {kalmantracker = kalmantracker_;}

    zeus_msgs::Detections3D track(const zeus_msgs::Detections3D &det,
    						const Eigen::Matrix4f& robot_pose=Eigen::Matrix4f::Identity(),
							bool prune_by_confidence = true);

    /*!
       \brief Incorporates new detections into the current list of object tracks.
       First, data association is computed between new detections and the existing object tracks.
       Existing objects are updated based on this association and unassociated detections become new objects.
       Object tracks that are too close to each other, outside the relevant region, or have gone unobserved for a long
       period are pruned.
       \param det zeus_msg representing a list of 3D detections (in the "c2" frame).
       \param odom The odometry (position and orientation) at the current time step.
       \pre The detection and odom topic should be synchronized.
    */
    void callback(const zeus_msgs::Detections3D::ConstPtr & det, const nav_msgs::OdometryConstPtr& odom) {
    	// auto ret = track(det, odom);
    }

    /*!
       \brief Initializes the static transforms so that they only need to be queried once.
    */
    void initialize_transforms();

    KalmanPtr& getTracker() {
    	return kalmantracker;
    }

    void updateObjectConfidence(std::map<int, double> measurements, double std);

    void updateObjectLUT() {
    	object_id_idx_lut_.clear();
    	auto& objects = kalmantracker->get_mutable_object_list();
    	for (size_t idx(0); idx<objects.size(); idx++) {
    		object_id_idx_lut_[objects[idx].ID] = idx;
    	}
    }

    void backupObjectModel(int idx) {
    	updateObjectLUT();
    	auto& objects = kalmantracker->get_mutable_object_list();
    	objects.at(object_id_idx_lut_.at(idx)).backupModel();
    }

    void restoreObjectModel(int idx) {
    	updateObjectLUT();
    	auto& objects = kalmantracker->get_mutable_object_list();
    	objects.at(object_id_idx_lut_.at(idx)).restoreModel();
    }

    double getObjectConf(int idx) {
    	updateObjectLUT();
    	auto& objects = kalmantracker->get_mutable_object_list();
    	return objects.at(object_id_idx_lut_.at(idx)).getConfidence();
    }

    std::vector<double> getObjectBetaParams(int idx) {
    	updateObjectLUT();
    	auto& objects = kalmantracker->get_mutable_object_list();
    	return objects.at(object_id_idx_lut_.at(idx)).getBetaParams();
    }

    int numObjects() {
    	return kalmantracker->get_mutable_object_list().size();
    }

 private:
    Eigen::Matrix4d Tic2 = Eigen::Matrix4d::Identity();     /*!< Transform from c2 to imu_link */
    zeus_tf::tfBufferPtr tfBuffer;
    zeus_tf::tfListenerPtr tfListener;
    ros::NodeHandle nh_;
    ros::Publisher det_pub_;
    KalmanPtr kalmantracker;                                /*!< Pointer to kalman::KalmanTracker object. */
    std::string world_frame_id = "map";
    ros::Publisher sm_target_pub_;
    ros::Publisher sm_source_pub_;
    ros::Publisher sm_refined_pub_;

    std::map<int, size_t> object_id_idx_lut_;
};

}

#endif  // OBJECT_TRACKER_OBJECT_TRACKER_NODE_H
