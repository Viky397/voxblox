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

/*!
   \file transform_utils.hpp
   \brief This file contains several helper functions for converting between ROS transforms and 4x4 transformations.
*/
#pragma once
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Geometry>
#include <string>
#include <vector>

namespace zeus_tf {

typedef tf2_ros::Buffer tfBuffer;
typedef boost::shared_ptr<tf2_ros::Buffer> tfBufferPtr;
typedef tf2_ros::TransformListener tfListener;
typedef boost::shared_ptr<tf2_ros::TransformListener> tfListenerPtr;

/*!
   \brief Returns the inverse of a 4x4 transformation matrix.
   Where T = [R, t; 0 0 0 1], inv(T) = [R.transpose, -R.transpose() * T; 0 0 0 1];
   \param T The input 4x4 transformation matrix.
   \return returns the inverse of the transformation matrix T.
*/
Eigen::Matrix4d get_inverse_tf(Eigen::Matrix4d T);

/*!
   \brief Extracts the roll-pitch-yaw parameters from an odometry message.
   \param odom An odometry message (potentially from a GPS/IMU sensor)
   \param rpy [out] This parameter will be set to a vector of the [roll, pitch, yaw] values corresponding to odom.
*/
void get_rpy_from_odom(nav_msgs::Odometry odom, std::vector<double> &rpy);

/*!
   \brief Retrives a 4x4 transformation matrix from (imu_link) to (odom) given a ROS odometry message.
   \param odom An odometry message (potentially from a GPS/IMU sensor)
   \param Toi [out] This parameter will be set to the 4x4 transform corresponding to odom.
*/
void get_odom_tf(nav_msgs::Odometry odom, Eigen::Matrix4d & Toi);

void get_odom_tf(nav_msgs::Odometry odom, double &init_x, double &init_y, double &init_z,
    bool &init_pos_set, bool zero_odom, Eigen::Matrix4d & Toi);

/*!
   \brief Extract a 4 x 4 transformation matrix from the ROS TF tree from src_frame to tgt_frame.
   \param tfBuffer Required buffer object for reading from the TF tree.
   \param src_frame starting frame ID.
   \param tgt_frame ending frame ID.
   \param T [out] The 4x4 transform will be written to this parameter.
   \post This function will wait forever for the transform to be available
*/
void get_transform(tfBufferPtr tfBuffer, std::string src_frame, std::string tgt_frame, Eigen::Matrix4d& T);

}  // namespace zeus_tf
