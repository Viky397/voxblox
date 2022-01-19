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
#pragma once
#include <Eigen/Geometry>
#include <deque>
#include <vector>
#include <iostream>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "utils/hmm.hpp"
#include "utils/zeus_pcl.hpp"
#include "types/Pose2.hpp"

//* Object
/**
* \brief This class is used to contain information about an object (that may be tracked).
*/
class Object {
 public:
	Object() {
		confidence = a / (a + b);
	}
	~Object() {}
    Eigen::MatrixXd x_hat = Eigen::MatrixXd::Zero(5, 1);    /*!< [x, y, z, xdot, ydot] 3D position and 2D velocity */
    Eigen::MatrixXd P_hat = Eigen::MatrixXd::Zero(5, 5);    /*!< Covariance of the state */
    Eigen::MatrixXd y_prev = Eigen::MatrixXd::Zero(3, 1);
    float w = 0, l = 0, h = 0, yaw = 0;                     /*!< Shape of the (3D) bounding box, yaw = orient about z */
    int type = 0;                                           /*!< Which state is the object most likely in */
    float confidence = 0.0;                                 /*!< Confidence level for the object: \f$\in [0, 1] \f$ */
    int ID = 0;                                             /*!< A unique identifier for each object */
    int filter_length = 5;                                  /*!< Only used for detecting flashing red lights. */
    std::deque<int> past_types;                             /*!< Only used for detecting flashing red lights. */
    HMMPtr hmm;                                             /*!< Pointer to HiddenMarkovModel object */
    int camera = 1;                                         /*!< Which camera was the object observed in. */
    bool is_observed = false;
    bool is_new = false;
    double last_observed_time = -1;
    double delta_t = 0.1;                                   /*!< current_time - last_observed_time */
    double first_observed_time = -1;
    double last_updated = -1;
    double a = 2.0;
    double b = 1.0;
    double mu = 0.0;
    double sig = 0.5;

    int life = 0;

    bool inlier = true;

    double eps = 1e-5;

    int change_sign = 1;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_obs = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    /*!
       \brief Returns the most likely object state (type)
    */
    int getType();

    void set_type(int type_);

    /*!
       \brief Return how long it has been since an object was last observed in seconds.
    */
    double getLostTime(double current_time);

    /*!
       \brief Returns how long it has been since an object was first observed.
    */
    double getAge(double current_time);

    std::vector<double> mergeNewCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pcl);

    void updateProbability(double change, double std_change);

    bool expectedToObserve(Pose2 cam_pose, float fov);

    Pose2 getPose2() {
    	return Pose2(x_hat(0, 0), x_hat(1, 0), 0);
    }

    /*
    void initPrior() {
    	if (type == 0) {
    		// low dynamic
    		a = 3.5;
    		b = 2;
    	} else if (type == 1) {
    		a = 2;
    		b = 2;
    	} else if (type == 2) {
    		// high dynamic
    		a = 2;
    		b = 3;
    	}
    	confidence = a / (a + b);
    }
    */

    void initPrior() {
    	a = 2;
    	b = 1;
    	confidence = a / (a + b);
    }

    double get2DArea() const {
    	return l * w;
    }

    double get3DVolume() const {
    	return l * w * h;
    }

    friend std::ostream &operator<<(std::ostream &output, const Object &O) {
        output << "x: " << O.x_hat(0, 0) << " y: " << O.x_hat(1, 0) << " z: " << O.x_hat(2, 0) << " vx: " <<
            O.x_hat(3, 0) << " vy: " << O.x_hat(4, 0) << " ID: " << O.ID << " type: " << O.type << " conf: " <<
            O.confidence;
        return output;
    }

private:
    std::pair<double, double> K(double k) const;
};
