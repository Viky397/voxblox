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
#include "utils/hmm.hpp"

//* Object
/**
* \brief This class is used to contain information about an object (that may be tracked).
*/
class Object {
 public:
    ~Object() {}
    Eigen::MatrixXd x_hat = Eigen::MatrixXd::Zero(5, 1);    /*!< [x, y, z, xdot, ydot] 3D position and 2D velocity */
    Eigen::MatrixXd P_hat = Eigen::MatrixXd::Zero(5, 5);    /*!< Covariance of the state */
    float w = 0, l = 0, h = 0, yaw = 0;                     /*!< Shape of the (3D) bounding box, yaw = orient about z */
    int type = 0;                                           /*!< Which state is the object most likely in */
    float confidence = 1.0;                                 /*!< Confidence level for the object: \f$\in [0, 1] \f$ */
    int ID = 0;                                             /*!< A unique identifier for each object */
    int filter_length = 5;                                  /*!< Only used for detecting flashing red lights. */
    std::deque<int> past_types;                             /*!< Only used for detecting flashing red lights. */
    HMMPtr hmm;                                             /*!< Pointer to HiddenMarkovModel object */
    int camera = 1;                                         /*!< Which camera was the object observed in. */
    double last_observed_time;
    double delta_t = 0.1;                                   /*!< current_time - last_observed_time */
    double first_observed_time;
    double last_updated;

    /*!
       \brief Returns the most likely object state (type)
    */
    int getType();

    /*!
       \brief "Pushes" the latest detection and class confidences onto the HiddenMarkovModel.
       \param t The most likely object state, according to the latest detection.
       \param class_confidences Either: a 1-dim vector with the confidence level for the detection
            OR: a n-dim vector n = number of states, confidence-level for each class, sum(class_confidences) = 1.
    */
    void push_type(int t, std::vector<float> class_confidences);

    /*!
       \brief Return how long it has been since an object was last observed in seconds.
    */
    double getLostTime(double current_time);

    /*!
       \brief Returns how long it has been since an object was first observed.
    */
    double getAge(double current_time);

    /*!
       \brief Specifically for flashing red light detection. This function checks to see if the ratio of detections
       of type is between ratio lower and ratio upper. Ex: A RED detection occuring between 0.35 and 0.65 of the time
       may be flashing. The ratio is determined by keeping a history of the previous filter_length detections.
       \param type The type that we would like to check whether it is flashing.
       \param lower The lower-bound of a ratio of ON times / OFF times.
       \param upper The upper-bound of a ratio of ON times / OFF times.
       \return true/false is the state flashing?
    */
    bool checkFlashing(int type, float lower, float upper);

    friend std::ostream &operator<<(std::ostream &output, const Object &O) {
        output << "x: " << O.x_hat(0, 0) << " y: " << O.x_hat(1, 0) << " z: " << O.x_hat(2, 0) << " vx: " <<
            O.x_hat(3, 0) << " vy: " << O.x_hat(4, 0) << " ID: " << O.ID << " type: " << O.type << " conf: " <<
            O.confidence;
        return output;
    }
};
