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
#include <vector>
#include <deque>
#include <boost/shared_ptr.hpp>

namespace kalman {

//* HiddenMarkovModel
/**
* \brief This class is used for filtering multinomial random variables.
*
* This class can and is used to calculate the most likely type (state) for object detection, traffic signs, and
* traffic lights.
*/
class HiddenMarkovModel {
 public:
    HiddenMarkovModel(Eigen::MatrixXd A_, Eigen::MatrixXd C_, Eigen::MatrixXd pi_, std::vector<int> types_) :
        A(A_), C(C_), pi(pi_), types(types_) {num_types = (int)types.size();}

    /*!
       \brief This function updates the internal (latent) state of the HMM based on the transition probabibility matrix
            A, observation matrix C and the inputs (det_type, class_confidences).
       \param det_type The detected type with the highest confidence.
       \param class_confidences Can be used to provide the HMM with a distribution of confidences over the possible
            states. Otherwise, it must be a vector with the confidence of the most likely detection.
    */
    void filter(int det_type, std::vector<float> class_confidences);

    /*!
       \brief Returns the most likely state of the object at the current time step.
    */
    int get_type();

    /*!
       \brief Returns the log likelihood of data given the latent states and the parameters (A, C).
       Value is averaged over the averaging_period.
    */
    double get_log_likelihood();

 private:
    int num_types = 2;                                                /*!< The total number of possible types. */
    Eigen::MatrixXd alpha = Eigen::MatrixXd::Zero(num_types, 1);      /*!< p(z_t | x_{1:t}) */
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(num_types, num_types);  /*!< transition matrix p(z_t = i | z_{t-1} = j) */
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(num_types, num_types);  /*!< observation matrix p(x_t = l | z_t = k) */
    Eigen::MatrixXd pi = Eigen::MatrixXd::Zero(num_types, 1);         /*!< initial p(z_0) guess */
    std::vector<int> types;                 /*!< A vector of detector types used for conversion.*/
    uint averaging_period = 100;            /*!< Period over which log likelihood is averaged. */
    std::deque<double> log_likelihoods;     /*!< A queue containing the past history of log likelihoods */
    bool init = false;                      /*!< Whether or not the filter has been initialized */
};
}  // namespace kalman

typedef boost::shared_ptr<kalman::HiddenMarkovModel> HMMPtr;
