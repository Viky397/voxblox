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
   \file association.hpp
   \brief This file contains functions for performing data association on a cost matrix.
*/
#pragma once
#include <Eigen/Geometry>
#include <vector>

namespace kalman {

/*!
   \brief This function performs the optimal Hungarian algorithm for data association O(n^3).
   Based on Munkres (1957) and Bourgeois/Lassalle (1971).
   Returns the assignments of the column indices to the row indices
   size(assignments) == C.rows()
   \param C cost matrix (rows = objects, columns = detections) C(i, j) = cost of associating detection j to object i.
   \param infeasible_cost The cost of an infeasible assignment. (set to INF all assignments feasible).
   \pre Before using this function, be sure to remove "outliers": detections that won't match to any objects.
   In addition, remove the objects that don't have any detections within their "gate".
   Pre-processing cost matrix (C): set the infeasible assignments costs to the largest feasible assignment cost + 1.
   \return A vector of indices corresponding the assignment of detections (columns) to the objects (rows)
        If an object is not assigned a detection, the corresponding index will be -1.
*/
std::vector<int> hungarian(Eigen::MatrixXd C, double infeasible_cost);

/*!
   \brief This function performs brute force data association.
   O(n!) but optimal. For small n (<= 6), brute force can compete with hungarian for speed.
   \param C cost matrix (rows = objects, columns = detections) C(i, j) = cost of associating detection j to object i.
   \param infeasible_cost The cost of an infeasible assignment. (set to INF all assignments feasible).
   \pre Pre-processing cost matrix (C): set the infeasible assignments costs to the largest feasible assignment cost + 1
   \return A vector of indices corresponding the assignment of detections (columns) to the objects (rows)
        If an object is not assigned a detection, the corresponding index will be -1.
*/
std::vector<int> brute_force(Eigen::MatrixXd C, double infeasible_cost);

/*!
   \brief Greedy data association. O(n) but suboptimal.
   \param C cost matrix (rows = objects, columns = detections) C(i, j) = cost of associating detection j to object i.
   \param infeasible_cost The cost of an infeasible assignment. (set to INF all assignments feasible).
   \pre Pre-processing cost matrix (C): set the infeasible assignments costs to the largest feasible assignment cost + 1
   \return A vector of indices corresponding the assignment of detections (columns) to the objects (rows)
        If an object is not assigned a detection, the corresponding index will be -1.
*/
std::vector<int> greedy(Eigen::MatrixXd C, double infeasible_cost);

/*!
    \brief This function performs data association based on a cost function.
    \param C cost matrix (rows = objects, columns = detections) C(i, j) = cost of associating detection j to object i.
    \param type 0 --> optimal, 1 --> greedy
    \param infeasible_cost The cost of an infeasible assignment. (set to INF all assignments feasible).
    \pre Before using this function, be sure to remove "outliers": detections that won't match to any objects.
    In addition, remove the objects that don't have any detections within their "gate".
    Pre-processing cost matrix (C): set the infeasible assignments costs to the largest feasible assignment cost + 1.
    \return A vector of indices corresponding the assignment of detections (columns) to the objects (rows)
         If an object is not assigned a detection, the corresponding index will be -1.
*/
std::vector<int> association(Eigen::MatrixXd C, int type, double infeasible_cost);

}  // namespace kalman
