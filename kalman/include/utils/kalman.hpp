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
#include <zeus_msgs/BoundingBox3D.h>
#include <vector>
#include <map>
#include <iostream>
#include "types/objects3D.hpp"
#include "types/Pose2.hpp"

namespace kalman {

//* KalmanTracker
/**
* \brief This class enables tracking and maintaining a list of 2D or 3D objects.
*
* A linear Kalman Filter is used to update an object track's position and velocity
* given an associated detection. This class also has methods for data association
* between tracks and detections as well as pruning duplicate tracks.
*
* A HiddenMarkovModel is used to filter the object states temporally.
*
* Note that we maintain a confidence level for each object track. This confidence is based primarily on the confidence
* output by the detector (possibly a DNN) and is smoothed temporally. The confidence decays exponentially when there are
* no observations. Object tracks should be ignored by the planner below a certain threshold. Objects that are unobserved
* for a long period (delete_time) are deleted permanently.
*
* Matrix Toc is the transformation from the sensor frame to the static world frame (odom). We use this to track objects
* in a static world frame. Setting this matrix to identity enables tracking within the sensor frame.
*/
class KalmanTracker {
 public:
   //: A(A_), C(C_), Q(Q_), R(R_), P0(P0_)
   KalmanTracker(Eigen::MatrixXd A_, Eigen::MatrixXd C_, std::map<std::string, Eigen::MatrixXd> Q_, std::map<std::string, Eigen::MatrixXd> R_,
   Eigen::MatrixXd P0_) {
	   A = A_;
	   C = C_;
	   P0 = P0_;
       if (Q_.size() == 1){
           is_object_tracker = false;
           Q_all = Q_["all"];
           R_all = R_["all"];
       } else{
           is_object_tracker = true;
           Q_all = Q_["all"];
           R_all = R_["all"];
           Q_dynamic = Q_["dynamic"];
           R_dynamic = R_["dynamic"];
       }
       xdim = A.cols();
       ydim = C.rows();
   }
   // Setters
   void setCurrentTime(double current_time_) {current_time = current_time_;}
   void set1DKalmanGains(std::vector<double> one_d_kalman_gains_) {one_d_kalman_gains = one_d_kalman_gains_;}
   void setPointCloudRange(std::vector<double> point_cloud_range_) {point_cloud_range = point_cloud_range_;}
   void setConfidenceDrop(double confidence_drop_) {confidence_drop = confidence_drop_;}
   void setMetricGate(double metricGate_) {metricGate = metricGate_;}
   void setIOUThes(double iou_thres_) {iou_thres = iou_thres_;}
   void setMetricThres(double metric_thres_) {metric_thres = metric_thres_;}
   void setDeleteTime(double delete_time_) {delete_time = delete_time_; delete_time_unknown = delete_time; }
   void setCostParameters(double alpha, double beta) {cost_alpha = alpha; cost_beta = beta;}
   void setFilterLength(int filter_length_) {filter_length = filter_length_;}
   void setPedestrianType(int pedestrian_type_) {pedestrian_type = pedestrian_type_;}
   void setUnknownType(int unknown_type_) {unknown_type = unknown_type_;}
   void setUnknownDynamicType(int unknown_dynamic_type_) {unknown_dynamic_type = unknown_dynamic_type_;}

   void setDynamicObjConf(float dynamic_obj_conf_) {dynamic_obj_conf = dynamic_obj_conf_;}
   void setStaticObjConf(float static_obj_conf_) {static_obj_conf = static_obj_conf_;}
   void setDeerDims(std::vector<double> deer_L_, std::vector<double> deer_W_, std::vector<double> deer_H_) {
       deer_L = deer_L_;
       deer_W = deer_W_;
       deer_H = deer_H_;
   }
   void setBarrelDims(std::vector<double> barrel_L_, std::vector<double> barrel_W_, std::vector<double> barrel_H_) {
       barrel_L = barrel_L_;
       barrel_W = barrel_W_;
       barrel_H = barrel_H_;
   }
   void setPedDims(std::vector<double> ped_L_, std::vector<double> ped_W_, std::vector<double> ped_H_) {
       ped_L = ped_L_;
       ped_W = ped_W_;
       ped_H = ped_H_;
   }
   void setHMMParameters(std::vector<int> types_, Eigen::MatrixXd A_, Eigen::MatrixXd C_, Eigen::MatrixXd pi_) {
       types = types_;
       num_types = types.size();
       A_hmm = A_;
       C_hmm = C_;
       pi_hmm = pi_;
   }

   /*!
      \brief This method performs data assocation between new detections and the current object list.

      Data association is performed by generating a cost matrix based on the Euclidean distances between
      detections and object tracks. Associations that result in a distance greater than metricGate are
      considered infeasible. The data association is performed in an optimal manner.

      For each unassociated detection, we add a new object to the object list (X).

      \param dets The raw detections output in the sensor frame
      \param Toc Transformation from the sensor frame to the static world frame (odom). We use this to track objects
        in a static world frame. Setting this matrix to identity enables tracking within the sensor frame.
   */
   void association(const std::vector<zeus_msgs::BoundingBox3D>& dets, Eigen::Matrix4f robot_pose);

   /*!
      \brief This method performs the kalman filtering update step, incorporating new measurements.

      Objects without a detection are propagated using the motion model (A).

      The w, l, h, yaw, and confidence are also updated using 1D Kalman filters.

      Object confidences decay exponentially while undetected.

      \param dets The raw detections output in the sensor frame
      \param Toc Transformation from the sensor frame to the static world frame (odom). We use this to track objects
        in a static world frame. Setting this matrix to identity enables tracking within the sensor frame.
   */
   std::vector<pcl::PointCloud<pcl::PointXYZRGB> > filter(std::vector<zeus_msgs::BoundingBox3D> &dets, Eigen::Matrix4f robot_pose);

   /*!
      \brief This method prunes object tracks that are within metricGate of each other. Higher confidence track is kept.

      In addition, This method prunes objects that exit the rectangular prism defined by point_cloud_range.

      \param Tco Transformation from the static world frame to the sensor frame.
   */
   void prune(Eigen::Matrix4f robot_pose, bool prune_by_confidence);

   /*!
      \brief This method prunes object tracks that overlap when projected onto the image plane.

      The iou_thres parameter is used to determine if a track will be pruned. Higher confidence track is kept.

      \param Tco Transformation from the static world frame to the sensor frame.
      \param CAM Camera projection matrix. Ex: [fx, 0, cx, 0; 0, fy, cy, 0; 0, 0, 1, 0; 0, 0, 0, 1]
   */
   void prune_2d_overlap(Eigen::Matrix4d Tco, Eigen::MatrixXd CAM);

   /*!
      \brief Retrieve the current vector of objects being tracked.
   */
   const std::vector<Object>& get_object_list();

   std::vector<Object>& get_mutable_object_list();

   /*!
      \brief This method prints the current list of objects being tracked.
   */
   void print();

   /*!
      \brief This method checks whether the desired ID is in the current object list.
      \param ID The ID of the object that we are looking for.
   */
   bool checkIfIDPresent(int ID);

   /*!
      \brief This method forces a new object to be added to the object list
      \param det This detection will be turned into a new object track
      \param ID This is the ID that will be used for this new object track.
      \param current_time Current time in ROS, required for initializing an object track.
   */
   void add_new(const zeus_msgs::BoundingBox3D &det, int ID, double current_time);

   /*!
      \brief Remove the objects that are not contained in the provided ID list.
      \param keepIDs The list of object IDs that should be kept.
   */
   void removeObjectsNotInList(std::vector<int> keepIDs);

   /*!
      \brief Set the indices vector, internal to the KalmanTracker class, based on the given map of object IDs to dets.
      \param associations A map from object IDs to their associated detection index.
   */
   void setIndices(std::map<int64_t, int> associations);

   /*!
      \brief Update the HMM state of each object based on the detections provided.
      \param dets The raw detections output in the sensor frame
      \param current_time Current time in ROS, required for initializing an object track.
      \pre Either setIndices() or association() should be run before this.
   */
   void pushTypes(std::vector<zeus_msgs::BoundingBox3D> dets, double current_time);

   /*!
      \brief Prune old object tracks based on the current time.

      For each object track, we check the time that it was last observed and delete it if delete_time has been exceeded
      since the last detection.
      \param current_time Current time in ROS, required for initializing an object track.
   */
   void pruneOld(double current_time);

 private:
    int xdim = 5;       /*!< size of object state vector (x, y, z, xdot, ydot) */
    int ydim = 3;       /*!< size of measurement vector (x, y, z) */
    Eigen::MatrixXd A = Eigen::MatrixXd::Identity(xdim, xdim);      /*!< Motion model */
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(ydim, xdim);          /*!< Observation model */

    Eigen::MatrixXd Q_all = Eigen::MatrixXd::Identity(xdim, xdim);      /*!< Process Noise */
    Eigen::MatrixXd R_all = Eigen::MatrixXd::Identity(ydim, ydim);      /*!< Measurement Noise */
    Eigen::MatrixXd Q_dynamic = Eigen::MatrixXd::Identity(xdim, xdim);      /*!< Process Noise */
    Eigen::MatrixXd R_dynamic = Eigen::MatrixXd::Identity(ydim, ydim);      /*!< Measurement Noise */

    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(xdim, xdim);     /*!< Initial Covariance */
    std::vector<Object> X;      /*!< The list of objects being tracked */
    std::vector<int> indices;   /*!< if indices[i] >= 0, indices[i] is the detection associated with object i */
    bool is_object_tracker = false;
    int objectID = 0;           /*!< A unique ID counter for each object */
    double current_time = 0;
    std::vector<double> one_d_kalman_gains = {0.5, 0.5, 0.5, 0.5, 0.5}; /*!< Kw, Kl, Kh, Kyaw, Kconfidence */
    std::vector<double> point_cloud_range = {1.1, 40.0, -14.5, 14.5, -1.8, 3.5};
    /*!< xmin, xmax, ymin, ymax, zmin, zmax */
    double confidence_drop = 0.12;
    double metricGate = 1.2;            /*!< Euclidean distance gate for data association */
    double iou_thres = 0.75;             /*!< 2D intersection over union distance threshold for pruning */
    double metric_thres = 0.5;          /*!< Euclidean distance threshold for pruning */
    float delete_time = 0.5;            /*!< If an object is unobserved for this long, it will be deleted */
    float delete_time_unknown = 0.5;

    float dynamic_obj_conf = 0.75;
    float static_obj_conf = 0.9;

    int pedestrian_type = -1;           /*!< Classes with a pedestrian type are given this class type */
    int unknown_type = -1;              /*!< Classes with an unknown type are given this class type */
    int unknown_dynamic_type = -1;      /*!< Classes with an unknown dynamic type are given this class type */
    double cost_alpha = 0.5, cost_beta = 0.5;   /*!< d = d_euclid + alpha * d_size + beta * d_confidence */
    int filter_length = 5;              /*!< This parameter is currently only used for flashing red light detection */
    // HiddenMarkovModel Parameters
    int num_types = 2;                  /*!< The number of states each object can occupy ex: (pedestrian | unknown) */
    std::vector<int> types = {1, 3};    /*!< The vector of types that objects may occupy */
    Eigen::MatrixXd A_hmm = Eigen::MatrixXd::Zero(num_types, num_types);   /*!< Transition Matrix (HMM) */
    Eigen::MatrixXd C_hmm = Eigen::MatrixXd::Zero(num_types, num_types);   /*!< Observation Matrix (HMM) */
    Eigen::MatrixXd pi_hmm = Eigen::MatrixXd::Zero(num_types, 1);          /*!< Initial State Prior (HMM) */

    // Deer (dynamic object), barrel and pedestrian bounding box dimensions [lower limit, upper limit]
    std::vector<double> deer_L;
    std::vector<double> deer_W;
    std::vector<double> deer_H;
    std::vector<double> barrel_L;
    std::vector<double> barrel_W;
    std::vector<double> barrel_H;
    std::vector<double> ped_L;
    std::vector<double> ped_W;
    std::vector<double> ped_H;

    /*!
       \brief This method creates a new object based on the given detection.
       \param det this is the detection that will be turned into a new object
       \param objectID The new object will possess this ID
       \param current_time The new object will be initialized with this as the created time and last observed time.
       \pre Before being added to the object list, the detection should be transformed into the desired tracking frame.
       \return The new object that has been created
    */
    Object create_new(const zeus_msgs::BoundingBox3D &det, int &objectID, double current_time);

    /*!
       \brief This method generates a 2D cost matrix to be used in a data association algorithm.
       \param dets These are the detections that are candidates for being assigned to existing object tracks.
       \param dets_indices This vector represents the subset of all the detections in dets that are going to be assigned
       \param object_indices This vector represents the subset of the object list that we will consider
       assigning the detections to.
       \param infeasible_cost This parameter will be set to the cost of an infeasible association by this method.
       \pre The dets should be in the same frame as the object list.
       \post Infeasible associations will be denoted by infeasible_cost. If the data association function output an
       association with an infeasible cost, it should be discarded.
       \return This method returns the generated 2D cost matrix. Size: (object_indices.size(), dets_indices.size())
    */
    Eigen::MatrixXd generateCostMatrix(const std::vector<zeus_msgs::BoundingBox3D> &dets,
        std::vector<int> dets_indices, std::vector<int> object_indices, double &infeasible_cost);

    Eigen::MatrixXd generateCostMatrixSM(const std::vector<zeus_msgs::BoundingBox3D> &dets,
            std::vector<int> dets_indices, std::vector<int> object_indices, double &infeasible_cost);

    /*!
       \brief This method first clusters objects and detections together based on the maximum association distance
       defined by metricGate. Then, it generates a cost matrix for each cluster and uses the optimal association
       methods defined in the association library.
       \param dets This vector represents all the detections received at this time frame.
       \param indices This method will write to this parameter so that indices[i] < 0 for objects without a detection
       and indices[i] >= 0 corresponds to the associated detection to object i.
       \param notassoc This vector will contain the detections which were not associated to any objects.
       \param current_time This value represents the current time in ROS.
       \pre dets should have been transformed into the same frame as the object list before this.
    */
    void optimalAssociation(const std::vector<zeus_msgs::BoundingBox3D> &dets, std::vector<int>& indices,
        std::vector<int>& notassoc, double current_time);

    void optimalAssociationSM(const std::vector<zeus_msgs::BoundingBox3D> &dets, std::vector<int>& indices,
            std::vector<int>& notassoc, double current_time);

    bool boxSizeSimilarToPed(const Object& obj);
    bool boxSizeSimilarToPed(const zeus_msgs::BoundingBox3D& obj);

    Eigen::Matrix4f matchPCDs(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr source, const Object& target, int max_iter=10);

    float norm(Eigen::Matrix4f p) {
    	return sqrt(pow(p(0,3), 2) + pow(p(1,3), 2) + pow(p(2,3), 2));
    }
};

}  // namespace kalman
