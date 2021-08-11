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
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Geometry>
#include <zeus_msgs/BoundingBox2D.h>
#include <limits>
#include <string>
#include <vector>
#include <map>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include "types/zeus_pcl_types.hpp"

namespace zeus_pcl {

struct greater_than_bb {
  inline bool operator() (const zeus_msgs::BoundingBox2D& bb1, const zeus_msgs::BoundingBox2D& bb2){
    return ((bb1.w * bb1.h) > (bb2.w * bb2.h));
  }
};

void fromPCLRGB(const pcl::PointCloud<pcl::PointXYZRGB>& pclIn, PointCloudPtr cloudOut);

/*!
   \brief Convert a PointCloud2 ROS message into our custom pointcloud format. (x, y, z)
*/
void fromROSMsg(const sensor_msgs::PointCloud2ConstPtr& ros_msg, PointCloudPtr cloudOut);

/*!
   \brief Convert a PointCloud2 ROS message into our custom pointcloud format. (x, y, z, i)
*/
void fromROSMsg(const sensor_msgs::PointCloud2ConstPtr& ros_msg, IPointCloudPtr cloudOut);

/*!
   \brief Convert from our custom pointcloud format into a PointCloud2 ROS message.
   \post Remember to set msg.header.stamp, msg.header.frame_id, and msg.fields after this.
*/
void toROSMsg(PointCloudPtr pc, sensor_msgs::PointCloud2& msg, std::string frame="");

/*!
   \brief Sort bounding boxes in descending order of size (size = width * height).
*/
void sort_bbs_by_size(std::vector<zeus_msgs::BoundingBox2D> &bbs);

/*!
   \brief Discard points outside the rectangular prism defined by min_x, max_x, min_y, max_y, min_z, max_z.
   \param cloudIn input PointCloud
   \param cloudOut output PointCloud
*/
void passthrough(PointCloudPtr cloudIn, PointCloudPtr cloudOut, float min_x,
    float max_x, float min_y, float max_y, float min_z, float max_z);

/*!
   \brief Discard points outside the rectangular, perform the operation on the input cloud.
*/
void passthrough(PointCloudPtr cloud, float min_x, float max_x, float min_y,
    float max_y, float min_z, float max_z);

/*!
   \brief Discard points outside a rectangular area defined by min_x, max_x, min_y, max_y.
   \param cloudIn input PointCloud
   \param cloudOut output PointCloud
   \param indices The indexes of the points that are kept from cloudIn are writtent to this vector.
*/
void passthrough(PointCloudPtr cloudIn, PointCloudPtr cloudOut, float min_x,
    float max_x, float min_y, float max_y, std::vector<int> &indices);

/*!
   \brief Randomly downsample a pointcloud
   \param pc The input pointcloud that will be downsampled.
   \param prob_keep The probabibility of keeping any particular point.  \f$ p_{keep} \in [0, 1] \f$.
*/
void randomDownSample(PointCloudPtr pc, float prob_keep);

void radialDownSample(PointCloudPtr pc);

/*!
   \brief Extract points from a pointcloud givena list of indices.
   \param cloudIn The input pointcloud which will not be modified.
   \param cloudOut After removing points, this pointer points to the output cloud.
   \param indices This vector contains the indices that we want to KEEP.
*/
void extract_points(PointCloudPtr cloudIn, PointCloudPtr cloudOut, std::vector<int> &indices);

/*!
   \brief Extract points from a pointcloud givena list of indices.
   \param cloud The input pointcloud is modified so that it only contains the points indicated by the indices vector.
   \param indices This vector contains the indices that we want to KEEP.
*/
void extract_points(PointCloudPtr cloud, std::vector<int> &indices);

/*!
   \brief Remove points from a pointcloud givena list of indices.
   \param cloud The input pointcloud is modified so that it does NOT contain the specified indices.
   \param indices This vector contains the indices that we want to GET RID OF.
*/
void extract_negative(PointCloudPtr cloud, std::vector<int> &indices);

/*!
   \brief Removes the instances of input that appear in so_far.
   \param input vector of indices to be modified.
   \param so_far vector of indices that are to be removed from input.
*/
void get_subset_unclustered(std::vector<int> &input, std::vector<int> &so_far);

/*!
   \brief Calculate the centroid and bounding box (cuboid) for the given pointcloud.
   \param pc The input pointcloud
   \param cuboid_parameters [xmin, ymin, zmin, xmax, ymax, zmax] this function outputs a bounding box that contains pc
   \return euclidean centroid of the points contained in pc.
*/
Eigen::Vector4d get_centroid(PointCloudPtr pc, std::vector<float> &cuboid_parameters);

/*!
   \brief Calculate the centroid and bounding box (cuboid) for the given pointcloud.
   \param pc The input pointcloud
   \param indices The list of indices wrt to the input pointcloud about which we will calculate the centroid.
   \return euclidean centroid of the points contained in pc[indices].
*/
Eigen::Vector4d get_centroid(PointCloudPtr pc, std::vector<int> &indices);


void to_2d(PointCloudPtr cloudIn, PointCloudPtr cloudOut);

/*!
   \brief Remove the points from pc that correspond to the ground.

   This function uses RANSAC plane fitting to estimate the ground plane for the current time step.
   A linear Kalman filter is used to smooth the ground plane parameters (a,b,c,d) temporally.
   Linear Least Squares is NOT used to obtain a slightly more accurate plane model for speed (similar to PCL).

   \param groundPrior This pointcloud is a subset of pc, specifying approximately where we expect the ground to be.
   \param pc The input pointcloud that will be modified.
   \param gp_params This vector keeps track of the current time filtered plane parameters (a,b,c,d).
   \param K This matrix represents the kalman gains for the ground plane parameters (each parameter is independent).
   \param dist_thres How close does a point need to be to the plane to be considered a ground point, in metric.
   \param max_iters The maximum number of iterations that RANSAC will run for.
*/
void removeGroundPlane(PointCloudPtr groundPrior, PointCloudPtr pc, Eigen::Vector4f &gp_params,
    Eigen::Matrix4f &K, float dist_thres, unsigned int max_iters, bool ret_gp=false, PointCloudPtr gp=nullptr);

/*!
   \brief Remove the points from pc that correspond to the ground.

   This function uses the (Himmelsbach, 2010) algorithm for ground plane extraction.
   This algo is not quite as fast as RANSAC but can be much more fine-grained in the pointcloud filtering.

   \param pc The input pointcloud that will be modified.
   \param alpha Number of segments = (2 * M_PI) / alpha
   \param tolerance Metric distance from the ground plane model to be considered a ground point
   \param Tm Slopes greater than this value will be considered non-ground.
   \param Tm_small Slopes less than this value will be checked for being a plateau
   \param Tb Flat regions that are higher than this value will be considered non-ground
   \param Trmse If the RSME of the line fit exceeds this value, it will be rejected
   \param Tdprev Maximum allowed distance between previous line and start of new line
*/
std::vector<std::pair<std::vector<float>, std::vector<float> > > removeGroundPlane2(PointCloudPtr pc, float alpha, float tolerance, float Tm, float Tm_small,
    float Tb, float Trmse, float Tdprev, bool ret_gp=false, PointCloudPtr gp=nullptr);

/*!
   \brief Creates a copy of the input pointcloud.
   \param cloudIn [in] input pointcloud to be copied.
   \param cloudOut [out] will point to a copy of the input cloud.
*/
void copyCloud(PointCloudPtr cloudIn, PointCloudPtr cloudOut);

/*!
   \brief Performs Euclidean clustering on the input pointcloud and outputs a vector of pointcloud labels.
   \param pc The input pointcloud.
   \param min_samples The minimum number of points required for a valid cluster.
   \param tolerance If the distance between two points is less than this value, they will be clustered together.
   \param labels [out] A vector of clusters. Clusters are vectors of point indices from the input cloud pc.
*/
void cluster_point_cloud(PointCloudPtr pc, int min_samples, float tolerance, std::vector<std::vector<int>> &labels);

/*!
   \brief This function filters a set of clusters and returns the best one if it passes several criteria.
   When projecting LIDAR clusters onto the image, we want to pick the BEST cluster.
   Experimentally, we have found that keeping the CLOSEST cluster usually retrieves the desired cluster.
   The depth to the cluster and the bbox width and height (in pixels) are used to infer the metric width and height.
   Clusters that are outside min_height and max_height are rejected.
   \pre Input points (bb_pts) need to be in the lidar frame (x forwads, y to the left, z up)
   \return the index of the best cluster, or -1 of none of clusters were valid.
*/
int filter_clusters(std::vector<std::vector<int>> &clusters, PointCloudPtr bb_pts, zeus_msgs::BoundingBox2D bbox,
    Eigen::Matrix4f CAM, Eigen::Vector4d &centroid_out, float min_height, float max_height);

/*!
   \brief Performs a camera projection of the input pointcloud (cloud), stores output in (uv).
   \param cloud input pointcloud, will not be modified.
   \param uv For each point in cloud (x), uv = P * x / (x(2))
   \param P The 4x4 camera projection matrix.
   \post uv(0) = u, uv(1) = v, uv(2) = depth, uv(3) = 1
*/
void project_points(PointCloudPtr cloud, PointCloudPtr uv, Eigen::Matrix4f P);

/*!
   \brief Transforms an input pointcloud inplace, using a 4x4 transformation matrix.
   \param cloud The input pointcloud that will be transformed.
   \param T The 4x4 transformation matrix.
*/
void transform_cloud(PointCloudPtr cloud, Eigen::Matrix4d T);

struct Node {
    explicit Node(int pindex) : index(pindex) { }
    ~Node() {}
    boost::shared_ptr<Node> left;
    boost::shared_ptr<Node> right;
    int index = -1;
    int comparedim = 0;
    bool assigned = false;
    bool hasleft = false;
    bool hasright = false;
};

typedef boost::shared_ptr<Node> NodePtr;

//* KdTree
/**
* \brief This class is used to construct KdTrees for point clouds, intended to be used for Euclidean clustering.
*/
class KdTree {
 public:
    KdTree(PointCloudPtr pc_, int dimensionality_);
    ~KdTree() {}
    /*!
      \brief Prints the contents of the KdTree using a depth-first search.
      \param n The node at which to start printing and iterating over the tree.
    */
    void print_tree(NodePtr n);

    /*!
      \brief Counts the number of nodes that are children of the given node (n) + 1.
      \param n The node at which to start counting and iterating over the tree.
      \return The number of nodes that children of the given node (n) + 1.
    */
    int count_tree(NodePtr n);

    /*!
      \brief Retrieves the indices of the points with radius_squared squared Euclidean distance of the specified point.
      O(log N) where N = pc->size().
      \param idx The index of the point, who's neighbors we are retrieving.
      \param radius_squared If a point is closer than radius_squared to the desired point, it is a neighbor.
      \return a vector of indices wrt input cloud pc corresponding to the neighbors of idx.
   */
   std::vector<int> radiusSearch(int idx, float radius_squared);

 private:
    uint sample_size = 75;  /*!< "The number of samples used to estimate the median (most informative dimension)." */
    PointCloudPtr pc;
    int dimensionality = 2;  /*!< "Dimensionality of the pointcloud being operated on." */
    float xmin = 0, xmax = 0, ymin = 0, ymax = 0, zmin = 0, zmax = 0;
    NodePtr root;
    /*!
       \brief Constructs a graph of nodes linked together by left, right pointers to children (A KdTree).
       \param parent This parameter indicates the node that will be used as the root of the KdTree.
       \param indices A vector of indices of the points that children of the parent.
       \pre parent needs to be a pointer to an initialized Node object.
       Be sure to set the comparedim value for the root node according to the desired split dimension.
    */
    void construct(NodePtr parent, std::vector<int> &indices);

    /*!
       \brief Using only a small sample of the input points, this function estimates the index of the median point.
       The most informative dimension is determined by calculating the dimension with the most spread (in the sample).
       The median index is then the index of the sample point closest to the middle of the most informative dimension.
       \param indices This vector specifies the subset of the input cloud that we are focusing on finding a median for.
       \param comparedim [out] This parameter will be set to the index of the point to be used as the splitting point.
    */
    int sample_median(std::vector<int> &indices, int &comparedim);

    /*!
       \brief This function sorts the points into the left and right sub-trees based on the parent node.
       Splitting dimension is determined by parent->comparedim.
       Points are sorted into the left subtree if point.dim <= parent.dim, and the right sub-tree otherwise.
       \param parent The node about which the points will be sorted.
       \param indices A list of indices relative to the input cloud (pc) which will be sorted.
       \param lindices [out] This parameter will be filled with the indices of the points in the left sub-tree.
       \param rindices [out] This parameter will be filled with the indices of the points in the right sub-tree.
       \pre parent->comparedim should correspond to the desired split dimension for this node.
    */
    void sort_points(NodePtr parent, std::vector<int> indices, std::vector<int> &lindices,
        std::vector<int> &rindices);

    /*!
       \brief A recurive helper function for finding the neighbors within radius_squared of idx.
       \param indices [out] New neighbors will be pushed back onto of this vector recursively.
       \param node The node about which we will (begin/continue) the search for neighbors of idx.
       \param idx The index of the point about which a radius is being performed.
       \param radius_squared If a point is closer than radius_squared to the desired point, it is a neighbor.
    */
    void radiusSearchHelper(std::vector<int> &indices, NodePtr node, int idx, float radius_squared);

    /*!
       \brief Returns the squared Euclidean distance between two points.
       \param id1 Index of point 1.
       \param id2 Index of point 2.
       \return the squared Euclidean distance.
    */
    float distance(int id1, int id2);

    /*!
       \brief Returns the squared Euclidean distance between two points.
       \param n1 a pointer to Node 1.
       \param n2 a pointer to Node 2.
       \return the squared Euclidean distance.
    */
    float distance(NodePtr n1, NodePtr n2);
};

typedef boost::shared_ptr<KdTree> KdTreePtr;

//* Cluster
/**
* \brief This class is used for performing Euclidean clustering on pointclouds.
*/
class Cluster {
 public:
    Cluster(PointCloudPtr pc_, float tolerance_, int minClusterSize_, int dimensionality);
    ~Cluster() {}

    /*!
       \brief Perform Euclidean clustering and extract the cluster indices.
       \param clusters [out] a vector of clusters, each cluster is a vector of indices wrt the input cloud (pc).
    */
    void extract(std::vector<std::vector<int>> &clusters);

 private:
    PointCloudPtr pc;                   /*!< PointCloud to be clustered */
    float tolerance_squared = 0.0625;   /*!< Points closer than tolerance_squared will be clustered together. */
    int minClusterSize = 5;             /*!< Clusters with less than this many points will not be returned. */
    KdTreePtr kdt;
    std::vector<bool> assigned;
    std::vector<int> indices;
    int current_index = -1;

    /*!
       \brief Retrives the next index to do a radius search on.
       \return returns the index of the point to be used in clustering/searching. -1 if all points have been searched.
    */
    int nextSeedIndex();

    /*!
       \brief Performs a brute force search over all points for neighbors within radius_squared of idx.
       \param idx The index of the point whose neighbors we are retrieving.
       \param radius_squared If a point is closer than radius_squared to the desired point, it is a neighbor.
       \return a vector of indices wrt input cloud pc corresponding to the neighbors of idx.
    */
    std::vector<int> linearRadiusSearch(int idx, float radius_squared);

    /*!
       \brief Returns the squared Euclidean distance between two points.
       \param id1 Index of point 1.
       \param id2 Index of point 2.
       \return the squared Euclidean distance.
    */
    float distance(int id1, int id2);
};

//* Ransac
/**
* \brief This class uses RANSAC to fit a plane to an unordered set of points.
*/
class Ransac {
 public:
    Ransac(PointCloudPtr pc_, float tolerance_, int iterations_) : pc(pc_), tolerance(tolerance_),
       iterations(iterations_) {}
    void setPointCloud(PointCloudPtr pc_) {pc = pc_;}
    /*!
       \brief Perform RANSAC to fit a plane to the input point cloud.
       \post The best model is stored in the private variable best_model.
    */
    void computeModel();

    /*!
       \brief Retrieve the model coefficients for the best model computed by RANSAC.
       \param model The plane parameters (a, b, c, d) where ax + by + cz + d = 0
    */
    void getModelCoefficients(Eigen::Vector4f &model) {model = best_model;}

    /*!
       \brief Retrieves a vector of point indices which are NOT inliers to the plane model.
       \param model the model we are checking for outliers (a,b,c,d)
       \param outliers [out] This parameter is filled with a vector of the outlier indices.
    */
    void get_outliers(Eigen::Vector4f model, std::vector<int> &outliers);

 private:
    PointCloudPtr pc;
    float tolerance = 0.1;                     /*!< Points closer than tolerance to the plane model are inliers. */
    int iterations = 25;                        /*!< The maximum number of iterations that RANSAC will run for. */
    float inlier_ratio = 0.85;                  /*!< If a candidate achieves > inlier_ratio inliers, we return it. */
    Eigen::Vector4f best_model;
    float collinear_angle_threshold = 0.996;    /*!< Used to check for collinear points (cosine(theta)) */
    template <class PointType> int get_num_inliers(PointType n, PointType p);
};

struct Line {
    float m;
    float b;
    float start;
    float end;
    size_t start_idx;
    size_t end_idx;
    Line(PointCloudPtr pc, std::vector<int> line_set, float m_, float b_);
};

//* Himmelsbach
/**
* \brief This class uses the (Himmelsbach, 2010) algo to compute a ground plane and get the inliers to that model.
*/
class Himmelsbach {
 public:
    explicit Himmelsbach(PointCloudPtr pc_) : pc(pc_) {}
    void set_alpha(float alpha_) {alpha = alpha_;}
    void set_tolerance(float tolerance_) {tolerance = tolerance_;}
    void set_thresholds(float Tm_, float Tm_small_, float Tb_, float Trmse_, float Tdprev_) {
       Tm = Tm_;
       Tm_small = Tm_small_;
       Tb = Tb_;
       Trmse = Trmse_;
       Tdprev = Tdprev_;
    }

    /*!
       \brief Computes the ground plane model using the Himmelsbach algo and returns the inliers to that model.
       \param inliers [out] The parameter is filled with a vector of indices which correspond to the ground points.
    */
    std::vector<std::pair<std::vector<float>, std::vector<float> > > compute_model_and_get_inliers(std::vector<int> &inliers);

 private:
    PointCloudPtr pc;
    float alpha = 5 * M_PI / 180;       /*!< Number of segments = (2 * M_PI) / alpha */
    int num_bins_small = 20;
    int num_bins_large = 16;
    float bin_size_small = 0.1;
    float bin_size_large = 0.5;
    float rmin = 0;
    float rmax = 10;
    float tolerance = 0.25;         /*!< Metric distance from the ground plane model to be considered a ground point */
    float Tm = 15 * M_PI / 180;         /*!< Slopes greater than this value will be considered non-ground. */
    float Tm_small = 5 * M_PI / 180;    /*!< Slopes less than this value will be checked for being a plateau */
    float Tb = 1.5;                /*!< Flat regions that are higher than this value will be considered non-ground */
    float Trmse = 0.1;                  /*!< If the RSME of the line fit exceeds this value, it will be rejected */
    float Tdprev = 0.25;                /*!< Maximum allowed distance between previous line and start of new line */

    /*!
       \brief Sorts points into one of N = (2 * pi) / alpha segments.
       \param segments [out] This parameter will be filled with a vector of segments, each segment consists of a
           vector of points indices.
       \post If a segment does not contain a point, it will be set to an empty vector.
    */
    void sort_points_segments(std::vector<std::vector<int>> &segments);

    /*!
       \brief Sorts points into bins and extracts a representative point for each bin.
       The point with the lowest z-value in each bin becomes the representative point.
       \param segment A vector of point indices for the current segment.
       \param bins [out] This parameter will be set to a vector of indices corresponding to the representative point
            for each bin.
       \post If a bin does not contain a point, the index will be set to -1.
    */
    void sort_points_bins(const std::vector<int>& segment, std::vector<int> &bins);

    /*!
       \brief Fits a line to a vector of points using Eigen's linear solver.
       \param line_set A vector of point indices which will be used to fit a line.
       \param m [out] The slope of the output line. z = m*r +b
       \param b [out] The z-intercept of the output line. z = m*r + b
    */
    float fitline(const std::vector<int>& line_set, float&m, float&b);
    float fitline(std::vector<int> line_set, int idx, float&m, float&b);

    /*!
       \brief Computes the distance from a point to a line.
       \param line A line object (z = m*r + b)
       \param idx the index of the point.
       \return returns the Euclidean distance from the point to the line.
    */
    float distpointline(const Line& line, int idx);
};

}  // namespace zeus_pcl
