// Author: Keenan Burnett
#include <string>
#include <vector>
#include "utils/transform_utils.hpp"

namespace zeus_tf {

Eigen::Matrix4d get_inverse_tf(Eigen::Matrix4d T) {
    Eigen::Matrix4d inv = Eigen::MatrixXd::Identity(4, 4);
    Eigen::Matrix<double, 3, 3> R = T.block<3, 3>(0, 0);
    Eigen::Matrix<double, 3, 1> t = T.block<3, 1>(0, 3);
    inv.block<3, 3>(0, 0) = R.transpose();
    inv.block<3, 1>(0, 3) = -R.transpose()*t;
    return inv;
}

void get_rpy_from_odom(nav_msgs::Odometry odom, std::vector<double> &rpy) {
    double EPS = 1e-15;  // check if things are close to zero

    int i = 0, j = 1, k = 2;
    Eigen::Matrix4d T;
    get_odom_tf(odom, T);
    double cy = sqrt(T(i, i) * T(i, i) + T(j, i) * T(j, i));
    double ax = 0, ay = 0, az = 0;
    if (cy > EPS) {
        ax = atan2(T(k, j), T(k, k));
        ay = atan2(-T(k, i), cy);
        az = atan2(T(j, i), T(i, i));
    } else {
        ax = atan2(-T(j, k), T(j, j));
        ay = atan2(-T(k, i), cy);
        az = 0.0;
    }
    rpy = {ax, ay, az};
}

void get_odom_tf(nav_msgs::Odometry odom, Eigen::Matrix4d & Toi) {
    double init_x = 0;
    double init_y = 0;
    double init_z = 0;
    bool init_pos_set = false;
    get_odom_tf(odom, init_x, init_y, init_z, init_pos_set, false, Toi);
}

void get_odom_tf(nav_msgs::Odometry odom, double &init_x, double &init_y,
    double &init_z, bool &init_pos_set, bool zero_odom, Eigen::Matrix4d & Toi) {
    if (zero_odom) {
        if (!init_pos_set) {
            init_x = odom.pose.pose.position.x;
            init_y = odom.pose.pose.position.y;
            init_z = odom.pose.pose.position.z;
            init_pos_set = true;
        }
    }
    double EPS = 1e-15;  // check if things are close to zero

    double x = odom.pose.pose.position.x - init_x;
    double y = odom.pose.pose.position.y - init_y;
    double z = odom.pose.pose.position.z - init_z;
    double qx = odom.pose.pose.orientation.x;
    double qy = odom.pose.pose.orientation.y;
    double qz = odom.pose.pose.orientation.z;
    double qw = odom.pose.pose.orientation.w;

    Eigen::Matrix<double, 3, 1> p = {x, y, z};
    Eigen::Vector4d qbar = {qx, qy, qz, qw};
    Eigen::Matrix3d R = Eigen::MatrixXd::Identity(3, 3);
    if (qbar.transpose() * qbar > EPS) {
        // Convert ROS quaternion to a rotation matrix
        Eigen::Matrix<double, 3, 1> epsilon = {qx, qy, qz};
        double eta = qw;
        Eigen::Matrix<double, 3, 3> epsilon_cross;
        epsilon_cross << 0, -epsilon(2), epsilon(1),
                         epsilon(2), 0, -epsilon(0),
                         -epsilon(1), epsilon(0), 0;
        Eigen::Matrix3d I = Eigen::MatrixXd::Identity(3, 3);
        R = (pow(eta, 2.0) - epsilon.transpose() * epsilon) * I +
            2 * (epsilon * epsilon.transpose()) - 2 * eta * epsilon_cross;
    }
    // TODO(keenan): ensure this is a valid rotation
    Toi << R.transpose(), p, Eigen::MatrixXd::Zero(1, 3), 1.0;
}

// Converts a ROS transform into a 4x4 transformation matrix (C++ Eigen)
static void convert_to_eigen(geometry_msgs::TransformStamped tf, Eigen::Matrix4d & T) {
    geometry_msgs::Quaternion q_ros = tf.transform.rotation;  // **** (x, y, z, w)
    geometry_msgs::Vector3 p_ros = tf.transform.translation;  // (x,y,z)
    Eigen::Matrix<double, 3, 1> p = {p_ros.x, p_ros.y, p_ros.z};

    // Convert ROS quaternion to a rotation matrix
    Eigen::Matrix<double, 3, 1> epsilon = {q_ros.x, q_ros.y, q_ros.z};
    double eta = q_ros.w;
    Eigen::Matrix<double, 3, 3> epsilon_cross;
    epsilon_cross << 0, -epsilon(2), epsilon(1),
                     epsilon(2), 0, -epsilon(0),
                     -epsilon(1), epsilon(0), 0;
    Eigen::Matrix<double, 3, 3> R, I;
    I = Eigen::MatrixXd::Identity(3, 3);
    R = (pow(eta, 2.0) - epsilon.transpose() * epsilon) * I +
        2 * (epsilon * epsilon.transpose()) - 2 * eta * epsilon_cross;
    p = -1 * R * p;
    // TODO(keenan): ensure this is a valid rotation
    T << R, p, Eigen::MatrixXd::Zero(1, 3), 1.0;
}

void get_transform(tfBufferPtr tfBuffer, std::string src_frame, std::string tgt_frame, Eigen::Matrix4d& T) {
    bool got_tf = false;
    float timeout = 0.025;
    geometry_msgs::TransformStamped tf_temp;
    while (!got_tf && ros::ok()) {
        try {
            tf_temp = tfBuffer->lookupTransform(src_frame, tgt_frame, ros::Time(0), ros::Duration(timeout));
            got_tf = true;
        }
        catch (tf2::TransformException &ex) {
            got_tf = false;
            ROS_WARN_STREAM_THROTTLE(1, "Transform from "  << src_frame << " to " << tgt_frame <<
                " was not found! Is TF tree and clock being published?");
        }
        if (got_tf) {
            convert_to_eigen(tf_temp, T);
        }
    }
}
}  // namespace zeus_tf
