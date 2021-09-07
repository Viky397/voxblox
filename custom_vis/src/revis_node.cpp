#include <ros/ros.h>
// #include <iostream>
#include <fstream>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
// #include <geodesy/utm.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <zeus_msgs/Detections3D.h>
#include <math.h>

enum class Sign {
    SIGN_DO_NOT_ENTER = 0,
    SIGN_RIGHT_TURN_ONLY = 1,
    SIGN_LEFT_TURN_ONLY = 2,
    SIGN_PARKING = 3,
    SIGN_DISABILITY_PARKING = 4,
    SIGN_SPEED_LIMIT_5 = 5,
    SIGN_SPEED_LIMIT_10 = 6,
    SIGN_SPEED_LIMIT_15 = 7,
    SIGN_SPEED_LIMIT_20 = 8,
    SIGN_SPEED_LIMIT_25 = 9,
    SIGN_STOP = 10,
    SIGN_RAILROAD = 11,
    SIGN_DO_NOT_TURN_LEFT = 12,
    SIGN_DO_NOT_TURN_RIGHT = 13,
	SIGN_YIELD = 14,
    SIGN_INVALID = 15
};

static inline Sign toSign(int id) {
    Sign ret;
    switch (id) {
    case 0 :
    {
        ret = Sign::SIGN_DO_NOT_ENTER;
        break;
    }
    case 1 :
    {
        ret = Sign::SIGN_RIGHT_TURN_ONLY;
        break;
    }
    case 2 :
    {
        ret = Sign::SIGN_LEFT_TURN_ONLY;
        break;
    }
    case 3 :
    {
        ret = Sign::SIGN_PARKING;
        break;
    }
    case 4 :
    {
        ret = Sign::SIGN_DISABILITY_PARKING;
        break;
    }
    case 5 :
    {
        ret = Sign::SIGN_SPEED_LIMIT_5;
        break;
    }
    case 6 :
    {
        ret = Sign::SIGN_SPEED_LIMIT_10;
        break;
    }
    case 7 :
    {
        ret = Sign::SIGN_SPEED_LIMIT_15;
        break;
    }
    case 8 :
    {
        ret = Sign::SIGN_SPEED_LIMIT_20;
        break;
    }
    case 9 :
    {
        ret = Sign::SIGN_SPEED_LIMIT_25;
        break;
    }
    case 10 :
    {
        ret = Sign::SIGN_STOP;
        break;
    }
    case 11 :
    {
        ret = Sign::SIGN_RAILROAD;
        break;
    }
    case 12 :
    {
        ret = Sign::SIGN_DO_NOT_TURN_LEFT;
        break;
    }
    case 13 :
    {
        ret = Sign::SIGN_DO_NOT_TURN_RIGHT;
        break;
    }
    case 14 :
    {
        ret = Sign::SIGN_YIELD;
        break;
    }
    default :
    {
        ret = Sign::SIGN_INVALID;
        break;
    }
    }
    return ret;
}

static inline std::string toString(int n) {
    std::string ret;
    Sign sign = toSign(n);
    switch (sign) {
    case Sign::SIGN_DO_NOT_ENTER :
    {
        ret = std::string("Do Not Enter");
        break;
    }
    case Sign::SIGN_RIGHT_TURN_ONLY :
    {
        ret = std::string("Right Turn Only");
        break;
    }
    case Sign::SIGN_LEFT_TURN_ONLY :
    {
        ret = std::string("Left Turn Only");
        break;
    }
    case Sign::SIGN_PARKING :
    {
        ret = std::string("Parking");
        break;
    }
    case Sign::SIGN_DISABILITY_PARKING :
    {
        ret = std::string("Disability Parking");
        break;
    }
    case Sign::SIGN_SPEED_LIMIT_5 :
    {
        ret = std::string("Speed Limit 5");
        break;
    }
    case Sign::SIGN_SPEED_LIMIT_10 :
    {
        ret = std::string("Speed Limit 10");
        break;
    }
    case Sign::SIGN_SPEED_LIMIT_15 :
    {
        ret = std::string("Speed Limit 15");
        break;
    }
    case Sign::SIGN_SPEED_LIMIT_20 :
    {
        ret = std::string("Speed Limit 20");
        break;
    }
    case Sign::SIGN_SPEED_LIMIT_25 :
    {
        ret = std::string("Speed Limit 25");
        break;
    }
    case Sign::SIGN_STOP :
    {
        ret = std::string("Stop");
        break;
    }
    case Sign::SIGN_RAILROAD :
    {
        ret = std::string("Railroad crossing");
        break;
    }
    case Sign::SIGN_DO_NOT_TURN_LEFT :
    {
        ret = std::string("Do Not Turn Left ");
        break;
    }
    case Sign::SIGN_DO_NOT_TURN_RIGHT :
    {
        ret = std::string("Do Not Turn Right");
        break;
    }
    case Sign::SIGN_YIELD :
    {
        ret = std::string("Yield");
        break;
    }
    default :
    {
        ret = std::string("Invalid");
        break;
    }
    }
    return ret;
}

class Revis {
	typedef pcl::PointXYZI PointTypeI;
	typedef pcl::PointCloud<PointTypeI> PointCloudI;
public:

	Revis() :
			nh_(ros::NodeHandle("~")) {

		// inspvax_sub_ = nh_.subscribe("/novatel_data/inspvax", 1, &SBETTranslator::INSPVAXCallback, this);
		// inspvax_sub_ = nh_.subscribe("/navsat/odom", 1, &SBETTranslator::NavsatCallback, this);
		// write_serv_ = nh_.advertiseService("write_sbet", &SBETTranslator::WriteSBETCallback, this);
		b_first_odom_ = false;
	}

	void init() {
		// parameters
		nh_.param<double>("downsample", downsample_, 0.5);
		if (downsample_ < 1e-3)
			downsample_ = 0;

		// lookup tf
		uint tf_lookup_attempts = 1;
		/***
		while (tf_lookup_attempts != 0) {
			try {
				lr_.lookupTransform("velodyne", "imu_link", ros::Time(0),
						Tf_vel_imu_);
				tf_lookup_attempts = 0;
				std::cout <<
						"[Revis][init()] Transform lookup found for velodyne to imu_link" << std::endl;

			} catch (...) {
				std::cout <<
						"[Revis][init()] Transform lookup not available for velodyne to imu_link. Trying again in 1.0 second." <<std::endl;
				ros::Duration(1.0).sleep();
				++tf_lookup_attempts;
				if (tf_lookup_attempts > 1000)
					throw std::runtime_error(
							"[Revis][init()] Transform lookup not available for velodyne to imu. Exceeded attempts.");

			}
		}
		***/

		// subscribers
		s_mpc_ref_ = nh_.subscribe("/mpc_references", 1, &Revis::MPCRefCallback, this);
		s_mpc_pred_ = nh_.subscribe("/mpc_predictions", 1, &Revis::MPCPredCallback, this);
		s_path_max_vel_ = nh_.subscribe("/Debug/max_speed", 1, &Revis::MaxSpeedCallback, this);
		s_odom_ = nh_.subscribe("/navsat/odom", 1, &Revis::OdomCallback, this);
		s_tf_odom_ = nh_.subscribe("/Debug/transformed_pose", 1, &Revis::TFOdomCallback, this);
		s_planner_roads_ = nh_.subscribe("/planner_roads", 1,
				&Revis::RoadsCallback, this);
		s_planner_lattice_ = nh_.subscribe("/planner_lattice", 1,
						&Revis::LatticeCallback, this);
		s_desired_path_ = nh_.subscribe("/PathPlanner/desired_path", 1,
				&Revis::DesiredPathCallback, this);
		s_planner_parking_lots_ = nh_.subscribe("/planner_parking_lots", 1,
				&Revis::ParkingCallback, this);
		s_planner_objects_ = nh_.subscribe("/planner_obstacles", 1,
				&Revis::ObjectsCallback, this);
		s_planner_lights_ = nh_.subscribe("/planner_lights", 1,
						&Revis::LightsCallback, this);
		s_detected_lights_ = nh_.subscribe("/planner_detected_lights", 1,
						&Revis::DetectedLightsCallback, this);
		s_pointcloud_ = nh_.subscribe("/velodyne_points", 1,
				&Revis::PointcloudCallback, this);
		s_obstacle_1_ = nh_.subscribe("/Object/Detections3D",10,&Revis::Obs1Callback, this);
		//s_obstacle_2_ = nh_.subscribe("/Object/RawDetections3D_secondary",10,&Revis::Obs2Callback, this);
		//s_obstacle_3_ = nh_.subscribe("/Object/Detections3D_4",10,&Revis::Obs3Callback, this);
        s_trafficsigns_ = nh_.subscribe("/TrafficSign/Detections3D", 10, &Revis::TrafficSignsCallback, this);
        s_speedsigns_ = nh_.subscribe("/TrafficSign/SpeedDetections3D", 10, &Revis::SpeedSignsCallback, this);

        // publishers
		p_mpc_ref_ = nh_.advertise<visualization_msgs::Marker>("mpc_references", 1);
		p_mpc_pred_ = nh_.advertise<visualization_msgs::Marker>("mpc_predictions", 1);
		p_path_max_vel_ = nh_.advertise<nav_msgs::Path>("max_speed", 1);
		p_tf_odom_ = nh_.advertise<nav_msgs::Odometry>("transformed_odom", 1);
		p_desired_path_ = nh_.advertise<nav_msgs::Path>("desired_path", 1);
		p_planner_roads_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"planner_roads", 1);
		p_lattice_ = nh_.advertise<visualization_msgs::MarkerArray>(
						"planner_lattice", 1);
		p_planner_parking_lots_ =
				nh_.advertise<visualization_msgs::MarkerArray>(
						"planner_parking_lots", 1);
		p_planner_objects_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"planner_obstacles", 1);
		p_planner_lights_ = nh_.advertise<visualization_msgs::MarkerArray>(
						"planner_lights", 1);
		p_detected_lights_ = nh_.advertise<visualization_msgs::MarkerArray>(
						"detected_lights", 1);
		p_pointcloud_ = nh_.advertise<sensor_msgs::PointCloud2>(
				"velodyne_points", 1);
		p_obstacle_1_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"detected_obstacles_1", 1);
		p_obstacle_label_1_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"detected_obstacle_labels_1", 1);
		p_obstacle_predict_1_ = nh_.advertise<visualization_msgs::MarkerArray>(
						"detected_obstacle_prediction_1", 1);
		//p_obstacle_2_ = nh_.advertise<visualization_msgs::MarkerArray>(
		//		"detected_obstacles_secondary", 1);
		//p_obstacle_3_ = nh_.advertise<visualization_msgs::MarkerArray>(
		//		"detected_obstacles_3", 1);
        p_trafficsigns_ = nh_.advertise<visualization_msgs::MarkerArray>("traffic_signs", 1);
        p_trafficsigns_label_ = nh_.advertise<visualization_msgs::MarkerArray>("traffic_signs_label", 1);
        p_speedsigns_ = nh_.advertise<visualization_msgs::MarkerArray>("speed_signs", 1);
        p_speedsigns_label_ = nh_.advertise<visualization_msgs::MarkerArray>("speed_signs_label", 1);
    }

	void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		if (!b_first_odom_) {
			x_ref_ = msg->pose.pose.position.x;
			y_ref_ = msg->pose.pose.position.y;
			z_ref_ = msg->pose.pose.position.z;
			b_first_odom_ = true;
			std::cout << "[Revis] Set init pose x: " << x_ref_ << " y: "
					<< y_ref_ << " z: " << z_ref_ << std::endl;
		}

		// publish local tf
		geometry_msgs::Pose pose_local = msg->pose.pose;
		pose_local.position.x -= x_ref_;
		pose_local.position.y -= y_ref_;
		pose_local.position.z -= z_ref_ - 1.5;
		x_cur_ = pose_local.position.x;
		y_cur_ = pose_local.position.y;
		z_cur_ = pose_local.position.z;
		tf::Transform Tf_odom_imu;
		tf::poseMsgToTF(pose_local, Tf_odom_imu);
		// br_.sendTransform(tf::StampedTransform(Tf_odom_imu.inverse(),
		//                                    ros::Time::now(),
		//                                    "imu_link",
		//                                    "odom_local"));
		// br_.sendTransform(tf::StampedTransform(Tf_odom_imu.inverse(),
		//                                msg->header.stamp,
		//                                "imu_link",
		//                                "odom_local"));

		br_.sendTransform(
				tf::StampedTransform(Tf_odom_imu, msg->header.stamp,
						"odom_local", "imu_link_local"));

		br_.sendTransform(
				tf::StampedTransform(Tf_vel_imu_.inverse(), msg->header.stamp,
						"imu_link_local", "velodyne_local"));

		return;
	}

	void TFOdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		nav_msgs::Odometry msg_new = *msg;

		msg_new.pose.pose.position.x -= x_ref_;
		msg_new.pose.pose.position.y -= y_ref_;
		msg_new.pose.pose.position.z -= z_ref_;

		msg_new.header.frame_id = "odom_local";
		p_tf_odom_.publish(msg_new);
		return;
	}

	void MPCRefCallback(const visualization_msgs::Marker::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		visualization_msgs::Marker msg_new = *msg;
		for (int j = 0; j < msg_new.points.size(); ++j) {
			msg_new.points[j].x -= x_ref_;
			msg_new.points[j].y -= y_ref_;
		}

		msg_new.header.frame_id = "odom_local";
		p_mpc_ref_.publish(msg_new);

		return;
	}

	void MPCPredCallback(const visualization_msgs::Marker::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		visualization_msgs::Marker msg_new = *msg;
		for (int j = 0; j < msg_new.points.size(); ++j) {
			msg_new.points[j].x -= x_ref_;
			msg_new.points[j].y -= y_ref_;
		}

		msg_new.header.frame_id = "odom_local";
		p_mpc_pred_.publish(msg_new);

		return;
	}

	void MaxSpeedCallback(const nav_msgs::Path::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		nav_msgs::Path msg_new = *msg;
		msg_new.header.frame_id = "odom_local";

		for (int i = 0; i < msg_new.poses.size(); ++i) {
			msg_new.poses[i].pose.position.x -= x_ref_;
			msg_new.poses[i].pose.position.y -= y_ref_;
			msg_new.poses[i].pose.position.z -= z_ref_;
		}
		p_path_max_vel_.publish(msg_new);

		return;
	}

	void DesiredPathCallback(const nav_msgs::Path::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		nav_msgs::Path msg_new = *msg;
		msg_new.header.frame_id = "odom_local";

		for (int i = 0; i < msg_new.poses.size(); ++i) {
			msg_new.poses[i].pose.position.x -= x_ref_;
			msg_new.poses[i].pose.position.y -= y_ref_;
			//msg_new.poses[i].pose.position.z -= z_ref_;
		}
		p_desired_path_.publish(msg_new);

		return;
	}

	void RoadsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		visualization_msgs::MarkerArray msg_new = *msg;

		for (int i = 0; i < msg_new.markers.size(); ++i) {
			if (msg_new.markers[i].type == visualization_msgs::Marker::ARROW
					|| msg_new.markers[i].type
							== visualization_msgs::Marker::LINE_STRIP
					||		 msg_new.markers[i].type
							== visualization_msgs::Marker::LINE_LIST) {
				for (int j = 0; j < msg_new.markers[i].points.size(); ++j) {
					msg_new.markers[i].points[j].x -= x_ref_;
					msg_new.markers[i].points[j].y -= y_ref_;
					msg_new.markers[i].points[j].z -= z_ref_;
				}
			} else if (msg_new.markers[i].type
					== visualization_msgs::Marker::TEXT_VIEW_FACING) {
				msg_new.markers[i].pose.position.x -= x_ref_;
				msg_new.markers[i].pose.position.y -= y_ref_;
				msg_new.markers[i].pose.position.z -= z_ref_;
			}
			msg_new.markers[i].header.frame_id = "odom_local";
		}
		p_planner_roads_.publish(msg_new);

		return;
	}

	void LatticeCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		visualization_msgs::MarkerArray msg_new = *msg;

		for (int i = 0; i < msg_new.markers.size(); ++i) {
			if (msg_new.markers[i].type == visualization_msgs::Marker::LINE_STRIP
					|| msg_new.markers[i].type == visualization_msgs::Marker::LINE_LIST) {
				for (int j = 0; j < msg_new.markers[i].points.size(); ++j) {
					msg_new.markers[i].points[j].x -= x_ref_;
					msg_new.markers[i].points[j].y -= y_ref_;
					msg_new.markers[i].points[j].z -= z_ref_;
				}
			}
			msg_new.markers[i].header.frame_id = "odom_local";
		}

		p_lattice_.publish(msg_new);
		return;
	}

	void ParkingCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		visualization_msgs::MarkerArray msg_new = *msg;

		for (int i = 0; i < msg_new.markers.size(); ++i) {
			if (msg_new.markers[i].type
					== visualization_msgs::Marker::TRIANGLE_LIST) {
				for (int j = 0; j < msg_new.markers[i].points.size(); ++j) {
					msg_new.markers[i].points[j].x -= x_ref_;
					msg_new.markers[i].points[j].y -= y_ref_;
					msg_new.markers[i].points[j].z -= z_ref_;
				}
			} else if (msg_new.markers[i].type
					== visualization_msgs::Marker::TEXT_VIEW_FACING) {
				msg_new.markers[i].pose.position.x -= x_ref_;
				msg_new.markers[i].pose.position.y -= y_ref_;
				msg_new.markers[i].pose.position.z -= z_ref_;
			}
			msg_new.markers[i].header.frame_id = "odom_local";
		}
		p_planner_parking_lots_.publish(msg_new);

		return;
	}

	void LightsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		visualization_msgs::MarkerArray msg_new = *msg;

		for (int i = 0; i < msg_new.markers.size(); ++i) {
			if (msg_new.markers[i].type == visualization_msgs::Marker::SPHERE
					|| msg_new.markers[i].type
							== visualization_msgs::Marker::TEXT_VIEW_FACING) {
				msg_new.markers[i].pose.position.x -= x_ref_;
				msg_new.markers[i].pose.position.y -= y_ref_;
				msg_new.markers[i].pose.position.z -= z_ref_;
			}
			msg_new.markers[i].header.frame_id = "odom_local";
		}
		p_planner_lights_.publish(msg_new);

		return;
	}

	void DetectedLightsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		visualization_msgs::MarkerArray msg_new = *msg;

		for (int i = 0; i < msg_new.markers.size(); ++i) {
			if (msg_new.markers[i].type == visualization_msgs::Marker::SPHERE
					|| msg_new.markers[i].type
							== visualization_msgs::Marker::TEXT_VIEW_FACING) {
				msg_new.markers[i].pose.position.x -= x_ref_;
				msg_new.markers[i].pose.position.y -= y_ref_;
				msg_new.markers[i].pose.position.z -= z_ref_;
			}
			msg_new.markers[i].header.frame_id = "odom_local";
		}
		p_detected_lights_.publish(msg_new);

		return;
	}

	void ObjectsCallback(const visualization_msgs::MarkerArray::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		visualization_msgs::MarkerArray msg_new = *msg;
		for (int i = 0; i < msg_new.markers.size(); ++i) {
			msg_new.markers[i].pose.position.x -= x_ref_;
			msg_new.markers[i].pose.position.y -= y_ref_;
			msg_new.markers[i].pose.position.z -= z_ref_;
			msg_new.markers[i].header.frame_id = "odom_local";
		}
		p_planner_objects_.publish(msg_new);

		return;
	}

	void PointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
		if (!b_first_odom_)
			return;

		if (downsample_ == 0)
			return;

		// sensor_msgs::PointCloud2 msg_new = *msg;
		// msg_new.header.frame_id = "velodyne_local";

		PointCloudI::Ptr msg_new(new PointCloudI);
		msg_new->header.frame_id = "velodyne_local";
		msg_new->height = 1;

		PointTypeI pcl_point;

		unsigned int max_num_points = msg->width;
		sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
		sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
		sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
		sensor_msgs::PointCloud2ConstIterator<float> iter_i(*msg, "intensity");
		sensor_msgs::PointCloud2ConstIterator<int> iter_r(*msg, "ring");

		unsigned int point_count = 0;
		for (int i = 0; i < max_num_points;
				++i, ++iter_i, ++iter_x, ++iter_y, ++iter_z, ++iter_r) {
			if ((float) std::rand() / (float) RAND_MAX < downsample_) {
				pcl_point.x = *iter_x;
				pcl_point.y = *iter_y;
				pcl_point.z = *iter_z;
				pcl_point.intensity = *iter_i;
				// const int &ring = *iter_r;
				// std::cout <<  pcl_point.x  << ", " <<  pcl_point.y  << ", " <<  pcl_point.z  << ", " <<  pcl_point.intensity  << std::endl;

				msg_new->points.push_back(pcl_point);
				++point_count;
			}

		}

		msg_new->width = point_count;

		p_pointcloud_.publish(msg_new);

		return;
	}

	void Obs1Callback(const zeus_msgs::Detections3DConstPtr& msg) {
		//set to false then check all obstacles.
		std::vector<zeus_msgs::BoundingBox3D> bbs = msg->bbs;

		visualization_msgs::MarkerArray msg_new;
		visualization_msgs::MarkerArray msg_label_new;
		visualization_msgs::MarkerArray msg_predict_new;
		visualization_msgs::Marker obstacle_marker;
		visualization_msgs::Marker obstacle_label;
		visualization_msgs::Marker obstacle_predict;
		// init marker
		obstacle_marker.header.frame_id = "map";
		obstacle_marker.header.stamp = ros::Time();
		obstacle_marker.ns = "obstacles1";
		obstacle_marker.id = 1;
		obstacle_marker.type = visualization_msgs::Marker::CUBE;
		obstacle_marker.action = visualization_msgs::Marker::ADD;
		obstacle_marker.lifetime = ros::Duration(0);
		obstacle_marker.color.a = 0.8;
		obstacle_marker.color.r = 1.0;
		obstacle_marker.color.g = 0.6;
		obstacle_marker.color.b = 0.0;
		obstacle_marker.scale.x = 1;
		obstacle_marker.scale.y = 1;
		obstacle_marker.scale.z = 1.7;

		obstacle_label = obstacle_marker;
		obstacle_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		obstacle_label.action = visualization_msgs::Marker::ADD;
		obstacle_label.lifetime = ros::Duration(0);
		obstacle_label.color.r = 1.0;
		obstacle_label.color.g = 1.0;
		obstacle_label.color.b = 1.0;
		obstacle_label.scale.x = 0;
		obstacle_label.scale.y = 0;
		obstacle_label.scale.z = 0.5;

		obstacle_predict = obstacle_marker;
		obstacle_predict.type = visualization_msgs::Marker::LINE_STRIP;
		obstacle_predict.action = visualization_msgs::Marker::ADD;
		obstacle_predict.lifetime = ros::Duration(0);
		obstacle_predict.color.r = 1.0;
		obstacle_predict.color.g = 0;
		obstacle_predict.color.b = 0;
		obstacle_predict.scale.x = 0.1;
		obstacle_predict.scale.y = 0;
		obstacle_predict.scale.z = 0;

		if (bbs.size() == 0 ){//|| bbs.size() < prev_obj_num_) {
			obstacle_marker.action = visualization_msgs::Marker::DELETEALL;
			msg_new.markers.push_back(obstacle_marker);

			obstacle_label.action = visualization_msgs::Marker::DELETEALL;
			msg_label_new.markers.push_back(obstacle_label);

			obstacle_predict.action = visualization_msgs::Marker::DELETEALL;
			msg_predict_new.markers.push_back(obstacle_predict);
		}
		prev_obj_num_ = bbs.size();

		for (size_t i(0); i<bbs.size(); i++) {
			obstacle_marker.color.r = 1.0;
			obstacle_marker.color.g = 0.0;
			obstacle_marker.color.b = 0.0;
			std::string str_type;

			if (bbs[i].type == 1)
				str_type = "Pedestrian";
			else if (bbs[i].type == 3)
				str_type = "Unknown";
			else if (bbs[i].type == 4)
				str_type = "Unknown Dynamic";

			double speed = sqrt(pow(bbs[i].x_dot, 2) + pow(bbs[i].y_dot, 2));

			if ((bbs[i].type == 1 && bbs[i].confidence < 0.01) || (bbs[i].type == 3 && speed <= 1.0)){
				obstacle_marker.color.r = 0.7;
				obstacle_marker.color.g = 0.6;
				obstacle_marker.color.b = 0.0;
			}

	        double x = bbs[i].x;
	        double y = bbs[i].y;
	        double z = bbs[i].z;
	        double quat_z = cos(bbs[i].yaw/2);

			obstacle_marker.pose.position.x = x;
			obstacle_marker.pose.position.y = y;
			obstacle_marker.pose.position.z = z;
			obstacle_marker.scale.x = bbs[i].w;
			obstacle_marker.scale.y = bbs[i].l;
			obstacle_marker.scale.z = bbs[i].h;
            obstacle_marker.pose.orientation.w = sqrt(1 - quat_z);
            obstacle_marker.pose.orientation.x = 0;
            obstacle_marker.pose.orientation.y = 0;
            obstacle_marker.pose.orientation.z = quat_z;
            obstacle_marker.id++;

            if (bbs[i].confidence > 0.0) {
            	msg_new.markers.push_back(obstacle_marker);

            	if (true) { // bbs[i].type == 1 || speed >= 0.8 || bbs[i].confidence > 0.1) {
    				obstacle_label.pose.position.x = x;
    				obstacle_label.pose.position.y = y;
    				obstacle_label.pose.position.z = 3;
    				obstacle_label.pose.orientation.w = sqrt(1 - quat_z);
    				obstacle_label.pose.orientation.x = 0;
    				obstacle_label.pose.orientation.y = 0;
    				obstacle_label.pose.orientation.z = quat_z;
    				double dist = sqrt(pow(x_cur_-x, 2) + pow(y_cur_-y, 2));
    				std::string str_speed = std::to_string(speed);
    				std::string str_conf = std::to_string(bbs[i].confidence * 100.0);
    				std::string str_bb_h = std::to_string(bbs[i].h);
    				std::string str_bb_l = std::to_string(bbs[i].l);
    				std::string str_bb_w = std::to_string(bbs[i].w);
    				std::string str_dist = std::to_string(dist);
    				obstacle_label.text = std::string("Obj ID: ") + std::to_string(bbs[i].ID) + std::string("  Conf: ") + str_conf.substr(0, str_conf.find(".") + 3);

    						// str_bb_h.substr(0, str_bb_h.find(".") + 3) + std::string("  ") + str_bb_l.substr(0, str_bb_l.find(".") + 3) + std::string("  ") + str_bb_w.substr(0, str_bb_w.find(".") + 3);
    				msg_label_new.markers.push_back(obstacle_label);
    				obstacle_label.id++;

    				obstacle_predict.points.clear();
    				geometry_msgs::Point pm;
    				pm.x = x;
    				pm.y = y;
    				pm.z = 0.0;
    				for (int j(0); j<6; j++) {
    					obstacle_predict.points.push_back(pm);
    					pm.x += bbs[i].x_dot;
    					pm.y += bbs[i].y_dot;
    				}
    				obstacle_predict.id++;
    				msg_predict_new.markers.push_back(obstacle_predict);
            	}
            }
		}

		p_obstacle_1_.publish(msg_new);
		p_obstacle_label_1_.publish(msg_label_new);
		p_obstacle_predict_1_.publish(msg_predict_new);
	}

    void TrafficSignsCallback(const zeus_msgs::Detections3DConstPtr& msg){
        if (!b_first_odom_)
            return;

        visualization_msgs::MarkerArray marker_arr, marker_label_arr;
        std::vector<zeus_msgs::BoundingBox3D> signs = msg->bbs;
        visualization_msgs::Marker marker;
        visualization_msgs::Marker marker_label;

        marker.header.frame_id = "odom_local";
		marker.header.stamp = ros::Time();
		marker.ns = "traffic_signs";

		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration(0.2);

		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 4;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;

		marker_label = marker;
		marker_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker_label.action = visualization_msgs::Marker::ADD;
		marker_label.lifetime = ros::Duration(0);
		marker_label.scale.z = 1;
		marker_label.color.a = 1.0;
		marker_label.color.r = 1.0;
		marker_label.color.g = 1.0;
		marker_label.color.b = 1.0;


        for (int i = 0; i < signs.size(); ++i) {
        	if (signs[i].confidence < 0.7) continue;
        	marker.id = i;
            marker.pose.position.x = signs[i].x - x_ref_;
            marker.pose.position.y = signs[i].y - y_ref_;
            marker.pose.position.z = 3;

            marker_label.id = i;
            marker_label.pose.position.x = signs[i].x - x_ref_;
            marker_label.pose.position.y = signs[i].y - y_ref_;
            marker_label.pose.position.z = 6;
            marker_label.text = toString(signs[i].type);

            marker_arr.markers.push_back(marker);
            marker_label_arr.markers.push_back(marker_label);
        }

		if (signs.size() == 0) {
			marker.action = visualization_msgs::Marker::DELETEALL;
			marker_arr.markers.push_back(marker);

			marker_label.action = visualization_msgs::Marker::DELETEALL;
			marker_label_arr.markers.push_back(marker_label);
		}


        p_trafficsigns_.publish(marker_arr);
        p_trafficsigns_label_.publish(marker_label_arr);

        return;
    }

    void SpeedSignsCallback(const zeus_msgs::Detections3DConstPtr& msg){
        if (!b_first_odom_)
            return;

        visualization_msgs::MarkerArray marker_arr, marker_label_arr;
        std::vector<zeus_msgs::BoundingBox3D> signs = msg->bbs;
        visualization_msgs::Marker marker;
        visualization_msgs::Marker marker_label;

        marker.header.frame_id = "odom_local";
		marker.header.stamp = ros::Time();
		marker.ns = "traffic_signs";

		marker.type = visualization_msgs::Marker::CYLINDER;
		marker.action = visualization_msgs::Marker::ADD;
		marker.lifetime = ros::Duration(0.2);

		marker.pose.orientation.x = 0.0;
		marker.pose.orientation.y = 0.0;
		marker.pose.orientation.z = 0.0;
		marker.pose.orientation.w = 1.0;
		marker.scale.x = 1;
		marker.scale.y = 1;
		marker.scale.z = 4;
		marker.color.a = 1.0;
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;

		marker_label = marker;
		marker_label.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker_label.action = visualization_msgs::Marker::ADD;
		marker_label.lifetime = ros::Duration(0);
		marker_label.scale.z = 1;
		marker_label.color.a = 1.0;
		marker_label.color.r = 1.0;
		marker_label.color.g = 1.0;
		marker_label.color.b = 1.0;


        for (int i = 0; i < signs.size(); ++i) {
        	if (signs[i].confidence < 0.7) continue;
        	marker.id = i;
            marker.pose.position.x = signs[i].x - x_ref_;
            marker.pose.position.y = signs[i].y - y_ref_;
            marker.pose.position.z = 3;

            marker_label.id = i;
            marker_label.pose.position.x = signs[i].x - x_ref_;
            marker_label.pose.position.y = signs[i].y - y_ref_;
            marker_label.pose.position.z = 6;
            marker_label.text = toString(signs[i].type);

            marker_arr.markers.push_back(marker);
            marker_label_arr.markers.push_back(marker_label);
        }

		if (signs.size() == 0) {
			marker.action = visualization_msgs::Marker::DELETEALL;
			marker_arr.markers.push_back(marker);

			marker_label.action = visualization_msgs::Marker::DELETEALL;
			marker_label_arr.markers.push_back(marker_label);
		}


		p_speedsigns_.publish(marker_arr);
		p_speedsigns_label_.publish(marker_label_arr);

        return;
    }

private:
	//params
	double downsample_ = 1.0;

	// flags
	bool b_first_odom_;

	// reference position
	double x_ref_ = 0.0;
	double y_ref_ = 0.0;
	double z_ref_ = 0.0;

	double x_cur_ = 0.0;
	double y_cur_ = 0.0;
	double z_cur_ = 0.0;

	size_t prev_obj_num_ = 0;

	ros::NodeHandle nh_;

	// subscribers
	ros::Subscriber s_mpc_ref_;
	ros::Subscriber s_mpc_pred_;
	ros::Subscriber s_path_max_vel_;
	ros::Subscriber s_desired_path_;
	ros::Subscriber s_odom_;
	ros::Subscriber s_tf_odom_;
	ros::Subscriber s_planner_roads_;
	ros::Subscriber s_planner_lattice_;
	ros::Subscriber s_planner_parking_lots_;
	ros::Subscriber s_planner_objects_;
	ros::Subscriber s_planner_lights_;
	ros::Subscriber s_detected_lights_;
	ros::Subscriber s_pointcloud_;
	ros::Subscriber s_obstacle_1_;
	//ros::Subscriber s_obstacle_2_;
	//ros::Subscriber s_obstacle_3_;
    ros::Subscriber s_trafficsigns_;
    ros::Subscriber s_speedsigns_;

    // publishers
	ros::Publisher p_mpc_ref_;
	ros::Publisher p_mpc_pred_;
	ros::Publisher p_path_max_vel_;
	ros::Publisher p_desired_path_;
	ros::Publisher p_tf_odom_;
	ros::Publisher p_planner_roads_;
	ros::Publisher p_lattice_;
	ros::Publisher p_planner_parking_lots_;
	ros::Publisher p_planner_objects_;
	ros::Publisher p_planner_lights_;
	ros::Publisher p_detected_lights_;
	ros::Publisher p_pointcloud_;
	ros::Publisher p_obstacle_1_;
	ros::Publisher p_obstacle_label_1_;
	ros::Publisher p_obstacle_predict_1_;
	ros::Publisher p_obstacle_2_;
	//ros::Publisher p_obstacle_3_;
    ros::Publisher p_trafficsigns_;
    ros::Publisher p_trafficsigns_label_;
    ros::Publisher p_speedsigns_;
    ros::Publisher p_speedsigns_label_;

    // tfs
	tf::TransformListener lr_;
	tf::TransformBroadcaster br_;
	tf::StampedTransform Tf_vel_imu_;
};

int main(int argc, char* argv[]) {

	ros::init(argc, argv, "revis_node");
	ros::NodeHandle nh("~");

	Revis revisualizer;
	revisualizer.init();

	// novatel_msgs::INSPVAX test;
	ros::spin();

	return 0;
}
