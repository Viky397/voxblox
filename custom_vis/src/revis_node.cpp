#include <math.h>
#include <iostream>
#include <fstream>

#include <eigen_conversions/eigen_msg.h>

#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>

#include <zeus_msgs/Detections3D.h>

class Revis {
	typedef pcl::PointXYZI PointTypeI;
	typedef pcl::PointCloud<PointTypeI> PointCloudI;
public:

	Revis() :
			nh_(ros::NodeHandle("~")) {
		b_first_odom_ = false;
	}

	void init() {
		// parameters
		nh_.param<double>("downsample", downsample_, 0.5);
		if (downsample_ < 1e-3)
			downsample_ = 0;

		// subscribers
		s_tracked_objects_ = nh_.subscribe("/kimera_semantics_node/Objects/TrackedObjects",10,&Revis::TrackedObjectsCallback, this);
		s_observations_ = nh_.subscribe("/kimera_semantics_node/Objects/Observations",10,&Revis::ObservationsCallback, this);

        // publishers
		p_tracked_objects_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"tracked_objects", 1);
		p_tracked_objects_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
				"tracked_objects_cloud", 1);
		p_tracked_objects_label_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"tracked_objects_label", 1);
		p_tracked_objects_predict_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"tracked_objects_predict", 1);

		p_observations_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"observations", 1);
		p_observations_cloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >(
				"observations_cloud", 1);
		/*
		p_observations_label_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"observations_label", 1);
		p_observations_predict_ = nh_.advertise<visualization_msgs::MarkerArray>(
				"observations_predict", 1);
		*/
    }

	void ProcessDetections(
			std::vector<zeus_msgs::BoundingBox3D>& bbs,
			visualization_msgs::MarkerArray& msg_new,
			visualization_msgs::MarkerArray& msg_label_new,
			visualization_msgs::MarkerArray& msg_predict_new,
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr& objects,
			bool is_obs) {

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
		obstacle_marker.color.a = 0.4;
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
		obstacle_label.scale.z = 0.15;

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
				str_type = "Low Dynamic";
			else if (bbs[i].type == 0)
				str_type = "High Dynamic";

			double speed = sqrt(pow(bbs[i].x_dot, 2) + pow(bbs[i].y_dot, 2));

			if (str_type == "Low Dynamic"){
				obstacle_marker.color.r = 0.0;
				obstacle_marker.color.g = 0.75;
				obstacle_marker.color.b = 0.0;
			} else if (str_type == "High Dynamic") {
				obstacle_marker.color.r = 0.8;
				obstacle_marker.color.g = 0.0;
				obstacle_marker.color.b = 0.0;
			}

	        double x = bbs[i].x;
	        double y = bbs[i].y;
	        double z = bbs[i].z;

	        double cy = cos(bbs[i].yaw * 0.5);
	        double sy = sin(bbs[i].yaw * 0.5);
	        double cp = cos(0 * 0.5);
	        double sp = sin(0 * 0.5);
	        double cr = cos(0 * 0.5);
	        double sr = sin(0 * 0.5);

			obstacle_marker.pose.position.x = x;
			obstacle_marker.pose.position.y = y;
			obstacle_marker.pose.position.z = z;
			obstacle_marker.scale.x = bbs[i].w;
			obstacle_marker.scale.y = bbs[i].l;
			obstacle_marker.scale.z = bbs[i].h;
            obstacle_marker.pose.orientation.w = cr * cp * sy - sr * sp * cy;
            obstacle_marker.pose.orientation.x = sr * cp * cy - cr * sp * sy;
            obstacle_marker.pose.orientation.y = cr * sp * cy + sr * cp * sy;
            obstacle_marker.pose.orientation.z = cr * cp * cy + sr * sp * sy;
            obstacle_marker.id++;

            if (bbs[i].confidence > 0.0) {
            	msg_new.markers.push_back(obstacle_marker);

    			obstacle_label.pose.position.x = x;
				obstacle_label.pose.position.y = y;
				obstacle_label.pose.position.z = z + bbs[i].h/2 + 0.5;
				obstacle_label.pose.orientation = obstacle_marker.pose.orientation;
				std::string str_conf = std::to_string(
						bbs[i].confidence * 100.0);
				obstacle_label.text = std::string("Obj ID: ")
						+ std::to_string(bbs[i].ID) + std::string("\nConf: ")
						+ str_conf.substr(0, str_conf.find(".") + 3);

				msg_label_new.markers.push_back(obstacle_label);
				obstacle_label.id++;

				/*
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
				 */
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr obj_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
				pcl::fromROSMsg(bbs[i].cloud, *obj_cloud);
				*objects += *obj_cloud;
            }
		}


	}

	void TrackedObjectsCallback(const zeus_msgs::Detections3DConstPtr& msg) {
		std::vector<zeus_msgs::BoundingBox3D> bbs = msg->bbs;

		visualization_msgs::MarkerArray msg_new;
		visualization_msgs::MarkerArray msg_label_new;
		visualization_msgs::MarkerArray msg_predict_new;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);

		ProcessDetections(bbs, msg_new, msg_label_new, msg_predict_new, objects, false);
		pcl_conversions::toPCL(msg->header, objects->header);

		p_tracked_objects_.publish(msg_new);
		p_tracked_objects_cloud_.publish(objects);
		p_tracked_objects_label_.publish(msg_label_new);
		p_tracked_objects_predict_.publish(msg_predict_new);
	}

	void ObservationsCallback(const zeus_msgs::Detections3DConstPtr& msg) {
		std::vector<zeus_msgs::BoundingBox3D> bbs = msg->bbs;

		visualization_msgs::MarkerArray msg_new;
		visualization_msgs::MarkerArray msg_label_new;
		visualization_msgs::MarkerArray msg_predict_new;
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr objects(new pcl::PointCloud<pcl::PointXYZRGB>);

		ProcessDetections(bbs, msg_new, msg_label_new, msg_predict_new, objects, true);
		pcl_conversions::toPCL(msg->header, objects->header);

		p_observations_.publish(msg_new);
		p_observations_cloud_.publish(objects);
		// p_observations_label_.publish(msg_label_new);
		// p_observations_predict_.publish(msg_predict_new);
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
	ros::Subscriber s_tracked_objects_;
	ros::Subscriber s_observations_;

    // publishers
	ros::Publisher p_tracked_objects_;
	ros::Publisher p_tracked_objects_cloud_;
	ros::Publisher p_tracked_objects_label_;
	ros::Publisher p_tracked_objects_predict_;

	ros::Publisher p_observations_;
	ros::Publisher p_observations_cloud_;
	//ros::Publisher p_observations_label_;
	//ros::Publisher p_observations_predict_;

};

int main(int argc, char* argv[]) {

	ros::init(argc, argv, "revis_node");
	ros::NodeHandle nh("~");

	Revis revisualizer;
	revisualizer.init();

	ros::spin();

	return 0;
}
