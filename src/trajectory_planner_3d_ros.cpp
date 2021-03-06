
#include <string>
#include <vector>

#include <boost/thread.hpp>
#include <boost/thread/locks.hpp>

#include <pluginlib/class_list_macros.h>
#include <base_local_planner_3d/trajectory_planner_3d_ros.h>

PLUGINLIB_EXPORT_CLASS(base_local_planner_3d::TrajectoryPlanner3dROS, nav_core::BaseLocalPlanner)

#define UP 0
#define DOWN 1

namespace base_local_planner_3d {

	void TrajectoryPlanner3dROS::initialize(std::string name, tf::TransformListener* tf,
        costmap_2d::Costmap2DROS* costmap_ros){
		base_local_planner::TrajectoryPlannerROS::initialize(name, tf, costmap_ros);

		p_ = i_ = d_ = 0.0;
		free_space_up_ = free_space_down_ = 0.0;

		ros::NodeHandle private_nh("~/" + name);
		std::string altimeter_topic_name;
		std::string up_sonar_topic_name;
		std::string down_sonar_topic_name;
		std::string nav_topic_name;
		private_nh.param("altimeter_topic_name", altimeter_topic_name, std::string("altimeter"));
		private_nh.param("up_sonar_topic_name", up_sonar_topic_name, std::string(""));
		private_nh.param("down_sonar_topic_name", down_sonar_topic_name, std::string(""));
		// private_nh.param("nav_topic_name", nav_topic_name, std::string("odom"));
		private_nh.param("altitude_tolerance", alt_tol_, 0.15);
		private_nh.param("up_safe_distance", up_safe_dist_, 0.15);
		private_nh.param("down_safe_distance", down_safe_dist_, 0.25);
		private_nh.param("sonar_max_range", sonar_max_range_, 3.0);
		private_nh.param("max_vel_z", max_vel_z_, 1.0);
		private_nh.param("proportional_gain", gain_p_, 1.0);
		private_nh.param("integral_gain", gain_i_, 0.0);
		private_nh.param("differential_gain", gain_d_, 0.0);

		ros::NodeHandle local_nh;
		alt_sub_ = local_nh.subscribe<hector_uav_msgs::Altimeter>(altimeter_topic_name.c_str(), 1, boost::bind(&TrajectoryPlanner3dROS::altimeterCB, this, _1));
		ROS_INFO("Local 3d trajectory planner registered to %s.", local_nh.resolveName(altimeter_topic_name).c_str());

		if(!up_sonar_topic_name.empty()){
			up_sonar_sub_ = local_nh.subscribe<sensor_msgs::Range>(up_sonar_topic_name.c_str(), 1, boost::bind(&TrajectoryPlanner3dROS::sonarCB, this, _1, UP));
			ROS_INFO("Local 3d trajectory planner registered to %s.", local_nh.resolveName(up_sonar_topic_name).c_str());
		}else{
			free_space_up_ = std::numeric_limits<double>::max();
		}

		if(!down_sonar_topic_name.empty()){
			down_sonar_sub_ = local_nh.subscribe<sensor_msgs::Range>(down_sonar_topic_name.c_str(), 1, boost::bind(&TrajectoryPlanner3dROS::sonarCB, this, _1, DOWN));
			ROS_INFO("Local 3d trajectory planner registered to %s.", local_nh.resolveName(down_sonar_topic_name).c_str());
		}else{
			free_space_up_ = std::numeric_limits<double>::max();
		}
	}

	void TrajectoryPlanner3dROS::sonarCB(const sensor_msgs::Range::ConstPtr& sonar, int direction){
		if(direction == UP)
			free_space_up_ = sonar->range;
		else if(direction == DOWN)
			free_space_down_ = sonar->range;
	}

	void TrajectoryPlanner3dROS::altimeterCB(const hector_uav_msgs::Altimeter::ConstPtr& altimeter){
		current_altitude_ = altimeter->altitude;
	}

	void TrajectoryPlanner3dROS::navCB(const nav_msgs::Odometry::ConstPtr& odom){
		current_altitude_ = odom->pose.pose.position.z;
	}

	bool TrajectoryPlanner3dROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
		bool state = base_local_planner::TrajectoryPlannerROS::computeVelocityCommands(cmd_vel);
		double current_time = ros::Time::now().toSec();
		double dt = current_time - last_update_time_;
		last_update_time_ = current_time;

		p_ = desired_altitude_ - current_altitude_;
		p_ = std::min(p_, free_space_up_ - up_safe_dist_);
		p_ = std::max(p_, -free_space_down_ + down_safe_dist_);

		// ROS_ERROR("dt: %f", dt);
		
		if(dt > 0){
			i_ += p_ * dt;
			d_  = p_ / dt;
		}else{
			i_ = 0;
			d_ = 0;
		}
		
		// int sign = 0;
		// if(cmd_vel.linear.z >= 0)
		// 	sign = 1;
		// else if(cmd_vel.linear.z < 0)
		// 	sign = -1;
		
		cmd_vel.linear.z = gain_p_ * p_ + gain_i_ * i_ + gain_d_ * d_;
		// ROS_ERROR("CMD_VEL Z1: %f", cmd_vel.linear.z);
		cmd_vel.linear.z = cmd_vel.linear.z <  max_vel_z_ ? cmd_vel.linear.z :  max_vel_z_;
		// ROS_ERROR("CMD_VEL Z2: %f", cmd_vel.linear.z);
		cmd_vel.linear.z = cmd_vel.linear.z > -max_vel_z_ ? cmd_vel.linear.z : -max_vel_z_;
		// ROS_ERROR("CMD_VEL Z3: %f", cmd_vel.linear.z);

		last_altitude_ = current_altitude_;
		return state;
	}


	bool TrajectoryPlanner3dROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
		if(!orig_global_plan.empty())
			desired_altitude_ = orig_global_plan.back().pose.position.z;

		// ROS_ERROR("Desired Altitude: %f", desired_altitude_);
		// ROS_ERROR("Current Altitude: %f", current_altitude_);

		p_ = i_ = d_ = 0.0;
		last_update_time_ = ros::Time::now().toSec();
		
		return base_local_planner::TrajectoryPlannerROS::setPlan(orig_global_plan);
	}

	bool TrajectoryPlanner3dROS::isGoalReached(){
		bool position_reached = base_local_planner::TrajectoryPlannerROS::isGoalReached();
		double diff_alt = desired_altitude_ - current_altitude_;
		bool altitude_reached = fabs(diff_alt) < alt_tol_;
		
		if(position_reached && !altitude_reached){
			// Check if we are limited in space
			if(free_space_up_ < sonar_max_range_ - 0.1 || free_space_down_ < sonar_max_range_ - 0.1){
				if(diff_alt > (free_space_up_ - up_safe_dist_) || diff_alt < (-free_space_down_ + down_safe_dist_)){
					if(diff_alt > 0)
						desired_altitude_ = current_altitude_ + free_space_up_ - up_safe_dist_;
					else
						desired_altitude_ = current_altitude_ - free_space_down_ + down_safe_dist_;
					ROS_WARN("Cannot reach desired altitude because of obstacle.");
					ROS_WARN("Setting desired altitude to %f", desired_altitude_);
				}
			}
		}

		return altitude_reached && position_reached;
	}
}