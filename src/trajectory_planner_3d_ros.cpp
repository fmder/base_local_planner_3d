
#include <string>
#include <vector>

#include <boost/thread.hpp>

#include <pluginlib/class_list_macros.h>
#include <base_local_planner_3d/trajectory_planner_3d_ros.h>

PLUGINLIB_EXPORT_CLASS(base_local_planner_3d::TrajectoryPlanner3dROS, nav_core::BaseLocalPlanner)

namespace base_local_planner_3d {

	void TrajectoryPlanner3dROS::initialize(std::string name, tf::TransformListener* tf,
        costmap_2d::Costmap2DROS* costmap_ros){
		base_local_planner::TrajectoryPlannerROS::initialize(name, tf, costmap_ros);

		p_ = i_ = d_ = 0.0;

		ros::NodeHandle private_nh("~/" + name);
		std::string altimeter_topic_name;
		std::string nav_topic_name;
		private_nh.param("altimeter_topic_name", altimeter_topic_name, std::string("altimeter"));
		// private_nh.param("nav_topic_name", nav_topic_name, std::string("odom"));
		private_nh.param("altitude_tolerance", alt_tol_, 0.15);
		private_nh.param("max_vel_z", max_vel_z_, 1.0);
		private_nh.param("proportional_gain", gain_p_, 1.0);
		private_nh.param("integral_gain", gain_i_, 0.0);
		private_nh.param("differential_gain", gain_d_, 0.0);

		ros::NodeHandle local_nh;
		alt_sub_ = local_nh.subscribe<hector_uav_msgs::Altimeter>(altimeter_topic_name.c_str(), 1, boost::bind(&TrajectoryPlanner3dROS::altimeterCB, this, _1));
		ROS_INFO("Local 3d trajectory planner registered to %s.\n", local_nh.resolveName(altimeter_topic_name).c_str());
	}

	void TrajectoryPlanner3dROS::altimeterCB(const hector_uav_msgs::Altimeter::ConstPtr& altimeter){
		current_altitude_ = altimeter->altitude;
	}

	void TrajectoryPlanner3dROS::navCB(const nav_msgs::Odometry::ConstPtr& odom){
		current_altitude_ = odom->pose.pose.position.z;
	}

	bool TrajectoryPlanner3dROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel){
		bool state = base_local_planner::TrajectoryPlannerROS::computeVelocityCommands(cmd_vel);
		double dt = ros::Time::now().toSec() - last_update_time_;
		
		p_  = desired_altitude_ - current_altitude_;
		i_ += p_ * dt;
		d_  = p_ / dt;
		
		cmd_vel.linear.z = gain_p_ * p_ + gain_i_ * i_ + gain_d_ * d_;
		cmd_vel.linear.z = cmd_vel.linear.z <  max_vel_z_ ? cmd_vel.linear.z :  max_vel_z_;
		cmd_vel.linear.z = cmd_vel.linear.z > -max_vel_z_ ? cmd_vel.linear.z : -max_vel_z_;

		last_altitude_ = current_altitude_;
		return state;
	}


	bool TrajectoryPlanner3dROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan){
		if(!orig_global_plan.empty())
			desired_altitude_ = orig_global_plan.back().pose.position.z;

		p_ = i_ = d_ = 0.0;
		
		return base_local_planner::TrajectoryPlannerROS::setPlan(orig_global_plan);
	}

	bool TrajectoryPlanner3dROS::isGoalReached(){
		// if(fabs(current_altitude_ - desired_altitude_) < alt_tol_)
		// 	ROS_INFO("Desired altitude reached\n");

		return fabs(current_altitude_ - desired_altitude_) < alt_tol_ && 
			   	 base_local_planner::TrajectoryPlannerROS::isGoalReached();
	}
}