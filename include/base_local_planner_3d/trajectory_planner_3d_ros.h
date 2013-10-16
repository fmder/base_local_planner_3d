
#include <hector_uav_msgs/Altimeter.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Range.h>
#include <base_local_planner/trajectory_planner_ros.h>

#ifndef __Trajectory_Planner_3d_ROS__
#define __Trajectory_Planner_3d_ROS__

namespace base_local_planner_3d {
  /**
   * @class TrajectoryPlanner3dROS
   * @brief
   */
  class TrajectoryPlanner3dROS : public base_local_planner::TrajectoryPlannerROS {
  public:
  	TrajectoryPlanner3dROS(){}
  	~TrajectoryPlanner3dROS(){}

    /**
     * @brief  Constructs the ros wrapper
     * @param name The name to give this instance of the trajectory planner
     * @param tf A pointer to a transform listener
     * @param costmap The cost map to use for assigning costs to trajectories
     */
    void initialize(std::string name, tf::TransformListener* tf,
        costmap_2d::Costmap2DROS* costmap_ros);

  	/**
     * @brief  Given the current position, orientation, and velocity of the robot,
   	 * compute velocity commands to send to the base
   	 * @param cmd_vel Will be filled with the velocity command to be passed to the robot base
   	 * @return True if a valid trajectory was found, false otherwise
   	 */
  	bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

    /**
     * @brief  Set the plan that the controller is following
     * @param orig_global_plan The plan to pass to the controller
     * @return True if the plan was updated successfully, false otherwise
     */
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

    /**
     * @brief  Check if the goal pose has been achieved
     * @return True if achieved, false otherwise
     */
    bool isGoalReached();

  protected:

    void sonarCB(const sensor_msgs::Range::ConstPtr& sonar, int direction);
    void altimeterCB(const hector_uav_msgs::Altimeter::ConstPtr& altimeter);
    void navCB(const nav_msgs::Odometry::ConstPtr& odom);

    ros::Subscriber alt_sub_;
    ros::Subscriber up_sonar_sub_;
    ros::Subscriber down_sonar_sub_;

    double sonar_max_range_;

    double free_space_up_;
    double free_space_down_;

    double up_safe_dist_;
    double down_safe_dist_;

    double desired_altitude_;
    double current_altitude_;

    double alt_tol_;
    double acc_lim_z_;
    double min_vel_z_;
    double max_vel_z_;

    double last_update_time_;
    double last_altitude_;
    double gain_p_, gain_i_, gain_d_;
    double p_, i_, d_;
  };
};

#endif