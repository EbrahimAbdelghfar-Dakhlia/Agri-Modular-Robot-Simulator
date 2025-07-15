# pragma once

#include <ros/ros.h>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <vector>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <rviz_2d_overlay_msgs/msg/overlay_text.hpp>
#include "mode1_mppi_3d/param.hpp"
#include "mode1_mppi_3d/mppi_3d_core.hpp"

namespace controller_mppi_3d
{
    class MPPI // MPPI execution (dependent on ROS)
    {
        public:
            MPPI();
            ~MPPI();
        private:
            ros::NodeHandle nh_;
            ros::NodeHandle private_nh_;

            // publisher
            ros::Publisher pub_cmd_vel_;
            ros::Publisher pub_cmd_absvel_;
            ros::Publisher pub_cmd_vx_;
            ros::Publisher pub_cmd_vy_;
            ros::Publisher pub_cmd_omega_;
            ros::Publisher pub_mppi_calc_time_;
            ros::Publisher pub_mppi_optimal_traj_;
            ros::Publisher pub_mppi_sampled_traj_;
            ros::Publisher pub_mppi_overlay_text_;
            ros::Publisher pub_mppi_eval_msg_;

            // subscriber
            //// odometry (global vehicle pose)
            ros::Subscriber sub_odom_;
            void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
            bool odom_received_;
            nav_msgs::Odometry latest_odom_;
            common_type::XYYaw observed_state_;

            //// reference path
            ros::Subscriber sub_ref_path_;
            void refPathCallback(const nav_msgs::Path::ConstPtr& msg);
            bool ref_path_received_;
            nav_msgs::Path latest_ref_path_;
            common_type::XYYaw goal_state_;

            //// collision costmap
            ros::Subscriber sub_collision_costmap_;
            void collisionCostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
            bool collision_costmap_received_;
            grid_map::GridMap collision_costmap_;

            //// distance error map
            ros::Subscriber sub_distance_error_map_;
            void distanceErrorMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
            bool distance_error_map_received_;
            grid_map::GridMap distance_error_map_;

            //// reference yaw map
            ros::Subscriber sub_ref_yaw_map_;
            void refYawMapCallback(const grid_map_msgs::GridMap::ConstPtr& msg);
            bool ref_yaw_map_received_;
            grid_map::GridMap ref_yaw_map_;

            // timer
            ros::Timer timer_control_interval_;
            void calcControlCommand(const ros::TimerEvent& event);

            // rviz visualization
            void publishOptimalTrajectory(const std::vector<common_type::XYYaw>& optimal_xyyaw_sequence);
            void publishSampledTrajectories(const std::vector<std::vector<common_type::XYYaw>>& sampled_state_sequences);
            void publishOverlayText(const std::string& text);

            // mppi core instance
            MPPI3DCore* mppi_core_;
    };
} // namespace controller_mppi_3d