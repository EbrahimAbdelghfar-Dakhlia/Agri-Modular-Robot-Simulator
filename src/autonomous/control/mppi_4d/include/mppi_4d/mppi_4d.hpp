#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <vector>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "mppi_4d/param.hpp"
#include "mppi_4d/mppi_4d_core.hpp"
#include "mppi_eval_msgs/msg/mppi_eval.hpp"

namespace controller
{
    class MPPI : public rclcpp::Node // MPPI execution (dependent on ROS2)
    {
        public:
            MPPI();
            ~MPPI();
        private:
            // publisher
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cmd_absvel_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cmd_vx_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cmd_vy_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_cmd_omega_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_mppi_calc_time_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mppi_optimal_traj_;
            rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mppi_sampled_traj_;
            rclcpp::Publisher<mppi_eval_msgs::msg::MPPIEval>::SharedPtr pub_mppi_eval_msg_;

            // subscriber
            //// odometry (global vehicle pose)
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
            void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
            bool odom_received_;
            nav_msgs::msg::Odometry latest_odom_;
            common_type::XYYaw observed_state_;

            //// reference path
            rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_ref_path_;
            void refPathCallback(const nav_msgs::msg::Path::SharedPtr msg);
            bool ref_path_received_;
            nav_msgs::msg::Path latest_ref_path_;
            common_type::XYYaw goal_state_;

            //// collision costmap
            rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_collision_costmap_;
            void collisionCostmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
            bool collision_costmap_received_;
            grid_map::GridMap collision_costmap_;

            //// distance error map
            rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_distance_error_map_;
            void distanceErrorMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
            bool distance_error_map_received_;
            grid_map::GridMap distance_error_map_;

            //// reference yaw map
            rclcpp::Subscription<grid_map_msgs::msg::GridMap>::SharedPtr sub_ref_yaw_map_;
            void refYawMapCallback(const grid_map_msgs::msg::GridMap::SharedPtr msg);
            bool ref_yaw_map_received_;
            grid_map::GridMap ref_yaw_map_;

            // timer
            rclcpp::TimerBase::SharedPtr timer_control_interval_;
            void calcControlCommand();

            // rviz visualization
            void publishOptimalTrajectory(const std::vector<common_type::XYYaw>& optimal_xyyaw_sequence);
            void publishSampledTrajectories(const std::vector<std::vector<common_type::XYYaw>>& sampled_state_sequences);
            // mppi core instance
            MPPICore* mppi_core_;
    };
} // namespace controller