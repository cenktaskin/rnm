// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>
#include <queue>
#include <mutex>
#include <atomic>

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/String.h>

namespace franka_example_controllers {

    class JointPositionExampleControllerSim
            : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
    public:
        bool init(hardware_interface::PositionJointInterface *hw, ros::NodeHandle &n) override;

        void starting(const ros::Time &) override;

        void update(const ros::Time &, const ros::Duration &period) override;

    private:
        hardware_interface::PositionJointInterface *position_joint_interface_;
        std::vector<hardware_interface::JointHandle> position_joint_handles_;
        ros::Duration elapsed_time_;
        std::array<double, 7> current_pose_{};

        ros::Subscriber command_sub_;
        std::shared_ptr<std::queue<double>> command_;

        void setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
        bool trajectory_received_{false};
        bool validateJointSpeed(double previousCommand, double commanded) const;
        realtime_tools::RealtimePublisher<std_msgs::String> execution_state_pub_;

        const double max_joint_speed = 1.5;
        ros::Time last_command_time_;
        double last_diff_ = 0;
        std::mutex check_mutex_;
    };

}  // namespace franka_example_controllers
