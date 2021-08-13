// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <array>
#include <string>
#include <vector>
#include <queue>
#include <atomic>

#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_msgs/Float64MultiArray.h>
#include <realtime_tools/realtime_publisher.h>
#include <std_msgs/String.h>


namespace franka_example_controllers {

    class JointPositionExampleController : public controller_interface::MultiInterfaceController<
            hardware_interface::PositionJointInterface> {
    public:
        bool init(hardware_interface::RobotHW *robot_hardware, ros::NodeHandle &node_handle) override;

        void starting(const ros::Time &) override;

        void update(const ros::Time &, const ros::Duration &period) override;


    private:
        hardware_interface::PositionJointInterface *position_joint_interface_;
        std::vector<hardware_interface::JointHandle> position_joint_handles_;
        ros::Duration elapsed_time_;
        std::array<double, 7> current_pose_{};

        ros::Subscriber command_sub_;
        std::shared_ptr<std::queue<double>> command_;
        bool trajectory_received_{false};
        realtime_tools::RealtimePublisher<std_msgs::String> execution_state_pub_;

        void setCommandCallback(const std_msgs::Float64MultiArrayConstPtr &msg);
    };

}  // namespace franka_example_controllers
