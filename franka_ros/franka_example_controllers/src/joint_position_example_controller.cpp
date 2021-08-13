// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_position_example_controller.h>

#include <cmath>
#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

namespace franka_example_controllers {

    bool JointPositionExampleController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
        constexpr size_t kNumberOfJoints = 7;
        position_joint_interface_ = robot_hardware->get<hardware_interface::PositionJointInterface>();
        if (position_joint_interface_ == nullptr) {
            ROS_ERROR(
                    "JointPositionExampleController: Error getting position joint interface from hardware!");
            return false;
        }
        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names)) {
            ROS_ERROR("JointPositionExampleController: Could not parse joint names");
        }
        if (joint_names.size() != kNumberOfJoints) {
            ROS_ERROR_STREAM("JointPositionExampleController: Wrong number of joint names, got "
                                     << joint_names.size() << " instead of " << kNumberOfJoints << " names!");
            return false;
        }
        position_joint_handles_.resize(kNumberOfJoints);
        for (size_t i = 0; i < kNumberOfJoints; ++i) {
            try {
                position_joint_handles_[i] = position_joint_interface_->getHandle(joint_names[i]);
            } catch (const hardware_interface::HardwareInterfaceException& e) {
                ROS_ERROR_STREAM(
                        "JointPositionExampleController: Exception getting joint handles: " << e.what());
                return false;
            }
        }

        command_ = std::make_shared<std::queue<double>>();

        command_sub_ = node_handle.subscribe<std_msgs::Float64MultiArray>(std::string("joint_command"), 1,
                                                                          &JointPositionExampleController::setCommandCallback, this,ros::TransportHints().tcpNoDelay());
        
        execution_state_pub_.init(node_handle, "/execution_state", 1);
        return true;
    }

    void JointPositionExampleController::starting(const ros::Time& /* time */) {
        for (size_t i = 0; i < position_joint_handles_.size(); ++i) {
            current_pose_[i] = position_joint_handles_[i].getPosition();
            command_->push(current_pose_[i]);
        }
        elapsed_time_ = ros::Duration(0.0);
    }

    void JointPositionExampleController::update(const ros::Time&, const ros::Duration& period) {
        elapsed_time_ += period;
        auto current_command = std::atomic_load(&command_);
        bool commands_empty = current_command->empty();
        if(commands_empty) {
            for(auto& angle : current_pose_) {
                current_command->push(angle);
            }
        } else {
            trajectory_received_ = true;
        }
        for (size_t i = 0; i < position_joint_handles_.size(); i++) {
            position_joint_handles_[i].setCommand(current_command->front());
            current_pose_[i] = current_command->front();
            current_command->pop();
        }
        if(trajectory_received_ && commands_empty) {
            ROS_INFO("Trajectory done");
            trajectory_received_ = false;
            execution_state_pub_.msg_.data = "reached";
            execution_state_pub_.unlockAndPublish();
        }
    }

    void JointPositionExampleController::setCommandCallback(const std_msgs::Float64MultiArrayConstPtr& msg) {
        auto tmp = std::make_shared<std::queue<double>>();
        if(msg->data.size() % position_joint_handles_.size() != 0) {
            ROS_ERROR("Could not set command. Did not receive multiple of %zu angles (%zu)",
                      position_joint_handles_.size(),
                      msg->data.size());
        }
        for(const auto& angle : msg->data) {
            tmp->push(angle);
        }
        std::atomic_store(&command_, tmp);
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointPositionExampleController,
                       controller_interface::ControllerBase)