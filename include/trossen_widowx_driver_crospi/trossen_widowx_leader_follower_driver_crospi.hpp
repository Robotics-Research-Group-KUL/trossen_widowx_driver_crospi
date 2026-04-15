#pragma once

#include <string>


#include "robot_interfacing_utils/robotdriver.hpp"
#include "robot_interfacing_utils/controlmodes_enum.hpp"

#include "libtrossen_arm/trossen_arm.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
// include wrench message
#include "geometry_msgs/msg/wrench.hpp"

#include "trossen_widowx_interfaces/srv/joint_modes.hpp"
#include "trossen_widowx_interfaces/srv/control_gripper.hpp"
#include "trossen_widowx_interfaces/srv/arm_mode.hpp"

namespace etasl {

class trossen_widowx_leader_follower_driver_crospi : public RobotDriver {
    public:
        typedef std::shared_ptr<trossen_widowx_leader_follower_driver_crospi> SharedPtr;


    private:

        // The driver for the robot
        std::unique_ptr<trossen_arm::TrossenArmDriver> follower_driver_{nullptr};
        std::unique_ptr<trossen_arm::TrossenArmDriver> leader_driver_{nullptr};
        // IP address of the driver this hardware interface connects to
        std::string driver_ip_address_follower_;
        std::string driver_ip_address_leader_;

        
        double periodicity; // Periodicity of the driver
        double lead_gain;
        double safety_velocity_threshold;
        double safety_torque_threshold;
        bool move_to_home_position_on_startup;
        double force_feedback_gain;

        // std::vector<double> initial_joints_follower;
        // std::vector<double> initial_joints_leader;

        // std::vector<double> vel_setpoint_follower;
        unsigned int DOF;

        robotdrivers::DynamicJointDataField follower_setpoint_joint_vel_struct;
        robotdrivers::DynamicJointDataField follower_joint_pos_struct;
        robotdrivers::DynamicJointDataField follower_joint_vel_struct;

        // Robot output 
        trossen_arm::RobotOutput follower_output_;
        trossen_arm::RobotOutput leader_output_;


        std::shared_ptr<rclcpp::Node> ros_node_;
        // rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr gripper_sub_;
        std::thread ros_thread_;
        
        std::string control_mode;
        
        // Robot setpoints
        std::vector<double> follower_joint_position_setpoints_;
        std::vector<double> follower_joint_velocity_setpoints_;

        std::vector<double> amplified_follower_joint_velocity_setpoints_;


        // std::vector<double> leader_joint_position_setpoints_;
        // std::vector<double> leader_joint_velocity_setpoints_;
        std::vector<double> leader_joint_external_efforts_setpoints_;

        std::vector<double> leader_joint_external_efforts_feedback_;
        std::vector<double> follower_joint_external_efforts_feedback_;




        // ROS publishers
        // rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr follower_joint_state_pub_;
        // rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr wrench_pub_;

        // ROS services
        rclcpp::Service<trossen_widowx_interfaces::srv::ArmMode>::SharedPtr set_up_follower_mode_srv_;
        rclcpp::Service<trossen_widowx_interfaces::srv::ControlGripper>::SharedPtr control_gripper_srv_;

        // ROS messages
        // sensor_msgs::msg::JointState joint_state_msg;
        // geometry_msgs::msg::Wrench wrench_msg;

        // Time of the last loop
        // rclcpp::Time loop_time;

        // Mutex to lock update
        std::mutex update_mutex_;

        void set_up_arm_mode( 
            const std::shared_ptr<trossen_widowx_interfaces::srv::ArmMode::Request> request,
            std::shared_ptr<trossen_widowx_interfaces::srv::ArmMode::Response> response);

        void control_gripper( 
            const std::shared_ptr<trossen_widowx_interfaces::srv::ControlGripper::Request> request,
            std::shared_ptr<trossen_widowx_interfaces::srv::ControlGripper::Response> response);

    public:
        trossen_widowx_leader_follower_driver_crospi();

        virtual void construct(std::string robot_name,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker) override;

        /**
         * will only return true if it has received values for all the joints named in jnames.
        */
        virtual bool initialize() override;

        virtual void update(volatile std::atomic<bool>& stopFlag) override;

        virtual void on_configure() override;

        virtual void on_activate() override;

        virtual void on_deactivate() override;

        virtual void on_cleanup() override;

        virtual void finalize() override;

        // virtual const std::string& getName() const override;
        
        virtual ~trossen_widowx_leader_follower_driver_crospi();
};

} // namespace etasl
