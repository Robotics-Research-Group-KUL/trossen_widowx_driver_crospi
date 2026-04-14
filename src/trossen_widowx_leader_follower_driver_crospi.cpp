#include "trossen_widowx_driver_crospi/trossen_widowx_leader_follower_driver_crospi.hpp"
#include <fmt/format.h>
#include <iostream>

// includ library for sleep
#include <chrono>
#include <thread>

namespace etasl {


trossen_widowx_leader_follower_driver_crospi::trossen_widowx_leader_follower_driver_crospi()
{
}

void trossen_widowx_leader_follower_driver_crospi::construct(std::string robot_name,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker)
{

    driver_ip_address_ = jsonchecker->asString(config, "ip_address");
    periodicity = jsonchecker->asDouble(config, "periodicity");
    control_mode = jsonchecker->asString(config, "control_mode");
    lead_gain = jsonchecker->asDouble(config, "lead_gain");
    safety_velocity_threshold = jsonchecker->asDouble(config, "safety_velocity_threshold");


    std::cout << "lead gain --------------------------------------- " << lead_gain << std::endl;



    name = robot_name; //defined in RobotDriver super class.
    std::cout << "Constructed object of trossen_widowx_leader_follower_driver_crospi class with name: " << name << std::endl;

    DOF = 6;

    AvailableFeedback available_fb{};
    available_fb.joint_pos = true;
    available_fb.joint_vel = true;

    constructPorts(DOF, available_fb); //Constructs all shared pointers and initialize data structures. Call after assigning available_feedback booleans.

    //Initialize vectors
    // initial_joints.resize(DOF, 0.0); 
    arm_joint_velocity_setpoints_.resize(DOF, 0.0);
    arm_joint_position_setpoints_.resize(DOF, 0.0);
    vel_setpoint.resize(DOF, 0.0);

    // Initialize structure data
    setpoint_joint_vel_struct.data.resize(DOF, 0.0); //resize and initialize setpoint joint velocities to zero
    joint_pos_struct.data.resize(DOF, 0.0);
    joint_vel_struct.data.resize(DOF, 0.0);

}

bool trossen_widowx_leader_follower_driver_crospi::initialize()
{
    arm_driver_ = std::make_unique<trossen_arm::TrossenArmDriver>();

    try
    {
        arm_driver_->configure(
                trossen_arm::Model::wxai_v0,
                trossen_arm::StandardEndEffector::wxai_v0_follower,
                driver_ip_address_.c_str(),
                true,
                5.0); // 5 seconds timeout for the connection
    }
    catch(const std::exception& e)
    {
        std::cerr << "Failed to configure arm driver: " << e.what() << '\n';
    }
    
    if (!arm_driver_->get_is_configured()) {
        std::cerr << "get_is_configured failed" << std::endl;
        return false;
    }

    // Update the robot output
    robot_output_ = arm_driver_->get_robot_output();
    arm_joint_position_setpoints_ = robot_output_.joint.arm.positions;
    
    assert(robot_output_.joint.arm.positions.size() == DOF);
    std::copy(robot_output_.joint.arm.positions.begin(), robot_output_.joint.arm.positions.end(), joint_pos_struct.data.begin());
    writeFeedbackJointPosition(joint_pos_struct);

    assert(robot_output_.joint.arm.velocities.size() == DOF);
    std::copy(robot_output_.joint.arm.velocities.begin(), robot_output_.joint.arm.velocities.end(), joint_vel_struct.data.begin());
    writeFeedbackJointVelocity(joint_vel_struct);

    
    // Set the arm joints to position mode
    if (control_mode == "JOINT_POSITION") {
        arm_driver_->set_arm_modes(trossen_arm::Mode::position);
        std::cout << "Control mode set to JOINT_POSITION" << std::endl;
    } 
    else if (control_mode == "JOINT_POSITION_WITH_FEEDBACK") {
        arm_driver_->set_arm_modes(trossen_arm::Mode::position);
        std::cout << "Control mode set to JOINT_POSITION_WITH_FEEDBACK" << std::endl;
    } 
    else if (control_mode == "JOINT_VELOCITY") {
        arm_driver_->set_arm_modes(trossen_arm::Mode::velocity);
        std::cout << "Control mode set to JOINT_VELOCITY" << std::endl;
    } 
    else {
        std::cerr << "Unknown control mode: " << control_mode << std::endl;
        return false;
    }

    arm_driver_->set_gripper_mode(trossen_arm::Mode::position);

    arm_driver_->set_gripper_position(0.04, 2.0, true); // Set the gripper to open position

    std::cout << fmt::format("Trossen Arm Driver initialized with IP address: {}", driver_ip_address_) << std::endl;

    // Create ROS2 node
    ros_node_ = std::make_shared<rclcpp::Node>("widowX_gripper_plugin_node");
    // Create ROS2 services
    control_gripper_srv_ = ros_node_->create_service<trossen_widowx_interfaces::srv::ControlGripper>(
        "widowX/control_gripper",
        std::bind(&trossen_widowx_leader_follower_driver_crospi::control_gripper, this, std::placeholders::_1, std::placeholders::_2));

    set_up_arm_mode_srv_ = ros_node_->create_service<trossen_widowx_interfaces::srv::ArmMode>(
    "widowX/set_up_arm_mode",
    std::bind(&trossen_widowx_leader_follower_driver_crospi::set_up_arm_mode, this, std::placeholders::_1, std::placeholders::_2));

    // Start ROS2 thread
    ros_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(ros_node_);
        exec.spin();
    });

    

    arm_driver_->set_arm_positions(robot_output_.joint.arm.positions,
                            1.0f, // goal_time
                            true); // Blocking, to let the robot hold the initial position before starting the control loop, since it drops a bit in the beginning.
    // sleep this thread
    // std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "Initialization complete" << std::endl;

    return true;
}


void trossen_widowx_leader_follower_driver_crospi::update(volatile std::atomic<bool>& stopFlag)
{   
    update_mutex_.lock();

    readSetpointJointVelocity(setpoint_joint_vel_struct);
    std::copy(setpoint_joint_vel_struct.data.begin(), setpoint_joint_vel_struct.data.end(), arm_joint_velocity_setpoints_.begin()); // arm_joint_velocity_setpoints_ = setpoint_joint_vel_struct.data;


    robot_output_ = arm_driver_->get_robot_output();

    std::copy(robot_output_.joint.arm.positions.begin(), robot_output_.joint.arm.positions.end(), joint_pos_struct.data.begin());
    writeFeedbackJointPosition(joint_pos_struct);

    std::copy(robot_output_.joint.arm.velocities.begin(), robot_output_.joint.arm.velocities.end(), joint_vel_struct.data.begin());
    writeFeedbackJointVelocity(joint_vel_struct);

    // TODO: Check the load of this loop
    if (std::any_of(
      arm_joint_velocity_setpoints_.begin(), arm_joint_velocity_setpoints_.end(),
      [this](double velocity) {
        return std::isnan(velocity) || std::isinf(velocity);
      }))
    {
        //  break the loop if any velocity is NaN or INF
        arm_joint_velocity_setpoints_.clear();
        arm_joint_velocity_setpoints_.resize(6, 0.0); // Reset to zero
        stopFlag.store(true);
        std::cerr << "Invalid joint velocity command detected" << std::endl;
        return; // Exit the update function
    }

    if (control_mode == "JOINT_POSITION"){
        for (unsigned int i=0; i<arm_joint_position_setpoints_.size(); ++i) {
            if(abs(robot_output_.joint.arm.velocities[i]) > safety_velocity_threshold){
                stopFlag.store(true);
                std::cerr << "Velocity of joint " << i+1 << " is too high: " << robot_output_.joint.arm.velocities[i] << std::endl;
                return; // Exit the update function
            }
            arm_joint_position_setpoints_[i] += arm_joint_velocity_setpoints_[i]*periodicity; //simple integration

        }
        arm_driver_->set_arm_positions(arm_joint_position_setpoints_,
                                    periodicity, // goal_time
                                    false, // blocking is false, so it will not wait for the goal to be reached
                                    arm_joint_velocity_setpoints_); // feedforward velocities
    }
    else if (control_mode == "JOINT_POSITION_WITH_FEEDBACK"){
        for (unsigned int i=0; i<arm_joint_position_setpoints_.size(); ++i) {
            if(abs(robot_output_.joint.arm.velocities[i]) > safety_velocity_threshold){
                stopFlag.store(true);
                std::cerr << "Velocity of joint " << i+1 << " is too high: " << robot_output_.joint.arm.velocities[i] << std::endl;
                return; // Exit the update function
            }
            // arm_joint_position_setpoints_[i] += arm_joint_velocity_setpoints_[i]*periodicity; //simple integration
            // arm_joint_position_setpoints_[i] = robot_output_.joint.arm.positions[i] + arm_joint_velocity_setpoints_[i]*periodicity*lead_gain; //simple integration
            arm_joint_position_setpoints_[i] = robot_output_.joint.arm.positions[i]; //I think this does not work because the robot keeps falling and falling due to gravity and low gains of the position loop.



        }
        arm_driver_->set_arm_positions(arm_joint_position_setpoints_,
                                    periodicity, // goal_time
                                    false); // blocking is false, so it will not wait for the goal to be reached
    }
    else if (control_mode == "JOINT_VELOCITY") {
        arm_driver_->set_arm_velocities(arm_joint_velocity_setpoints_,
                                    periodicity, // goal_time
                                    false); // blocking is false, so it will not wait for the goal to be reached
    }



    update_mutex_.unlock();
}

void trossen_widowx_leader_follower_driver_crospi::control_gripper( const std::shared_ptr<trossen_widowx_interfaces::srv::ControlGripper::Request> request,
                                                std::shared_ptr<trossen_widowx_interfaces::srv::ControlGripper::Response> response ) {

    arm_driver_->set_gripper_position(request->position, request->time, true); // Set the gripper position blocking
    response->success = true;

}


void trossen_widowx_leader_follower_driver_crospi::set_up_arm_mode( const std::shared_ptr<trossen_widowx_interfaces::srv::ArmMode::Request> request,
                                                std::shared_ptr<trossen_widowx_interfaces::srv::ArmMode::Response> response ) {
    
    // lock update hook
    update_mutex_.lock();
    // Set control mode
    control_mode = request->arm_mode;
    
    arm_joint_position_setpoints_ = arm_driver_->get_arm_positions(); // Initialize the position setpoints with the current positions
    std::copy(arm_joint_position_setpoints_.begin(), arm_joint_position_setpoints_.end(), joint_pos_struct.data.begin());

    // Set the arm joints to position mode
    if (control_mode == "JOINT_POSITION") {
        arm_driver_->set_arm_modes(trossen_arm::Mode::position);
        // logger
        RCLCPP_INFO(ros_node_->get_logger(), "Control mode set to JOINT_POSITION");
        // Log arm joint position setpoints
        // for (size_t i = 0; i < arm_joint_position_setpoints_.size(); ++i) {
        //     RCLCPP_INFO(ros_node_->get_logger(), "Joint %zu position setpoint: %f", i, arm_joint_position_setpoints_[i]);
        // }
        
        response->success = true;
    } 
    else if (control_mode == "JOINT_POSITION_WITH_FEEDBACK") {
        arm_driver_->set_arm_modes(trossen_arm::Mode::position);
        // logger
        RCLCPP_INFO(ros_node_->get_logger(), "Control mode set to JOINT_POSITION_WITH_FEEDBACK");
        // Log arm joint position setpoints
        // for (size_t i = 0; i < arm_joint_position_setpoints_.size(); ++i) {
        //     RCLCPP_INFO(ros_node_->get_logger(), "Joint %zu position setpoint: %f", i, arm_joint_position_setpoints_[i]);
        // }
        
        response->success = true;
    } 
    else if (control_mode == "JOINT_VELOCITY") {
        arm_driver_->set_arm_modes(trossen_arm::Mode::velocity);
        RCLCPP_INFO(ros_node_->get_logger(), "Control mode set to JOINT_VELOCITY");
        response->success = true;
    } 
    else {
        RCLCPP_ERROR(ros_node_->get_logger(), "Unknown control mode: %s", control_mode.c_str());
        response->success = false;
    }

    update_mutex_.unlock();

}

void trossen_widowx_leader_follower_driver_crospi::on_configure() {
    // std::cout << "entering on configure =======================" << std::endl;

}

void trossen_widowx_leader_follower_driver_crospi::on_activate() 
{


}

void trossen_widowx_leader_follower_driver_crospi::on_deactivate() {
    // std::cout << "entering on deactivate =======================" << std::endl;

}

void trossen_widowx_leader_follower_driver_crospi::on_cleanup() {
    // std::cout << "entering on cleanup =======================" << std::endl;

}


void trossen_widowx_leader_follower_driver_crospi::finalize() {
    std::cout << "finalize() called =======================" << std::endl;
    arm_driver_->set_all_modes(trossen_arm::Mode::idle);
    // Destroy driver object
    arm_driver_.reset();
    // Stop ROS 2 node
    // TODO: Check if this is the right way to stop the node
    if (ros_thread_.joinable()) {
        // Signal ROS to shutdown before joining the thread
        rclcpp::shutdown();
        ros_thread_.join();
    }

}

trossen_widowx_leader_follower_driver_crospi::~trossen_widowx_leader_follower_driver_crospi() {

};



} // namespace etasl


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(etasl::trossen_widowx_leader_follower_driver_crospi, etasl::RobotDriver)
