#include "trossen_widowx_driver_crospi/trossen_widowx_leader_follower_driver_crospi.hpp"
#include <fmt/format.h>
#include <iostream>

// includ library for sleep
#include <chrono>
#include <thread>

#include <iostream>
#include <thread>
#include <vector>

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

namespace etasl {


trossen_widowx_leader_follower_driver_crospi::trossen_widowx_leader_follower_driver_crospi()
{
}

void trossen_widowx_leader_follower_driver_crospi::construct(std::string robot_name,
                        const Json::Value& config,
                        std::shared_ptr<etasl::JsonChecker> jsonchecker)
{

    driver_ip_address_follower_ = jsonchecker->asString(config, "ip_address_follower");
    driver_ip_address_leader_ = jsonchecker->asString(config, "ip_address_leader");

    periodicity = jsonchecker->asDouble(config, "periodicity");
    control_mode = jsonchecker->asString(config, "control_mode");
    lead_gain = jsonchecker->asDouble(config, "lead_gain");
    K_p = jsonchecker->asDouble(config, "K_p");
    K_d = jsonchecker->asDouble(config, "K_d");
    force_feedback_gain = jsonchecker->asDouble(config, "force_feedback_gain");

    safety_velocity_threshold = jsonchecker->asDouble(config, "safety_velocity_threshold");
    safety_torque_threshold = jsonchecker->asDouble(config, "safety_torque_threshold");


    move_to_home_position_on_startup = jsonchecker->asBool(config, "move_to_home_position_on_startup");


    std::cout << "lead gain --------------------------------------- " << lead_gain << std::endl;



    name = robot_name; //defined in RobotDriver super class.

    std::cout << "Constructed object of trossen_widowx_leader_follower_driver_crospi class with name: " << name << std::endl;

    DOF = 6;

    AvailableFeedback available_fb{};
    available_fb.joint_pos = true;
    available_fb.joint_vel = true;

    constructPorts(DOF, available_fb); //Constructs all shared pointers and initialize data structures. Call after assigning available_feedback booleans.

    //Initialize vectors
    // initial_joints_follower.resize(DOF, 0.0); 
    crospi_joint_velocity_setpoints_.resize(DOF, 0.0);
    follower_joint_position_setpoints_.resize(DOF, 0.0);
    amplified_crospi_joint_velocity_setpoints_.resize(DOF, 0.0);
    // vel_setpoint_follower.resize(DOF, 0.0);

    // Initialize structure data
    follower_setpoint_joint_vel_struct.data.resize(DOF, 0.0); //resize and initialize setpoint joint velocities to zero
    follower_joint_pos_struct.data.resize(DOF, 0.0);
    follower_joint_vel_struct.data.resize(DOF, 0.0);

    leader_joint_position_setpoints_.resize(DOF, 0.0);
    leader_joint_velocity_setpoints_.resize(DOF, 0.0);
    leader_joint_external_efforts_setpoints_.resize(DOF, 0.0);

}

bool trossen_widowx_leader_follower_driver_crospi::initialize()
{

    // std::cout << "Configuring logging with spdlog..." << std::endl;
    // auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    // auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
    //     "trossen_widowx_leader_follower_driver_crospi.log",
    //     true
    // );
    // std::vector<spdlog::sink_ptr> sinks {stdout_sink, file_sink};
    // // auto default_logger = std::make_shared<spdlog::logger>(
    // //     trossen_arm::TrossenArmDriver::get_default_logger_name(), sinks.begin(), sinks.end()
    // // );
    // // spdlog::register_logger(default_logger);


    // auto follower_logger = std::make_shared<spdlog::logger>(
    //     trossen_arm::TrossenArmDriver::get_logger_name(
    //     trossen_arm::Model::wxai_v0,
    //     driver_ip_address_follower_.c_str()
    //     ), sinks.begin(), sinks.end()
    // );
    // spdlog::register_logger(follower_logger);
    // std::cout << "follower_logger registered..." << std::endl;

    // auto leader_logger = std::make_shared<spdlog::logger>(
    //     trossen_arm::TrossenArmDriver::get_logger_name(
    //     trossen_arm::Model::wxai_v0,
    //     driver_ip_address_leader_.c_str()
    //     ), sinks.begin(), sinks.end()
    // );
    // spdlog::register_logger(leader_logger);
    // std::cout << "leader_logger registered..." << std::endl;


    std::cout << "Configuring logging with spdlog..." << std::endl;

    auto stdout_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
        "trossen_widowx_leader_follower_driver_crospi.log", true);

    std::vector<spdlog::sink_ptr> sinks {stdout_sink, file_sink};

    // Avoids issues when creating multiple drivers.
    auto default_logger = std::make_shared<spdlog::logger>(
        trossen_arm::TrossenArmDriver::get_default_logger_name(),
        sinks.begin(), sinks.end());

    spdlog::register_logger(default_logger);

    
    // ---------------------- Follower driver configuration ----------------------
    
    follower_driver_ = std::make_unique<trossen_arm::TrossenArmDriver>();
    std::cout << "follower_driver_ initialized..." << std::endl;


    follower_driver_->configure(
            trossen_arm::Model::wxai_v0,
            trossen_arm::StandardEndEffector::wxai_v0_follower,
            driver_ip_address_follower_.c_str(),
            false,
            5.0); // 5 seconds timeout for the connection
    
            
    // ---------------------- Leader driver configuration ----------------------
    std::cout << "heeeelloooooooo" << std::endl;
    leader_driver_ = std::make_unique<trossen_arm::TrossenArmDriver>();
    std::cout << "leader_driver_ initialized..." << std::endl;  
    
    leader_driver_->configure(
        trossen_arm::Model::wxai_v0,
        trossen_arm::StandardEndEffector::wxai_v0_leader,
        driver_ip_address_leader_.c_str(),
        false,
        5.0); // 5 seconds timeout for the connection

 

    if (!follower_driver_->get_is_configured()) {
        std::cerr << "get_is_configured for follower_driver failed" << std::endl;
        return false;
    }
    std::cout << fmt::format("Trossen follower Driver initialized with IP address: {}", driver_ip_address_follower_) << std::endl;
    if (!leader_driver_->get_is_configured()) {
        std::cerr << "get_is_configured for leader_driver failed" << std::endl;
        return false;
    }
    std::cout << fmt::format("Trossen leader Driver initialized with IP address: {}", driver_ip_address_leader_) << std::endl;


        // ----------------------- Move robots to home position ----------------------
    follower_driver_->set_gripper_mode(trossen_arm::Mode::position);
    follower_driver_->set_gripper_position(0.04, 2.0, true); // Set the gripper to open position

    leader_driver_->set_gripper_mode(trossen_arm::Mode::position);
    leader_driver_->set_gripper_position(0.04, 2.0, true); // Set the gripper to open position
    
    follower_driver_->set_arm_modes(trossen_arm::Mode::position);
    leader_driver_->set_arm_modes(trossen_arm::Mode::position);

    if(move_to_home_position_on_startup){
        std::cout << "Moving to home positions..." << std::endl;
        follower_driver_->set_arm_positions(
            {0.0, M_PI_2, M_PI_2, 0.0, 0.0, 0.0},
            3.0f,
            true
        );
    
        leader_driver_->set_arm_positions(
            {0.0, M_PI_2, M_PI_2, 0.0, 0.0, 0.0},
            3.0f,
            true
        );
    }
    else{
        follower_driver_->set_arm_positions(follower_output_.joint.arm.positions,
                                1.0f, // goal_time
                                true); // Blocking, to let the robot hold the initial position before starting the control loop, since it drops a bit in the beginning.

        leader_driver_->set_arm_positions(leader_output_.joint.arm.positions,
                        1.0f, // goal_time
                        true); // Blocking, to let the robot hold the initial position before starting the control loop, since it drops a bit in the beginning.
    }

    

    std::cout << "Starting the online control of the robots..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    


    // Set the arm joints to position mode
    if (control_mode == "JOINT_POSITION") {
        follower_driver_->set_arm_modes(trossen_arm::Mode::position);
        std::cout << "Control mode of follower set to JOINT_POSITION" << std::endl;

        leader_driver_->set_arm_modes(trossen_arm::Mode::external_effort);
        std::cout << "Control mode of leader set to EXTERNAL_EFFORT" << std::endl;


        leader_driver_->set_arm_external_efforts(
            {0, 0, 0, 0, 0, 0},
            0.0f,
            false
        ); // Set initial external efforts to zero, non-blocking
    } 
    // else if (control_mode == "JOINT_POSITION_WITH_FEEDBACK") {
    //     follower_driver_->set_arm_modes(trossen_arm::Mode::position);
    //     std::cout << "Control mode set to JOINT_POSITION_WITH_FEEDBACK" << std::endl;
    // } 
    // else if (control_mode == "JOINT_VELOCITY") {
    //     follower_driver_->set_arm_modes(trossen_arm::Mode::velocity);
    //     std::cout << "Control mode set to JOINT_VELOCITY" << std::endl;
    // } 
    else {
        std::cerr << "Unknown control mode: " << control_mode << std::endl;
        return false;
    }





    // Create ROS2 node
    ros_node_ = std::make_shared<rclcpp::Node>("widowX_gripper_plugin_node");
    // Create ROS2 services
    control_gripper_srv_ = ros_node_->create_service<trossen_widowx_interfaces::srv::ControlGripper>(
        "widowX/control_gripper",
        std::bind(&trossen_widowx_leader_follower_driver_crospi::control_gripper, this, std::placeholders::_1, std::placeholders::_2));

    set_up_follower_mode_srv_ = ros_node_->create_service<trossen_widowx_interfaces::srv::ArmMode>(
    "widowX/set_up_arm_mode",
    std::bind(&trossen_widowx_leader_follower_driver_crospi::set_up_arm_mode, this, std::placeholders::_1, std::placeholders::_2));

    // Start ROS2 thread
    ros_thread_ = std::thread([this]() {
        rclcpp::executors::SingleThreadedExecutor exec;
        exec.add_node(ros_node_);
        exec.spin();
    });


        // Update the follower output
    follower_output_ = follower_driver_->get_robot_output();
    follower_joint_position_setpoints_ = follower_output_.joint.arm.positions;
    follower_joint_external_efforts_feedback_ = follower_driver_->get_arm_external_efforts();


    // Update the leader output
    leader_output_ = leader_driver_->get_robot_output();
    leader_joint_position_setpoints_ = leader_output_.joint.arm.positions;
    leader_joint_velocity_setpoints_ = {0, 0, 0, 0, 0, 0};
    leader_joint_external_efforts_setpoints_ = {0, 0, 0, 0, 0, 0}; // Initialize leader external efforts to zero
    leader_joint_external_efforts_feedback_ = leader_driver_->get_arm_external_efforts();

    assert(follower_output_.joint.arm.positions.size() == DOF);
    assert(leader_output_.joint.arm.positions.size() == DOF);

    std::copy(follower_output_.joint.arm.positions.begin(), follower_output_.joint.arm.positions.end(), follower_joint_pos_struct.data.begin());
    writeFeedbackJointPosition(follower_joint_pos_struct);

    assert(follower_output_.joint.arm.velocities.size() == DOF);
    std::copy(follower_output_.joint.arm.velocities.begin(), follower_output_.joint.arm.velocities.end(), follower_joint_vel_struct.data.begin());
    writeFeedbackJointVelocity(follower_joint_vel_struct);


    

    
    // sleep this thread
    // std::this_thread::sleep_for(std::chrono::seconds(10));
    std::cout << "Initialization complete" << std::endl;

    return true;
}


void trossen_widowx_leader_follower_driver_crospi::update(volatile std::atomic<bool>& stopFlag)
{   
    update_mutex_.lock();

    readSetpointJointVelocity(follower_setpoint_joint_vel_struct);
    std::copy(follower_setpoint_joint_vel_struct.data.begin(), follower_setpoint_joint_vel_struct.data.end(), crospi_joint_velocity_setpoints_.begin()); // crospi_joint_velocity_setpoints_ = follower_setpoint_joint_vel_struct.data;


    follower_output_ = follower_driver_->get_robot_output();
    leader_output_ = leader_driver_->get_robot_output();

    std::copy(follower_output_.joint.arm.positions.begin(), follower_output_.joint.arm.positions.end(), follower_joint_pos_struct.data.begin());
    writeFeedbackJointPosition(follower_joint_pos_struct);

    std::copy(follower_output_.joint.arm.velocities.begin(), follower_output_.joint.arm.velocities.end(), follower_joint_vel_struct.data.begin());
    writeFeedbackJointVelocity(follower_joint_vel_struct);

    // TODO: Check the load of this loop
    if (std::any_of(
      crospi_joint_velocity_setpoints_.begin(), crospi_joint_velocity_setpoints_.end(),
      [this](double velocity) {
        return std::isnan(velocity) || std::isinf(velocity);
      }))
    {
        //  break the loop if any velocity is NaN or INF
        crospi_joint_velocity_setpoints_.clear();
        crospi_joint_velocity_setpoints_.resize(6, 0.0); // Reset to zero
        stopFlag.store(true);
        std::cerr << "Invalid joint velocity command detected" << std::endl;
        return; // Exit the update function
    }

    if (control_mode == "JOINT_POSITION"){

        follower_joint_external_efforts_feedback_ = follower_driver_->get_arm_external_efforts();
        leader_joint_external_efforts_feedback_ = leader_driver_->get_arm_external_efforts();
        // leader_joint_external_efforts_setpoints_ = follower_driver_->get_arm_external_efforts();


        for (unsigned int i=0; i<follower_joint_position_setpoints_.size(); ++i) {
            if(abs(follower_output_.joint.arm.velocities[i]) > safety_velocity_threshold){
                stopFlag.store(true);
                std::cerr << "Follower velocity of joint " << i+1 << " is too high: " << follower_output_.joint.arm.velocities[i] << std::endl;
                return; // Exit the update function
            }

            if(abs(leader_output_.joint.arm.velocities[i]) > safety_velocity_threshold){
                stopFlag.store(true);
                std::cerr << "Leader velocity of joint " << i+1 << " is too high: " << leader_output_.joint.arm.velocities[i] << std::endl;
                return; // Exit the update function
            }

            if(abs(follower_joint_external_efforts_feedback_[i]) > safety_torque_threshold){
                stopFlag.store(true);
                std::cerr << "Follower external torque of joint " << i+1 << " is too high: " << follower_joint_external_efforts_feedback_[i] << std::endl;
                return; // Exit the update function
            }

            if(abs(leader_joint_external_efforts_feedback_[i]) > safety_torque_threshold){
                stopFlag.store(true);
                std::cerr << "Leader external torque of joint " << i+1 << " is too high: " << leader_joint_external_efforts_feedback_[i] << std::endl;
                return; // Exit the update function
            }

            // follower_joint_position_setpoints_[i] += crospi_joint_velocity_setpoints_[i]*periodicity; //simple integration
            // amplified_crospi_joint_velocity_setpoints_[i] = lead_gain * crospi_joint_velocity_setpoints_[i];
            // follower_joint_position_setpoints_[i] = (follower_output_.joint.arm.positions[i] + crospi_joint_velocity_setpoints_[i]*periodicity) - leader_output_.joint.arm.positions[i]; //Position setpoints for follower
            
            // leader_joint_external_efforts_setpoints_[i] = -force_feedback_gain * follower_joint_external_efforts_feedback_[i]; //External torque setpoints for leader
            
            // leader_joint_external_efforts_setpoints_[i] =  -force_feedback_gain * follower_joint_external_efforts_feedback_[i] + K_p * (follower_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i]); //External torque setpoints for leader
            // leader_joint_external_efforts_setpoints_[i] =  -force_feedback_gain * follower_joint_external_efforts_feedback_[i] + K_p * (lead_gain*crospi_joint_velocity_setpoints_[i]*periodicity); //Same as  K_p * (follower_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i])
            
            //Option without haptic feedback, just a spring for the leader robot:
            // follower_joint_position_setpoints_[i] += crospi_joint_velocity_setpoints_[i]*periodicity;
            // leader_joint_external_efforts_setpoints_[i] = K_p * (follower_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i])
            //                                             + K_d * (crospi_joint_velocity_setpoints_[i] - leader_output_.joint.arm.velocities[i]); //Same as  K_p * (follower_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i])
            

            // Option 1: the first one that we tried:
            // follower_joint_position_setpoints_[i] = leader_output_.joint.arm.positions[i] + lead_gain*crospi_joint_velocity_setpoints_[i]*periodicity; //Position setpoints for follower
            // leader_joint_external_efforts_setpoints_[i] = -force_feedback_gain * follower_joint_external_efforts_feedback_[i] + K_p * (follower_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i]); //Same as  K_p * (follower_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i])
            

            // Option 2: the second one that we tried, now with velocity term:
            follower_joint_position_setpoints_[i] = leader_output_.joint.arm.positions[i]; //Position setpoints for follower
            
            leader_joint_position_setpoints_[i] += crospi_joint_velocity_setpoints_[i]*periodicity; //simple integration (open loop)
            // leader_joint_position_setpoints_[i] = leader_output_.joint.arm.positions[i] + lead_gain*crospi_joint_velocity_setpoints_[i]*periodicity; //closed-loop position setpoint
            leader_joint_velocity_setpoints_[i] = crospi_joint_velocity_setpoints_[i];
            leader_joint_external_efforts_setpoints_[i] = -force_feedback_gain * follower_joint_external_efforts_feedback_[i] 
                                                        + K_p * (leader_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i])
                                                        + K_d * (leader_joint_velocity_setpoints_[i] - leader_output_.joint.arm.velocities[i]); //Same as  K_p * (follower_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i])
            

            // The following is an stiffness controller in jointspace (still needs damping):
            // leader_joint_external_efforts_setpoints_[i] =  K_p * (follower_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i]); //Same as  K_p * (follower_joint_position_setpoints_[i] - leader_output_.joint.arm.positions[i])


        }

        // std::cout << "last joint position: " << follower_joint_position_setpoints_[5] << std::endl;
        // follower_driver_->set_arm_positions(follower_joint_position_setpoints_,
        //                             0.0, // goal_time
        //                             false // blocking is false, so it will not wait for the goal to be reached
        // ); 

        follower_driver_->set_arm_positions(follower_joint_position_setpoints_,
                        0.0, // goal_time
                        false, // blocking is false, so it will not wait for the goal to be reached
                        crospi_joint_velocity_setpoints_); // feedforward velocities coming from Crospi
    

        leader_driver_->set_arm_external_efforts(
            leader_joint_external_efforts_setpoints_,
            0.0, // goal_time, it was originally 0.0f for the example of teleoperation from Trossen. 
            false
        );

    }


    // else if (control_mode == "JOINT_POSITION_WITH_FEEDBACK"){
    //     for (unsigned int i=0; i<follower_joint_position_setpoints_.size(); ++i) {
    //         if(abs(follower_output_.joint.arm.velocities[i]) > safety_velocity_threshold){
    //             stopFlag.store(true);
    //             std::cerr << "Velocity of joint " << i+1 << " is too high: " << follower_output_.joint.arm.velocities[i] << std::endl;
    //             return; // Exit the update function
    //         }
    //         // follower_joint_position_setpoints_[i] += crospi_joint_velocity_setpoints_[i]*periodicity; //simple integration
    //         // follower_joint_position_setpoints_[i] = follower_output_.joint.arm.positions[i] + crospi_joint_velocity_setpoints_[i]*periodicity*lead_gain; //simple integration
    //         follower_joint_position_setpoints_[i] = follower_output_.joint.arm.positions[i]; //I think this does not work because the robot keeps falling and falling due to gravity and low gains of the position loop.



    //     }
    //     follower_driver_->set_arm_positions(follower_joint_position_setpoints_,
    //                                 periodicity, // goal_time
    //                                 false); // blocking is false, so it will not wait for the goal to be reached
    // }
    else if (control_mode == "JOINT_VELOCITY") {
        follower_driver_->set_arm_velocities(crospi_joint_velocity_setpoints_,
                                    periodicity, // goal_time
                                    false); // blocking is false, so it will not wait for the goal to be reached
    }



    update_mutex_.unlock();
}

void trossen_widowx_leader_follower_driver_crospi::control_gripper( const std::shared_ptr<trossen_widowx_interfaces::srv::ControlGripper::Request> request,
                                                std::shared_ptr<trossen_widowx_interfaces::srv::ControlGripper::Response> response ) {

    follower_driver_->set_gripper_position(request->position, request->time, true); // Set the gripper position blocking
    response->success = true;

}


void trossen_widowx_leader_follower_driver_crospi::set_up_arm_mode( const std::shared_ptr<trossen_widowx_interfaces::srv::ArmMode::Request> request,
                                                std::shared_ptr<trossen_widowx_interfaces::srv::ArmMode::Response> response ) {
    
    // lock update hook
    update_mutex_.lock();
    // Set control mode
    control_mode = request->arm_mode;
    
    follower_joint_position_setpoints_ = follower_driver_->get_arm_positions(); // Initialize the position setpoints with the current positions
    std::copy(follower_joint_position_setpoints_.begin(), follower_joint_position_setpoints_.end(), follower_joint_pos_struct.data.begin());

    // Set the arm joints to position mode
    if (control_mode == "JOINT_POSITION") {
        follower_driver_->set_arm_modes(trossen_arm::Mode::position);
        // logger
        RCLCPP_INFO(ros_node_->get_logger(), "Control mode set to JOINT_POSITION");
        // Log arm joint position setpoints
        // for (size_t i = 0; i < follower_joint_position_setpoints_.size(); ++i) {
        //     RCLCPP_INFO(ros_node_->get_logger(), "Joint %zu position setpoint: %f", i, follower_joint_position_setpoints_[i]);
        // }
        
        response->success = true;
    } 
    else if (control_mode == "JOINT_POSITION_WITH_FEEDBACK") {
        follower_driver_->set_arm_modes(trossen_arm::Mode::position);
        // logger
        RCLCPP_INFO(ros_node_->get_logger(), "Control mode set to JOINT_POSITION_WITH_FEEDBACK");
        // Log arm joint position setpoints
        // for (size_t i = 0; i < follower_joint_position_setpoints_.size(); ++i) {
        //     RCLCPP_INFO(ros_node_->get_logger(), "Joint %zu position setpoint: %f", i, follower_joint_position_setpoints_[i]);
        // }
        
        response->success = true;
    } 
    else if (control_mode == "JOINT_VELOCITY") {
        follower_driver_->set_arm_modes(trossen_arm::Mode::velocity);
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
    follower_driver_->set_all_modes(trossen_arm::Mode::idle);
    leader_driver_->set_all_modes(trossen_arm::Mode::idle);

    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Destroy driver object
    follower_driver_.reset();
    leader_driver_.reset();
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
