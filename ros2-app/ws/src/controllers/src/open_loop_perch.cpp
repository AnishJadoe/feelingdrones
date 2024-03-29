#include "open_loop_perch.hpp"
#include <iostream>
#include <vector>

// Constants
float EPSILON = 0.03; 
int8_t OPEN = 1;
int8_t CLOSED = 0; 
int8_t UNKNOWN = -1; 


FeelyDrone::FeelyDrone()
    : Node("FeelyDrone"), _offboard_setpoint_counter(0), _current_state(States::IDLE)
{   

    /* Declare all the parameters */
    this->declare_parameter("frequency", 20.0);

    /* Actually get all the parameters */
    this->_frequency =  this->get_parameter("frequency").as_double();
    this->_timer = this->create_wall_timer(1.0 / this->_frequency * 1s,
                                            std::bind(&FeelyDrone::_timer_callback, this));
    this->_status_subscription = this->create_subscription<px4_msgs::msg::VehicleStatus>(
        "/fmu/out/vehicle_status", rclcpp::SensorDataQoS(), std::bind(&FeelyDrone::_status_callback, this, std::placeholders::_1));
    this->_timesync_subscription = this->create_subscription<px4_msgs::msg::TimesyncStatus>(
        "/fmu/out/timesync_status", rclcpp::SensorDataQoS(), std::bind(&FeelyDrone::_timesync_callback, this, std::placeholders::_1));
    this->_reference_subscription = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/bar/pose", rclcpp::SensorDataQoS(), std::bind(&FeelyDrone::_reference_callback, this, std::placeholders::_1));
    this->_vehicle_odometry_subscription = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        "/fmu/out/vehicle_odometry", rclcpp::SensorDataQoS(), std::bind(&FeelyDrone::_vehicle_odometry_callback, this, std::placeholders::_1));

    this->_gripper_state_subscription = this->create_subscription<std_msgs::msg::Int8>(
        "/gripper/out/gripper_state", rclcpp::SensorDataQoS(), std::bind(&FeelyDrone::_update_gripper_state_callback, this, std::placeholders::_1)
    );

    
    this->_gripper_publisher = this->create_publisher<std_msgs::msg::Int8>(
        "/gripper/in/gripper_state", 10);

    this->_drone_state_publisher = this->create_publisher<custom_msgs::msg::StampedInt8>(
        "/drone/out/state",10);
        
    this->_offboard_publisher = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
        "/fmu/in/offboard_control_mode", 10);
    this->_trajectory_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
        "/fmu/in/trajectory_setpoint", 10);
    this->_vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>(
        "/fmu/in/vehicle_command", 10);
    

    this->_beginning = this->now();

    /* Init Ref Pose */
    this->_ref_pos = {0.0, 0.0, -0.1};
    this->_ref_yaw = 0.0;
    this->_est_pos = {0.0,0.0,0.0};

    //Start counter
    this->_period_counter = 0;
    this->_change_state(States::HOVER);
}

std::string FeelyDrone:: stateToString(States state) const{
    switch (state) {
        case States::IDLE:
            return "IDLE";
        case States::MOVING:
            return "MOVING";
        case States::HOVER:
            return "HOVER";
        case States::TOUCHED:
            return "TOUCHED";
        case States::GRASP:
            return "GRASP";
        case States::EVALUATE:
            return "EVALUATE";
        case States::SEARCHING:
            return "SEARCHING";
        case States::LAND:
            return "LANDING";
        default:
            return "UNKNOWN"; // Handle invalid states gracefully
    }
}

void FeelyDrone::_change_state(States new_state)
{
    this->_current_state = new_state;
    RCLCPP_INFO(this->get_logger(), "State changed to %s", this->stateToString(new_state).c_str());
}

/* Callback Functions */
void FeelyDrone::_timer_callback()
{
    
    if (_offboard_setpoint_counter == 50) 
    {
        // // /* On the real system we want to arm and change mode using the remote control
        // //     Uncomment this for the SITL e.g. automatic arming and switch to offboard mode */
        // // // Change to Offboard mode after 10 setpoints
        // this->_publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
        // // // // Arm the vehicle
        // this->arm();

        // RCLCPP_INFO(this->get_logger(), "Arm now");
        // this->_change_state(States::HOVER);
    }
    this->_publish_drone_state();
    this->_perching_state_machine();

    if(this->_current_state != States::LAND)
    {
        this->_publish_offboard_control_mode();
        this->_publish_trajectory_setpoint();
    }

    // stop the counter after reaching 10
    if (_offboard_setpoint_counter < 51) {
        _offboard_setpoint_counter++;
    }

}

void FeelyDrone::_perching_state_machine(){
    if (this->_current_state == States::HOVER){
        this->_hover_event_handler();
        return;
    }

    if (this->_current_state == States::MOVING){
        this->_moving_event_handler();
        return;
    }


    if (this->_current_state == States::GRASP){
        this->_grasp_event_handler();
        return;
    }

    if (this->_current_state == States::LAND){
        this->_land_event_handler();
        return;
    }
}

void FeelyDrone::_hover_event_handler()
{
    double t = (this->now() - this->_beginning).seconds();

    RCLCPP_INFO(this->get_logger(), "HOVERING" );

    Eigen::Vector3d start_pos = {0,0,-0.8};
    if (t >= 6 && t < 15 && !this->_landed){
        this->_ref_pos = start_pos;
    }

    if (t >= 12 && !this->_landed && this->_gripper_state != OPEN){
        std_msgs::msg::Int8 msg{};
        msg.data = OPEN; 
        this->_gripper_publisher->publish(msg);
        return;
    }

    if (t > 15 && !this->_landed){
        this->_t_moving = t; 
        this->_change_state(States::MOVING);
        return;

    }
}
void FeelyDrone::_moving_event_handler(){

    double t = (this->now() - this->_beginning).seconds();
    float t_trajectory = t - this->_t_moving;
    RCLCPP_INFO(this->get_logger(), "MOVING TO OBJECT" );
    float EPSILON_SEARCH = 0.05;
    float velocity = 0.15;
    this->_goal_pos = {this->_obj_pos.x(), this->_obj_pos.y() + 0.15, this->_obj_pos.z() + 0.15};
    float norm = (this->_est_pos - this->_goal_pos).norm();
    bool at_pos = norm < EPSILON_SEARCH;    
    Eigen::Vector3d start_pos = {0,0,-0.8};
    //this->_ref_yaw = M_PI*0.5;

    if (at_pos){
            this->_change_state(States::GRASP);
            return;
        }
        else{
            this->_ref_pos = start_pos + t_trajectory* (this->_goal_pos - start_pos).normalized() * velocity;
            RCLCPP_INFO(this->get_logger(), "Flying to position [x: %f, y: %f, z: %f]", this->_ref_pos.x(), this->_ref_pos.y(), this->_ref_pos.z());
            RCLCPP_INFO(this->get_logger(), "Norm: %f", norm  );
        }
}


void FeelyDrone::_grasp_event_handler(){

    this->_ref_pos.x() = this->_obj_pos.x();
    this->_ref_pos.y() = this->_obj_pos.y() + 0.15;
    this->_ref_pos.z() = this->_obj_pos.z() + 0.05;
    RCLCPP_INFO(this->get_logger(), "ALIGNING" );
    float norm = (_est_pos - _ref_pos).squaredNorm();
    if ( norm < EPSILON)
    {
        std_msgs::msg::Int8 msg{};
        msg.data = CLOSED; 
        RCLCPP_INFO(this->get_logger(), "GRASPING" );
        this->_gripper_publisher->publish(msg);

        if (this->_gripper_state == CLOSED){
            this->_change_state(States::LAND);
        }
        return;
    }

}


void FeelyDrone::_publish_trajectory_setpoint()
{
    double t = (this->now() - this->_beginning).seconds();

    /* After mission time ran out */
    if(t > 300 && !this->_landed)
    {
        this->_landed = true;
        this->land();
        return;
    }

    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.position = {this->_ref_pos.x(),
                    this->_ref_pos.y(),
                    this->_ref_pos.z()};
    msg.yaw = this->_ref_yaw; // [-PI:PI]
    msg.timestamp = this->get_timestamp();
    this->_trajectory_publisher->publish(msg);

}
void FeelyDrone::_land_event_handler(){
    RCLCPP_INFO(this->get_logger(), "LANDING" );
    this->land();
    return;
}
void FeelyDrone::_publish_drone_state(){
        custom_msgs::msg::StampedInt8 msg{};
        msg.timestamp = this->get_timestamp();
        msg.data = (int) this->_current_state; 
        this->_drone_state_publisher->publish(msg);
        }

void FeelyDrone::_publish_offboard_control_mode()
{
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_timestamp();
    _offboard_publisher->publish(msg);
}

void FeelyDrone::_vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg){
    

    this->_est_pos.x() = msg->position[0];
    this->_est_pos.y() = msg->position[1];
    this->_est_pos.z() = msg->position[2];    

}

void FeelyDrone::_update_gripper_state_callback(const std_msgs::msg::Int8::SharedPtr msg){
    // Whenever the gripper state is changed, we update it here for continuity 
    if (msg->data == OPEN){
        this->_gripper_state = OPEN;
    }
    else if(msg->data == CLOSED){
        this->_gripper_state = CLOSED;
    }
    else{
        this->_gripper_state = UNKNOWN;
    }
}

void FeelyDrone::_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
    this->_nav_state = msg->nav_state;
    this->_arming_state = msg->arming_state;
}

void FeelyDrone::_reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{

    Eigen::Vector3d position = {msg->pose.position.x, msg->pose.position.y,  msg->pose.position.z};
    // transform to px4 (ned) frame
    position = px4_ros_com::frame_transforms::enu_to_ned_local_frame(position);
    this->_obj_pos.x() = position.x();
    this->_obj_pos.y() = position.y();
    this->_obj_pos.z() = position.z();

}

void FeelyDrone::_timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
{
    this->_timestamp_local = std::chrono::steady_clock::now();
    this->_timestamp_remote.store(msg->timestamp);
}

uint64_t FeelyDrone::get_timestamp()
{
    auto now = std::chrono::steady_clock::now();
    return this->_timestamp_remote.load() + std::chrono::round<std::chrono::microseconds>(now - this->_timestamp_local).count();
}

/**
 * @brief Send a command to Arm the vehicle
 */
void FeelyDrone::arm()
{
    this->_publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);

    RCLCPP_INFO(this->get_logger(), "Arm command send");
}
/**
 * @brief Send a command to Disarm the vehicle
 */
void FeelyDrone::disarm()
{
    _publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}
/**
 * @brief Send a command to takeoff
 */
void FeelyDrone::takeoff()
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param7 = 3.0;
    msg.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_timestamp();
    _vehicle_command_pub->publish(msg);

    this->_taken_off = true;
    RCLCPP_INFO(this->get_logger(), "Takeoff command send");
}
/**
 * @brief Send a command to land
 */
void FeelyDrone::land()
{
    _publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

    RCLCPP_INFO(this->get_logger(), "Land command send");
}


void FeelyDrone::_publish_vehicle_command(uint16_t command,
                                                            float param1,
                                                            float param2,
                                                            float param3,
                                                            float param4,
                                                            float param5,
                                                            float param6,
                                                            float param7)
{
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.param3 = param3;
    msg.param4 = param4;
    msg.param5 = param5;
    msg.param6 = param6;
    msg.param7 = param7;

    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_timestamp();
    this->_vehicle_command_pub->publish(msg);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
    
  rclcpp::spin(std::make_shared<FeelyDrone>());
  rclcpp::shutdown();
  return 0;
}
