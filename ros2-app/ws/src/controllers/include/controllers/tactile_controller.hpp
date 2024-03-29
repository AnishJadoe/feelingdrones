#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/timesync_status.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_ros_com/frame_transforms.h"
#include "std_msgs/msg/int8.hpp"
#include "custom_msgs/msg/stamped_int32_multi_array.hpp"
#include "custom_msgs/msg/stamped_int8.hpp"
#include <string>

using namespace std::chrono_literals;



class FeelyDrone : public rclcpp::Node
{
public:
    FeelyDrone();
private: 


    enum class States {IDLE, HOVER, MOVING, TOUCHED, GRASP, EVALUATE, REFINE, SEARCHING, LAND};
    States _current_state;

    uint8_t _nav_state, _arming_state, _gripper_state;
    bool _taken_off = false, _landed = false, _position_set = false, _calibrate_sensors=true;
    uint _offboard_setpoint_counter;
    double _frequency; 
    


    std::string stateToString(States state) const ;
    rclcpp::Time _beginning;
    rclcpp::TimerBase::SharedPtr _timer;

    /* Publishers and Subscribers */
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr _status_subscription;
    rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_subscription;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _reference_subscription;
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _vehicle_odometry_subscription;
    rclcpp::Subscription<custom_msgs::msg::StampedInt32MultiArray>::SharedPtr _tactile_sensor_subscription;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr _gripper_state_subscription;


    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _trajectory_publisher;
    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_publisher;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
    rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr _gripper_publisher;
    rclcpp::Publisher<custom_msgs::msg::StampedInt8>::SharedPtr _drone_state_publisher;
    rclcpp::Publisher<custom_msgs::msg::StampedInt32MultiArray>::SharedPtr _touched_state_publisher;

    // Time offset
    std::atomic<uint64_t> _timestamp_remote;
    std::chrono::time_point<std::chrono::steady_clock> _timestamp_local;

    // Current High Level Reference
    Eigen::Vector3d _ref_pos;
    Eigen::Vector3d _goal_pos;
    Eigen::Vector3d _est_pos;
    Eigen::Vector3d _touch_pos; // Location of touch event
    Eigen::Vector3d _grasp_pos;
    Eigen::Vector3d _obj_pos; // Location of object
    Eigen::Vector3d _init_pos = {0.0, 0.0, -0.8};
    float _ref_yaw;
    int8_t _tactile_state[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    int8_t _touched_state[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
    int32_t _init_raw_tactile_data[12];
    int32_t _current_raw_tactile_data[12];
    int _period_counter = 0;
    float _t_search;
    float _t_touch;
    float _x_search;
    float _y_search;
    float _t_evaluate;
    float _t_land;
    float _goal_norm;
    float _y_offset_bar;
    float _bar_height;
    uint8_t _grasp_counter = 0;





    /* Callback Functions */
    void _timer_callback();

    /**
     * @brief Publish a trajectory setpoint
     *        For this example, it sends a trajectory setpoint to make the
     *        vehicle hover at 5 meters with a yaw angle of 180 degrees.
     */
    void _publish_trajectory_setpoint();

    /**
     * @brief Publish the offboard control mode.
     *        For this example, only position and altitude controls are active.
     */
    void _publish_offboard_control_mode();

    States _get_current_state() const;
    void _change_state(States new_state);
    
    void _tactile_state_machine();
    void _hover_event_handler();
    void _rect_searching_event_handler();
    void _elipse_searching_event_handler();
    void _zigzag_searching_event_handler();
    void _moving_event_handler();
    void _touch_event_handler();
    void _grasp_event_handler();
    void _evaluate_event_handler();
    void _land_event_handler();



    void _publish_drone_state();
    void _publish_tactile_state();
    void _move_drone();

    Eigen::Vector3d _setpoint_generator(float t_trajectory);
    
    void _update_gripper_state_callback(const std_msgs::msg::Int8::SharedPtr msg);

    void _status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);

    void _reference_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void _timesync_callback(const px4_msgs::msg::TimesyncStatus::SharedPtr msg);

    void _tactile_callback(const custom_msgs::msg::StampedInt32MultiArray::SharedPtr msg);

    void _vehicle_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);

    void _vehicle_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);


    uint64_t get_timestamp();

    /**
     * @brief Send a command to Arm the vehicle
     */
    void arm();

    /**
     * @brief Send a command to Disarm the vehicle
     */
    void disarm();

    /**
     * @brief Send a command to takeoff
     */
    void takeoff();

    /**
     * @brief Send a command to land
     */
    void land();


    void _publish_vehicle_command(uint16_t command,
                                  float param1 = 0.0,
                                  float param2 = 0.0,
                                  float param3 = 0.0,
                                  float param4 = 0.0,
                                  float param5 = 0.0,
                                  float param6 = 0.0,
                                  float param7 = 0.0);

};

