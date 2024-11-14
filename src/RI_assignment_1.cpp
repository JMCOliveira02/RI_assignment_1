#include <sensor_msgs/LaserScan.h>
#include <signal.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <webots_ros/Int32Stamped.h>
#include <webots_ros/robot_get_device_list.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>
#include "ros/ros.h"

#define TIME_STEP 32

#define TIME_STEP_SRV "/robot/time_step"

#define ROBOT_1_NAME "robot1"
#define ROBOT_2_NAME "robot2"

#define ROBOT_1_MAX_LINEAR_SPEED 5.0
#define ROBOT_1_MAX_ANGULAR_SPEED 4.0

#define ROBOT_2_MAX_LINEAR_SPEED 7.0
#define ROBOT_2_MAX_ANGULAR_SPEED 4.0

#define SIDE_LIDAR 0
#define FRONT_LIDAR 1

#define LIDAR_SIDE_RESOLUTION 165
#define LIDAR_SIDE_FOV M_PI * 165 / 180
#define LIDAR_SIDE_SERVICE "/lidar_side/enable"
#define LIDAR_SIDE_TOPIC "/lidar_side/laser_scan"

#define LIDAR_FRONT_RESOLUTION 50
#define LIDAR_FRONT_FOV M_PI * 50 / 180
#define LIDAR_FRONT_SERVICE "/lidar_front/enable"
#define LIDAR_FRONT_TOPIC "/lidar_front/laser_scan"

#define RIGHT_WHEEL_POS_SERVICE "/right_wheel_motor/set_position"
#define RIGHT_WHEEL_VEL_SERVICE "/right_wheel_motor/set_velocity"

#define LEFT_WHEEL_POS_SERVICE "/left_wheel_motor/set_position"
#define LEFT_WHEEL_VEL_SERVICE "/left_wheel_motor/set_velocity"

/**
  @class Robot
  @brief Class that represents a robot in the simulation
  
 */
class Robot {
private:
  std::string name;
  ros::NodeHandle n;

  // Services
  ros::ServiceClient leftWheelPos_Client;
  webots_ros::set_float leftWheelPos_Srv;
  ros::ServiceClient rightWheelPos_Client;
  webots_ros::set_float rightWheelPos_Srv;

  ros::ServiceClient leftWheelVel_Client;
  webots_ros::set_float leftWheelVel_Srv;
  ros::ServiceClient rightWheelVel_Client;
  webots_ros::set_float rightWheelVel_Srv;

  ros::ServiceClient lidarSideEnable_Client;
  webots_ros::set_int lidarSideEnable_Srv;

  ros::ServiceClient lidarFrontEnable_Client;
  webots_ros::set_int lidarFrontEnable_Srv;

  ros::ServiceClient timeStepClient;
  webots_ros::set_int timeStepSrv;

  ros::Subscriber sideLidarSub;
  ros::Subscriber frontLidarSub;

  // Subscribers
  ros::Subscriber lidarSub;

  // Variables
  float v_max, w_max;
  float v, w, theta;                      // Linear and angular velocity of the robot, and the desired angle for the robot
  float angle_correction;

  bool follower;                          //Defines if robot is of the type follower

  float side_laser_scan[LIDAR_SIDE_RESOLUTION];
  float front_laser_scan[LIDAR_FRONT_RESOLUTION];
  float min_side_angle_distance[2];
  float min_front_angle_distance[2];

  float target_wall_distance;             // Desired distance from the wall
  float wall_distance_tolerance;          // Tolerance for the desired distance from the wall

  float angle_distance[2];
  float min_distance = 1000;
  float min_index = 0;
  float angle_step;

  float left_vel = 0; 
  float right_vel = 0;

  float Kp_theta = 18.0/(M_PI/2);         // Proportional gain for the angular correction with the wall
  float Kp_distance = 6.0;                // Proportional gain for the distance correction with the wall

  float error_theta = 0;                  // Error between the desired angle for the robot to mantain with the wall and current
  float wall_error_distance = 0;          // Error between the desired distance and the distance from the wall
  float front_error_distance = 0;         // Error between the desired distance and the distance from the front robot
  
  float Kp_v = 0;                         // Proportional gain for the linear velocity correction with the front obstacle
  float front_deaccel_distance = 0;       // Distance at which the robot starts to deaccelerate
  float equilib_distance = 0;             // Distance at which the robot is at the same speed with the front obstacle
public:
  

  void sideLidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    min_distance = 1000;
    min_index = 0;
    angle_step = LIDAR_SIDE_FOV / LIDAR_SIDE_RESOLUTION;
    for (int i = 0; i < LIDAR_SIDE_RESOLUTION; i++) {
      if (msg->ranges[i] < min_distance) {
        min_distance = msg->ranges[i];
        min_index = i;
      }
    }

    min_side_angle_distance[0] = min_index * angle_step;
    min_side_angle_distance[1] = min_distance;
  }
  
  void frontLidarCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    min_distance = 1000;
    min_index = 0;
    angle_step = LIDAR_FRONT_FOV / LIDAR_FRONT_RESOLUTION;
    for (int i = 0; i < LIDAR_FRONT_RESOLUTION; i++) {
      if (msg->ranges[i] < min_distance){
        min_distance = msg->ranges[i];
        min_index = i;
      }
    }
    
    min_front_angle_distance[0] = min_index * angle_step;
    min_front_angle_distance[1] = min_distance;
  }
  
    /**
  * @brief Constructor for the Robot class.
  *
  *
  *
  * @param[in] name Name of the robot (must coincide with webots name)
  * @param[in] v_max Maximum linear velocity desired  the robot
  * @param[in] w_max Maximum angular velocity desired for the robot
  * @param[in] follower Boolean that defines if the robot slows down with front obstacles
  * @param[in] target_wall_distance Desired distance from the wall
  * @param[in] wall_distance_tolerance Tolerance for the desired distance from the wall
  * @param[in] equilib_distance Desired distance between the robot and the front obstacle
  */
  Robot(std::string name, float v_max, float w_max, bool follower, float target_wall_distance, float wall_distance_tolerance, float equilib_distance) {
    this->name = name;
    this->v_max = v_max;
    this->w_max = w_max;

    v = 0;
    w = 0;

    theta = M_PI / 2;

    this->follower = follower;
    this->target_wall_distance = target_wall_distance;
    this->wall_distance_tolerance = wall_distance_tolerance;
    this->equilib_distance = equilib_distance;
    
    leftWheelPos_Client = n.serviceClient<webots_ros::set_float>('/' + name + LEFT_WHEEL_POS_SERVICE);
    rightWheelPos_Client = n.serviceClient<webots_ros::set_float>('/' + name + RIGHT_WHEEL_POS_SERVICE);
    leftWheelVel_Client = n.serviceClient<webots_ros::set_float>('/' + name + LEFT_WHEEL_VEL_SERVICE);
    rightWheelVel_Client = n.serviceClient<webots_ros::set_float>('/' + name + RIGHT_WHEEL_VEL_SERVICE);
    timeStepClient = n.serviceClient<webots_ros::set_int>('/' + name + TIME_STEP_SRV);
    timeStepSrv.request.value = TIME_STEP;

    sideLidarSub = n.subscribe('/' + name + LIDAR_SIDE_TOPIC, 1, &Robot::sideLidarCallback, this);

    if(follower){
      frontLidarSub = n.subscribe('/' + name + LIDAR_FRONT_TOPIC, 1, &Robot::frontLidarCallback, this);
    }

    setVelocity();
    setWheelPosition(INFINITY, INFINITY);
  }

  void enable_lidar() {
    this->lidarSideEnable_Client = n.serviceClient<webots_ros::set_int>('/' + name + LIDAR_SIDE_SERVICE);
    std::cout << "Enabling side lidar" << std::endl; 
    this->lidarSideEnable_Srv.request.value = TIME_STEP;
    if (this->lidarSideEnable_Client.call(this->lidarSideEnable_Srv) && this->lidarSideEnable_Srv.response.success) {
      std::cout << "Side Lidar enabled" << std::endl;
    } else {
      std::cout << "Failed to enable side lidar" << std::endl;
    }

    if(follower){
       this->lidarFrontEnable_Client = n.serviceClient<webots_ros::set_int>('/' + name + LIDAR_FRONT_SERVICE);
      std::cout << "Enabling front lidar" << std::endl;  // Debug
      this->lidarFrontEnable_Srv.request.value = TIME_STEP;
      if (this->lidarFrontEnable_Client.call(this->lidarFrontEnable_Srv) && this->lidarFrontEnable_Srv.response.success) {
        std::cout << "Front Lidar enabled" << std::endl;
      } else {
        std::cout << "Failed to enable front lidar" << std::endl;
      }
    }

    return;
  }

  bool setWheelPosition(float left_pos, float right_pos) {
    this->leftWheelPos_Srv.request.value = left_pos;
    this->rightWheelPos_Srv.request.value = right_pos;
    if (this->leftWheelPos_Client.call(this->leftWheelPos_Srv) && this->rightWheelPos_Client.call(this->rightWheelPos_Srv) &&
        this->leftWheelPos_Srv.response.success && this->rightWheelPos_Srv.response.success) {
      std::cout << "Position set to " << left_pos << " " << right_pos << std::endl;
      return true;
    }
    std::cout << "Failed to set position" << std::endl;
    return false;
  }

  bool setVelocity() {
    //std::cout << "Setting velocity to " << v << " " << w << std::endl;
    left_vel = v - w / 2;
    right_vel = v + w / 2;

    this->leftWheelVel_Srv.request.value = left_vel;
    this->rightWheelVel_Srv.request.value = right_vel;
    return this->leftWheelVel_Client.call(this->leftWheelVel_Srv) && this->rightWheelVel_Client.call(this->rightWheelVel_Srv) &&
           this->leftWheelVel_Srv.response.success && this->rightWheelVel_Srv.response.success;
  }

  float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }
    /**
  * Calcuates the angular velocity of the robot based on the tangent of and the distance from the wall
  *
  *
  */
  void calculateAngularVelocity() {
    // Uncomment to see the side lidar values
    // std::cout << "Nearest SIDE obstacle angle: " << min_side_angle_distance[0] * 180.0 / M_PI << std::endl;
    // std::cout << "Nearest SIDE obstacle distance: " << min_side_angle_distance[1] << std::endl;

    // Error between the desired angle of movement and the closest obstacle angle detected
    error_theta = theta - min_side_angle_distance[0];

    // Error between the desired distance and the 
    wall_error_distance = target_wall_distance - min_side_angle_distance[1];

    if(abs(wall_error_distance) < wall_distance_tolerance){
      wall_error_distance = 0;
    }

    // PID formula for angular velocity
    w = (Kp_theta * error_theta) - (Kp_distance * wall_error_distance);
    
    // Clamp the angular velocity to prevent it from becoming too large
    w = clamp(w, -w_max, w_max);

    // Debug output for tracKing components
    // std::cout << "theta: " << theta * 180.0 /M_PI << " min_angle_distance " << min_side_angle_distance[0] * 180.0 / M_PI << std::endl;
    // std::cout << "error_w: " << (error_theta * 180.0 / M_PI) << " angular velocity: " << w << std::endl;
  }

  /**
  * Calcuates the linear velocity of the robot. 
  * In case that the robot is follower type, it will slow down when detects the other robot in front of it.
  * 
  * @param[in] linear_speed_diff Difference in max linear speed between the two robots
  */
  void calculateLinearVelocity(float linear_speed_diff) {
    if(!follower){
      v = v_max;
      return;
    }
    
    //Uncomment to see the front lidar values
    //std::cout << "Nearest FRONT obstacle angle: " << min_front_angle_distance[0] * 180.0 / M_PI << std::endl;
    //std::cout << "Nearest FRONT obstacle distance: " << min_front_angle_distance[1] << std::endl;

    front_deaccel_distance = 2.0 * equilib_distance;
    Kp_v = linear_speed_diff / (front_deaccel_distance - equilib_distance);
    front_error_distance = front_deaccel_distance - min_front_angle_distance[1];
    
    //Debug output for tracking components
    //std::cout << "Front error distance: " << front_error_distance << std::endl;
    
    v = v_max - (Kp_v * front_error_distance);
    
    v = clamp(v, 0, v_max);
  }
};

void quit(int sig) {
  ROS_INFO("User stopped the 'wall_follower' node.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_follower", ros::init_options::AnonymousName);
  

  Robot robot1(ROBOT_1_NAME, ROBOT_1_MAX_LINEAR_SPEED, ROBOT_1_MAX_ANGULAR_SPEED, false, 0.75, 0.10, 1.5);
  Robot robot2(ROBOT_2_NAME, ROBOT_2_MAX_LINEAR_SPEED, ROBOT_2_MAX_ANGULAR_SPEED, true, 0.75, 0.10, 1.5);

  float linear_speed_diff = ROBOT_2_MAX_LINEAR_SPEED - ROBOT_1_MAX_LINEAR_SPEED;

  signal(SIGINT, quit);

  // Wait for the `ros` controller.
  ros::service::waitForService("/robot1/robot/time_step");
  std::cout << "time_step: " << TIME_STEP << std::endl;
  ros::spinOnce();

  robot1.enable_lidar();
  robot1.setWheelPosition(INFINITY, INFINITY);

  robot2.enable_lidar();
  robot2.setWheelPosition(INFINITY, INFINITY);

  while (ros::ok()) {
    ros::spinOnce();

    robot1.calculateAngularVelocity();
    robot1.calculateLinearVelocity(linear_speed_diff);

    robot2.calculateAngularVelocity();
    robot2.calculateLinearVelocity(linear_speed_diff);

    if (robot1.setVelocity()) {
    } else {
        ROS_WARN("Failed to set velocity for 1");
    }
    
     if (robot2.setVelocity()) {
     } else {
         ROS_WARN("Failed to set velocity for 2");
     }
  }
  return 0;
}