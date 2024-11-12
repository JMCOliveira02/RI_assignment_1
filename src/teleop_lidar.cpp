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
#define MAX_LINEAR_SPEED 10.0
#define MAX_ANGULAR_SPEED 4.0

#define TIME_STEP_SRV "/robot/time_step"

#define ROBOT_1_NAME "robot1"
#define ROBOT_2_NAME "robot2"

#define LIDAR_RESOLUTION 360
#define LIDAR_MIN_ANGLE - M_PI
#define LIDAR_SERVICE "/lidar/enable"
#define LIDAR_TOPIC "/lidar/laser_scan"

#define RIGHT_WHEEL_POS_SERVICE "/right_wheel_motor/set_position"
#define RIGHT_WHEEL_VEL_SERVICE "/right_wheel_motor/set_velocity"

#define LEFT_WHEEL_POS_SERVICE "/left_wheel_motor/set_position"
#define LEFT_WHEEL_VEL_SERVICE "/left_wheel_motor/set_velocity"

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

  ros::ServiceClient lidarEnable_Client;
  webots_ros::set_int lidarEnable_Srv;

  ros::ServiceClient timeStepClient;
  webots_ros::set_int timeStepSrv;

  // Subscribers
  ros::Subscriber lidarSub;

  // Variables
  float v_max, w_max;
  float v, w, theta;

  float desired_angle;
  float angle_correction;

  float b; // Distance between the two wheels(m)
  bool follower; //Defines if robot is of the type follower



public:
  
  float laser_scan[LIDAR_RESOLUTION];
  float min_angle_distance[2];
  float target_wall_distance;
  float wall_distance_tolerance;

  float target_front_distance;
  float front_distance_tolerance;


  Robot(std::string name, float v_max, float w_max, bool follower) {
    this->name = name;
    this->v_max = v_max;
    this->w_max = w_max;

    b = 0.381;
    v = 0;
    w = 0;

    theta = 0;
    desired_angle = M_PI/2;
    angle_correction = 10;

    this->follower = follower;
    leftWheelPos_Client = n.serviceClient<webots_ros::set_float>('/' + name + LEFT_WHEEL_POS_SERVICE);
    rightWheelPos_Client = n.serviceClient<webots_ros::set_float>('/' + name + RIGHT_WHEEL_POS_SERVICE);
    leftWheelVel_Client = n.serviceClient<webots_ros::set_float>('/' + name + LEFT_WHEEL_VEL_SERVICE);
    rightWheelVel_Client = n.serviceClient<webots_ros::set_float>('/' + name + RIGHT_WHEEL_VEL_SERVICE);
    timeStepClient = n.serviceClient<webots_ros::set_int>('/' + name + TIME_STEP_SRV);
    timeStepSrv.request.value = TIME_STEP;
    setVelocity();
    setWheelPosition(INFINITY, INFINITY);
    // lidarSub = n.subscribe('/' + name + LIDAR_TOPIC, 1, &Robot::getLaserScan, this);
  }

/*   void updateStateMachine() {
    
    if (min_angle_distance[1] < target_wall_distance - wall_distance_tolerance){
      //Too close to wall
      current_state = 1;
      theta = desired_angle - angle_correction;
      theta = desired_angle;
    }
    else if (min_angle_distance[1] > target_wall_distance + wall_distance_tolerance){
      //Too far from wall
      current_state = 2;
      theta = desired_angle + angle_correction;
      theta = desired_angle;
    }
    else{
      //Correct distance from wall
      current_state = 3;
      theta = desired_angle;
    }
  } */

  bool check_lidar() {
    sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>('/' + name + LIDAR_TOPIC, n, ros::Duration(1.0)); // timeout after 1 second
    if (!msg) {
      ROS_WARN("LIDAR not responding.");
      enable_lidar();  // Try re-enabling LIDAR
      return false;
    }
    return true;
  }

  void enable_lidar() {
    this->lidarEnable_Client = n.serviceClient<webots_ros::set_int>('/' + name + LIDAR_SERVICE);
    std::cout << "Enabling lidar" << std::endl;  // Debug
    this->lidarEnable_Srv.request.value = TIME_STEP;
    if (this->lidarEnable_Client.call(this->lidarEnable_Srv) && this->lidarEnable_Srv.response.success) {
      std::cout << "Lidar enabled" << std::endl;
    } else {
      std::cout << "Failed to enable lidar" << std::endl;
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
    std::cout << "Setting velocity to " << v << " " << w << std::endl;
    float left_vel = v - w / 2;
    float right_vel = v + w / 2;
    // std::cout << "Setting left velocity to " << left_vel << std::endl;
    // std::cout << "Setting right velocity to " << right_vel << std::endl;
    this->leftWheelVel_Srv.request.value = left_vel;
    this->rightWheelVel_Srv.request.value = right_vel;
    return this->leftWheelVel_Client.call(this->leftWheelVel_Srv) && this->rightWheelVel_Client.call(this->rightWheelVel_Srv) &&
           this->leftWheelVel_Srv.response.success && this->rightWheelVel_Srv.response.success;
  }

  void getLaserScan() {
    std::cout << "Waiting for laser scan" << std::endl;
    sensor_msgs::LaserScan::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::LaserScan>('/' + name + LIDAR_TOPIC);
    for (int i = 0; i < LIDAR_RESOLUTION; i++) {
      laser_scan[i] = msg->ranges[i];
      // std::cout << "Laser scan " << i << ": " << laser_scan[i] << std::endl;
    }
  }

  void getClosestObstacle(float fov_start, float fov_end) {
    // field of view in radians: [fov_start, fov_end]
    float angle_distance[2];
    float min_distance = 1000;
    float min_index = 0;
    float max_index = 0;
    float angle_step = (2*M_PI) / LIDAR_RESOLUTION;
    
    min_index = (int)(fov_start / angle_step);
    max_index = (int)(fov_end / angle_step);

    for (int i = min_index; i <= max_index; i++) {
      if (laser_scan[i] < min_distance) {
        min_distance = laser_scan[i];
        min_index = i;
      }
    }

    // Angle of nearest object in radians
    min_angle_distance[0] = min_index * 2 * M_PI / LIDAR_RESOLUTION;
    std::cout << "Nearest obstacle angle: " << min_angle_distance[0] * 180.0 / M_PI << std::endl;

    // Distance to nearest object
    min_angle_distance[1] = min_distance;

    return;
  }

  float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
  }

  // PID controller for angular velocity
  float Kp_theta = 18.0/(M_PI/2);  // Proportional gain, start small
  //float Ki_w = 0.000005; // integral_w gain, for accumulated error_w correction
  float Ki_theta = 0;
  //float Kd_w = 0.0001;   // derivative_w gain, for rate of change dampening
  float Kd_theta = 0;

  float Kp_distance = 6.0;


  float error_theta = 0;
  float previous_error_theta = 0;
  
  float error_distance = 0;
  float previous_error_distance = 0;
  float integral_w = 0;
  float derivative_w = 0;
  
  void calculateAngularVelocity() {
    // Get closest point  
    getClosestObstacle(0, M_PI);
    // Error between the desired angle of movement and the closest obstacle angle detected
    error_theta = theta - min_angle_distance[0];
    // Error between the desired distance and the 
    error_distance = target_wall_distance - min_angle_distance[1];

    if(abs(error_distance) < wall_distance_tolerance){
      error_distance = 0;
    }
    // Calculate the integral_w and derivative_w components
    //integral_w += error_w;  // Accumulate the integral_w of the error_w
    //derivative_w = error_w - previous_error_w;

    // Update previous error_w for the next cycle
    previous_error_theta = error_theta;
    previous_error_distance = error_distance;


    // PID formula for angular velocity
    w = (Kp_theta * error_theta) - (Kp_distance * error_distance) ;//+ (Ki_w * integral_w) + (Kd_w * derivative_w);
    
    // Clamp the angular velocity to prevent it from becoming too large
    w = clamp(w, -w_max, w_max);

    // Debug output for tracKing PID components
    std::cout << "theta: " << theta * 180.0 /M_PI << "min_angle_distance " << min_angle_distance[0] * 180.0 / M_PI << std::endl;
    ROS_INFO("error_w: %.4f | integral_w: %.4f | derivative_w: %.4f | Angular Velocity (w): %.4f", error_theta * 180.0 / M_PI, integral_w, derivative_w, w);
  }
  
  float error_v = 0;
  float in_target_v = 2.0;

  // PID controller for linear velocity
  float Kp_v = 0;  // Proportional gain, start small


  void calculateLinearVelocity(){
    if(!follower){
      v = 5.0;
      return;
    }

  }
};

void quit(int sig) {
  ROS_INFO("User stopped the 'wall_follower' node.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_follower", ros::init_options::AnonymousName);

  Robot robot1(ROBOT_1_NAME, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, false);
  Robot robot2(ROBOT_2_NAME, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED, true);

  signal(SIGINT, quit);

  // Wait for the `ros` controller.
  ros::service::waitForService("/robot1/robot/time_step");
  std::cout << "time_step: " <<TIME_STEP<< std::endl;
  ros::spinOnce();

  robot1.enable_lidar();
  robot1.setWheelPosition(INFINITY, INFINITY); // Não sei pq mas tive que voltar a fazer isto, caso contrário não funciona
  robot1.target_wall_distance = 0.75;
  robot1.wall_distance_tolerance = 0.10;

  robot2.enable_lidar();
  robot2.setWheelPosition(INFINITY, INFINITY);
  robot2.target_wall_distance = 0.75;
  robot2.wall_distance_tolerance = 0.10;

  // main loop
  while (ros::ok()) {
    ros::spinOnce();

    //if(robot1.check_lidar()) {
    //robot1.setVelocity(1.0, 0.0);
    robot1.getLaserScan();
    //std::cout << "Nearest obstacle angle: " << robot1.min_angle_distance[0] * 180 / M_PI << std::endl;
    std::cout << "Nearest obstacle distance: " << robot1.min_angle_distance[1] << std::endl;

    //robot1.updateStateMachine();

    robot1.calculateAngularVelocity();
    robot1.calculateLinearVelocity();

    // Set wheel velocity
    if (robot1.setVelocity()) {
        ROS_INFO("Velocity set successfully\n");
    } else {
        ROS_WARN("Failed to set velocity");
    }
    //}
  }
  return (0);
}
