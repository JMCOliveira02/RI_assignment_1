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
#define MAX_ANGULAR_SPEED 2.0

#define TIME_STEP_SRV "/robot/time_step"

#define ROBOT_1_NAME "/robot1"
#define ROBOT_2_NAME "/robot2"

#define LIDAR_RESOLUTION 50
#define LIDAR_SERVICE "/lidar/enable"
#define LIDAR_TOPIC "/lidar/laser_scan"

#define RIGHT_WHEEL_POS_SERVICE "/right_wheel/set_position"
#define RIGHT_WHEEL_VEL_SERVICE "/right_wheel/set_velocity"

#define LEFT_WHEEL_POS_SERVICE "/left_wheel/set_position"
#define LEFT_WHEEL_VEL_SERVICE "/left_wheel/set_velocity"

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
  webots_ros::set_float lidarEnable_Srv;

  ros::ServiceClient timeStepClient;
  webots_ros::set_int timeStepSrv;

  // Subscribers
  ros::Subscriber lidarRightSub;
  ros::Subscriber lidarFrontSub;

  // Variables
  float v_max, w_max;
  float v, w, theta;
  float laser_scan[LIDAR_RESOLUTION];

public:
  Robot(std::string name, float v_max, float w_max) {
    this->name = name;
    this->v_max = v_max;
    this->w_max = w_max;
    v = 0;
    w = 0;
    theta = 0;
    leftWheelPos_Client = n.serviceClient<webots_ros::set_float>('/' + name + LEFT_WHEEL_POS_SERVICE);
    rightWheelPos_Client = n.serviceClient<webots_ros::set_float>('/' + name + RIGHT_WHEEL_POS_SERVICE);
    leftWheelVel_Client = n.serviceClient<webots_ros::set_float>('/' + name + LEFT_WHEEL_VEL_SERVICE);
    rightWheelVel_Client = n.serviceClient<webots_ros::set_float>('/' + name + RIGHT_WHEEL_VEL_SERVICE);
    timeStepClient = n.serviceClient<webots_ros::set_int>('/' + name + TIME_STEP_SRV);
    timeStepSrv.request.value = TIME_STEP;
    setVelocity(0, 0);
    setWheelPosition(INFINITY, INFINITY);

    lidarRightSub = n.subscribe('/' + name + LIDAR_TOPIC, 1, &Robot::updateLaserScan, this);
  }

  bool enable_lidars() {
    this->lidarEnable_Client = n.serviceClient<webots_ros::set_float>('/' + name + LIDAR_SERVICE);
    this->lidarEnable_Srv.request.value = TIME_STEP;
    return this->lidarEnable_Client.call(this->lidarEnable_Srv) && this->lidarEnable_Srv.response.success;
  }

  bool setWheelPosition(float left_pos, float right_pos) {
    this->leftWheelPos_Srv.request.value = left_pos;
    this->rightWheelPos_Srv.request.value = right_pos;
    return this->leftWheelPos_Client.call(this->leftWheelPos_Srv) && this->rightWheelPos_Client.call(this->rightWheelPos_Srv) &&
           this->leftWheelPos_Srv.response.success && this->rightWheelPos_Srv.response.success;
  }

  bool setVelocity(float linear_vel, float angular_vel) {
    this->leftWheelVel_Srv.request.value = linear_vel + angular_vel / 2;
    this->rightWheelVel_Srv.request.value = linear_vel - angular_vel / 2;
    return this->leftWheelVel_Client.call(this->leftWheelVel_Srv) && this->rightWheelVel_Client.call(this->rightWheelVel_Srv) &&
           this->leftWheelVel_Srv.response.success && this->rightWheelVel_Srv.response.success;
  }

  void updateLaserScan(const sensor_msgs::LaserScan::ConstPtr &msg) {
    for (int i = 0; i < LIDAR_RESOLUTION; i++) {
      laser_scan[i] = msg->ranges[i];
    }
  }
};

void quit(int sig) {
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("User stopped the 'wall_follower' node.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_follower", ros::init_options::AnonymousName);

  Robot robot1(ROBOT_1_NAME, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);
  Robot robot2(ROBOT_2_NAME, MAX_LINEAR_SPEED, MAX_ANGULAR_SPEED);

  signal(SIGINT, quit);

  // Wait for the `ros` controller.
  ros::service::waitForService("/robot/time_step");
  ros::spinOnce();

  // main loop
  while (ros::ok()) {
    ros::spinOnce();
  }
  return (0);
}
