// Copyright 1996-2023 Cyberbotics Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// #### teleop_lidar
// This controller is a simple teleoperation controller that allows the user to move the robot using the keyboard, and also
// take snapshots of the environment using the lidar sensor.

#include "ros/ros.h"

#include <webots_ros/Int32Stamped.h>

#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/robot_get_device_list.h>

#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32
#define MAX_L_SPEED 10.0
#define MAX_W_SPEED 2.0

static double linear_vel = 0;
static double angular_vel = 0;

ros::ServiceClient leftWheelPos_Client;
webots_ros::set_float leftWheelPos_Srv;

ros::ServiceClient rightWheelPos_Client;
webots_ros::set_float rightWheelPos_Srv;

ros::ServiceClient leftWheelVel_Client;
webots_ros::set_float leftWheelVel_Srv;

ros::ServiceClient rightWheelVel_Client;
webots_ros::set_float rightWheelVel_Srv;

ros::ServiceClient lidarClient;
webots_ros::set_float lidarSrv;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

ros::ServiceClient enableKeyboardClient;
webots_ros::set_int enableKeyboardSrv;

void quit(int sig) {
  enableKeyboardSrv.request.value = 0;
  enableKeyboardClient.call(enableKeyboardSrv);
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("User stopped the 'keyboard_teleop' node.");
  ros::shutdown();
  exit(0);
}

void keyboardCallback(const webots_ros::Int32Stamped::ConstPtr &value) {
  int key = value->data;
  int send = 0;

  switch (key) {
    case 314:
      angular_vel = -MAX_W_SPEED;
      send = 1;
      break;
    case 316:
      angular_vel = MAX_W_SPEED;
      send = 1;
      break;
    case 312:
      ROS_INFO("END.");
      quit(-1);
      break;
    default:
      angular_vel = 0;
      send = 1;
      break;
  }

  switch (key) {
    case 315:
      linear_vel = MAX_L_SPEED;
      send = 1;
      break;
    case 317:
      linear_vel = -MAX_L_SPEED;
      send = 1;
      break;
    default:
      linear_vel = 0;
      send = 1;
      break;
  }

  leftWheelVel_Srv.request.value = linear_vel + angular_vel / 2;
  rightWheelVel_Srv.request.value = linear_vel - angular_vel / 2;

  if (send) {
    if (!leftWheelVel_Client.call(leftWheelVel_Srv) || !rightWheelVel_Client.call(rightWheelVel_Srv) ||
        !leftWheelVel_Srv.response.success || !rightWheelVel_Srv.response.success)
      ROS_ERROR("Failed to send new position commands to the robot.");
  }
  return;
}

int main(int argc, char **argv) {
  // create a node named 'keyboard_teleop' on ROS network
  ros::init(argc, argv, "keyboard_teleop", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  signal(SIGINT, quit);

  // Wait for the `ros` controller.
  ros::service::waitForService("/robot/time_step");
  ros::spinOnce();
  leftWheelPos_Client = n.serviceClient<webots_ros::set_float>("/left_wheel_motor/set_position");
  rightWheelPos_Client = n.serviceClient<webots_ros::set_float>("/right_wheel_motor/set_position");
  leftWheelPos_Srv.request.value = INFINITY;
  rightWheelPos_Srv.request.value = INFINITY;
  leftWheelPos_Client.call(leftWheelPos_Srv);
  rightWheelPos_Client.call(rightWheelPos_Srv);

  leftWheelVel_Client = n.serviceClient<webots_ros::set_float>("/left_wheel_motor/set_velocity");
  rightWheelVel_Client = n.serviceClient<webots_ros::set_float>("/right_wheel_motor/set_velocity");
  timeStepClient = n.serviceClient<webots_ros::set_int>("/robot/time_step");

  timeStepSrv.request.value = TIME_STEP;

  enableKeyboardClient = n.serviceClient<webots_ros::set_int>("/keyboard/enable");
  enableKeyboardSrv.request.value = TIME_STEP;
  if (enableKeyboardClient.call(enableKeyboardSrv) && enableKeyboardSrv.response.success) {
    ros::Subscriber sub_keyboard;
    sub_keyboard = n.subscribe("/keyboard/key", 1, keyboardCallback);
    while (sub_keyboard.getNumPublishers() == 0) {
    }
    ROS_INFO("Keyboard enabled.");
    ROS_INFO("Use the arrows in Webots window to move the robot.");
    ROS_INFO("Press the End key to stop the node.");

    // main loop
    while (ros::ok()) {
      ros::spinOnce();
      if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success)
        ROS_ERROR("Failed to call service time_step for next step.");
    }
  } else
    ROS_ERROR("Could not enable keyboard, success = %d.", enableKeyboardSrv.response.success);

  enableKeyboardSrv.request.value = 0;
  if (!enableKeyboardClient.call(enableKeyboardSrv) || !enableKeyboardSrv.response.success)
    ROS_ERROR("Could not disable keyboard, success = %d.", enableKeyboardSrv.response.success);
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ros::shutdown();
  return (0);
}
