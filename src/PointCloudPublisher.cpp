#include "ros/ros.h"

#include <webots_ros/Int32Stamped.h>

#include <webots_ros/lidar_get_layer_point_cloud.h>
#include <webots_ros/set_bool.h>
#include <webots_ros/set_float.h>
#include <webots_ros/set_int.h>

#include <webots_ros/robot_get_device_list.h>

#include <std_msgs/String.h>

#include <signal.h>
#include <stdio.h>

#define TIME_STEP 32
// ros::Rate r(10);

ros::ServiceClient lidarEnableClient;
webots_ros::set_int lidarEnableSrv;

ros::ServiceClient lidarEnablePointCloudClient;
webots_ros::set_bool lidarEnablePointCloudSrv;

ros::ServiceClient lidarGetPointCloudLayerClient;
webots_ros::lidar_get_layer_point_cloud lidarGetPointCloudLayerSrv;

ros::ServiceClient timeStepClient;
webots_ros::set_int timeStepSrv;

void quit(int sig) {
  lidarEnablePointCloudSrv.request.value = false;
  lidarEnablePointCloudClient.call(lidarEnablePointCloudSrv);
  lidarEnableSrv.request.value = 0;
  lidarEnableClient.call(lidarEnableSrv);
  timeStepSrv.request.value = 0;
  timeStepClient.call(timeStepSrv);
  ROS_INFO("User stopped the PointCloudPublisher node.");
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "PoinCloudPublisher", ros::init_options::AnonymousName);
  ros::NodeHandle n;

  signal(SIGINT, quit);

  ros::service::waitForService("/robot/time_step");
  ros::spinOnce();

  lidarEnableClient = n.serviceClient<webots_ros::set_int>("/lidar/enable");
  lidarEnablePointCloudClient = n.serviceClient<webots_ros::set_bool>("/lidar/enable_point_cloud");
  lidarGetPointCloudLayerClient = n.serviceClient<webots_ros::lidar_get_layer_point_cloud>("/lidar/get_layer_point_cloud");
  timeStepClient = n.serviceClient<webots_ros::set_int>("/robot/time_step");

  lidarEnableSrv.request.value = TIME_STEP;
  if (!lidarEnableClient.call(lidarEnableSrv) || !lidarEnableSrv.response.success) {
    ROS_ERROR("Failed to enable lidar, success = %d.", lidarEnableSrv.response.success);
    return 1;
  }

  lidarEnablePointCloudSrv.request.value = true;
  if (!lidarEnablePointCloudClient.call(lidarEnablePointCloudSrv) || !lidarEnablePointCloudSrv.response.success) {
    ROS_ERROR("Failed to enable lidar point cloud, success = %d.", lidarEnablePointCloudSrv.response.success);
    return 1;
  }

  ros::Publisher PointCloudPublisher = n.advertise<sensor_msgs::PointCloud>("/lidar/point_cloud", 1);

  timeStepSrv.request.value = TIME_STEP;
  while (ros::ok()) {
    ros::spinOnce();
    if (!timeStepClient.call(timeStepSrv) || !timeStepSrv.response.success) {
      ROS_ERROR("Failed to call service time_step for next step.");
    }

    lidarGetPointCloudLayerSrv.request.layer = 0;
    if (!lidarGetPointCloudLayerClient.call(lidarGetPointCloudLayerSrv)) {
      ROS_ERROR("Failed to get point cloud layer");
    }

    sensor_msgs::PointCloud pointCloudMsg;
    pointCloudMsg.header.stamp = ros::Time::now();
    pointCloudMsg.header.frame_id = "lidar_frame";
    pointCloudMsg.points = lidarGetPointCloudLayerSrv.response.pointCloud.points;
    PointCloudPublisher.publish(pointCloudMsg);

        std::cout << "Got point cloud layer" << std::endl;
    std::cout << "Example points: " << std::endl << lidarGetPointCloudLayerSrv.response.pointCloud.points[0] << std::endl;

    // r.sleep();
  }
}
