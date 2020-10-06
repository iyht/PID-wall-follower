#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <cmath>
#include <limits>


#include "std_msgs/Float32.h"
#include <wall_following_assignment/pid.h>

#include <dynamic_reconfigure/server.h>
#include <wall_following_assignment/PID_parasConfig.h>

ros::Publisher cmd_pub;
ros::Subscriber laser_sub;
ros::Publisher cte_pub;
double desired_distance_from_wall = 1.0; // in meters
double forward_speed = 1.0;              // in meters / sec
PID pid(1.1, 4, 500, 0.1);


void laser_scan_callback(const sensor_msgs::LaserScan::ConstPtr& msg) {
  geometry_msgs::Twist cmd;
  cmd.linear.x = forward_speed;  // forward speed is fixed
    
  // Populate this command based on the distance to the closest
  // object in laser scan. I.e. compute the cross-track error
  // as mentioned in the PID slides.

  // You can populate the command based on either of the following two methods:
  // (1) using only the distance to the closest wall
  // (2) using the distance to the closest wall and the orientation of the wall
  //
  // If you select option 2, you might want to use cascading PID control. 

  // Create the publisher and ROS message
  std_msgs::Float32 laser_msg; 

  // Get the cross-track error
  double min_dist = std::numeric_limits<double>::max();
  int min_dist_index = -1;

  // the range of laser:[-2.35rad:2.35rad] = [-135degree:135degree]
  // we looking for the min distance of left side of the robot about range of 90 degree
  // index 0f -45 degree = 720/(270/90) = 240
  // the range[0:240] is the laser range of [-135degree:-45degree]
  for (int i = 0; i < 240; i++)
  {
    // find the min distance
    if ((msg->ranges[i] >= msg->range_min) && (msg->ranges[i] < msg->range_max)&& (msg->ranges[i] < min_dist))
    {
      min_dist = msg->ranges[i];
      min_dist_index = i;
    }
  }
  laser_msg.data = min_dist- desired_distance_from_wall;
  cte_pub.publish(laser_msg);
  pid.update_control(laser_msg.data);
  cmd.angular.z = pid.get_control();
  cmd_pub.publish(cmd);
}


int main(int argc, char **argv) {
  ros::init(argc, argv, "wall_follower_node");
  ros::NodeHandle nh("~");

  // Getting params before setting up the topic subscribers
  // otherwise the callback might get executed with default
  // wall following parameters
  nh.getParam("forward_speed", forward_speed);
  nh.getParam("desired_distance_from_wall", desired_distance_from_wall);

  // todo: set up the command publisher to publish at topic '/husky_1/cmd_vel'
  // using geometry_msgs::Twist messages
  //cmd_pub = nh.advertise<geometry_msgs::Twist>("/husky_1/husky_velocity_controller/cmd_vel", 1000);
  cmd_pub = nh.advertise<geometry_msgs::Twist>("/husky_1/cmd_vel", 1000);

  cte_pub = nh.advertise<std_msgs::Float32>("/husky_1/cte", 10);
  // todo: set up the laser scan subscriber
  // this will set up a callback function that gets executed
  // upon each spinOnce() call, as long as a laser scan
  // message has been published in the meantime by another node
  laser_sub = nh.subscribe("/husky_1/scan", 10, laser_scan_callback);

  dynamic_reconfigure::Server<wall_following_assignment::PID_parasConfig> server;
  dynamic_reconfigure::Server<wall_following_assignment::PID_parasConfig>::CallbackType f;
  f = boost::bind(&PID::pid_config_callback, &pid, _1, _2);
  server.setCallback(f);

  
  ros::Rate rate(50);
  // this will return false on ctrl-c or when you call ros::shutdown()
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();

    geometry_msgs::Twist msg;
  }
  
  return 0;
}
   
