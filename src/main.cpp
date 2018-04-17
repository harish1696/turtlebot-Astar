/**
 *  MIT License
 *  
 *  Copyright (c) 2017 Harish Sampathkumar
 *  
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to deal
 *  in the Software without restriction, including without limitation the rights
 *  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *  copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *  
 *  The above copyright notice and this permission notice shall be included in all
 *  copies or substantial portions of the Software.
 *  
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *  SOFTWARE.
 */

/**
 *  @file    main.cpp
 *  @author  Harish Sampathkumar
 *  @copyright MIT License
 *
 *  @brief Implementing a simple walker node
 *
 *  @section DESCRIPTION
 *
 *  This program makes the turtle to move forward until it senses
 *  a obstacle(without colliding), then rotate in place until the
 *  way ahead is clear, then move forward again and repeat.
 */
#include <iostream>
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/Empty.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"

int main(int argc, char **argv) {
  // initializing a ros node called walker
  ros::init(argc, argv, "walker");
  
  std::ifstream lookuptable;
  lookuptable.open("/home/harish/turtlebot_gmap/src/turtlebot_spawn/PathVelocity.csv");
  std::vector<std::vector<int>> vel;
  std::string row, column;
  while (std::getline(lookuptable, row)) {
    std::vector<int> temp;
    std::stringstream buffer(row);
    while (std::getline(buffer, column, ',')) {
      std::stringstream temp1(column);
      double val;
      temp1 >> val;
      temp.push_back(val);
    }
    vel.push_back(temp);
    std::cout << "HERE" << std::endl; 
  }

  // creating a node handle
  ros::NodeHandle n;
  geometry_msgs::Pose start_pose;
  start_pose.position.x = -10.0;
  start_pose.position.y = 2.0;
  start_pose.position.z = 0.0;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.0;

  geometry_msgs::Twist start_twist;
  start_twist.linear.x = 0.0;
  start_twist.linear.y = 0.0;
  start_twist.linear.z = 0.0;
  start_twist.angular.x = 0.0;
  start_twist.angular.y = 0.0;
  start_twist.angular.z = 0.0;

  std_srvs::Empty reset;
  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = (std::string) "mobile_base";
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = start_pose;
  modelstate.twist = start_twist;
  
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo_msgs/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);

  ros::Duration(10).sleep();
  ros::service::call("/gazebo/reset_world", reset);

  // Creating a publisher object which publishes to the given topic
  ros::Publisher velocity = n.advertise<geometry_msgs::Twist>
                             ("/mobile_base/commands/velocity", 1000);

  ros::Rate loop_rate(2);

  
  geometry_msgs::Twist msg;

  // Initiaizing msg with 0
  msg.linear.x = 0.0;
  msg.linear.y = 0.0;
  msg.linear.z = 0.0;
  msg.angular.x = 0.0;
  msg.angular.y = 0.0;
  msg.angular.z = 0.0;
  double L = 0.354;
  double r = 0.038;
  for (int i = 0; i < vel.size(); i++) {
    std::cout << vel[i][0] << " " << vel[i][1] << std::endl;
    msg.angular.z = ((vel[i][0] - vel[i][1]) / L) * r;
    msg.linear.x = ((vel[i][0] + vel[i][1]) / 2) * r;
    velocity.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
