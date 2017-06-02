/*
 * transformation_node.cpp
 *
 * Created on: 18.01.2017
 *     Author: schaddebe
 */

#include <ros/ros.h>

#include "TransformationNode.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "transformation_node");
  TransformationNode heiko;
  heiko.start();
}
