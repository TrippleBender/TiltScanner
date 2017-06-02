/*
 * TransformationNode.h
 *
 *  Created on: 16.01.2017
 *      Author: schaddebe
 */

#ifndef SRC_TRANSFORMATIONNODE_H_
#define SRC_TRANSFORMATIONNODE_H_

#include <ros/ros.h>
#include <std_msgs/UInt16.h>
#include <tf/transform_broadcaster.h>

class TransformationNode
{
public:

  // Constructor
  TransformationNode();

  // Destructor
  virtual ~TransformationNode();

  // Start
  void start(void);



private:

  void run(void);

  void callBackAngle(const std_msgs::UInt16& angle);

  ros::NodeHandle _nh;

  // --Subscriber--

  ros::Subscriber _subCurAngle;


  // --Publisher--



  // --CallBack--

  tf::Transform _transform;
  tf::TransformBroadcaster _br;

};

#endif /* SRC_TRANSFORMATIONNODE_H_ */
