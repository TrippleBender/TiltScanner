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
#include <tilt_scanner/SrvSettings.h>

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


  // --CallBacks--

  void callBackAngle(const std_msgs::UInt16& angle);

	bool callBackService(tilt_scanner::SrvSettings::Request& req, tilt_scanner::SrvSettings::Response& res);

  ros::NodeHandle _nh;


  // --Services--

  ros::ServiceServer _srv;


  // --Subscriber--

  ros::Subscriber _subCurAngle;


	// --Members--

  bool _horizontalPosition;

  tf::Transform _transform;
  tf::TransformBroadcaster _br;
};

#endif /* SRC_TRANSFORMATIONNODE_H_ */
