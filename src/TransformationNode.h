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
#include <tilt_scanner/MsgSettings.h>

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

	void callBackSettings(const tilt_scanner::MsgSettings& settings);

  ros::NodeHandle _nh;


  // --Services--


  // --Subscriber--

  ros::Subscriber _subCurAngle;
  ros::Subscriber _subSettings;


  // --Publisher--

  ros::Publisher _pubHorizontal;


	// --Members--

  bool _horizontalPosition;

  tf::Transform _transform;
  tf::TransformBroadcaster _br;

  std_msgs::UInt16 _pubhorizontalAngle;
};

#endif /* SRC_TRANSFORMATIONNODE_H_ */
