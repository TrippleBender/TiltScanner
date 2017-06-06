/*
 * TransformationNode.cpp
 *
 *  Created on: 16.01.2017
 *      Author: schaddebe
 */

#include "TransformationNode.h"

//Dynamixel Value
#define MaxAngle 3100
#define MinAngle 1000

#define MotorIncrement 0.0879
#define PI 3.14159265

TransformationNode::TransformationNode()
{
	_horizontalPosition = false;

	// -- Service --


  // --Subscriber--
  _subCurAngle = _nh.subscribe("pitch", 1, &TransformationNode::callBackAngle, this);
  _subSettings = _nh.subscribe("settings", 1, &TransformationNode::callBackSettings, this);

  // --Publisher--
  _pubHorizontal = _nh.advertise<std_msgs::UInt16>("horizontal", 1);
}

TransformationNode::~TransformationNode()
{
  // TODO Auto-generated destructor stub
}

void TransformationNode::start(void)
{
  this -> run();
}

void TransformationNode::run(void)
{
  ros::spin();
}


void TransformationNode::callBackAngle (const std_msgs::UInt16& pitch)
{
  const float unit = MotorIncrement*PI/180;                               //unit of the increments and the horizontal Position of the Dynamixel MX28-T
  unsigned int horizontal = 0;
  float r = 0.103;                                                        //[m] length of the lever
  float motorAngle = 0.00;

  if(_horizontalPosition)																									//use the current horizontal angle of pitch, if received
  {
  	horizontal = pitch.data;
    _horizontalPosition = false;

    _pubhorizontalAngle.data = horizontal;
    _pubHorizontal.publish(_pubhorizontalAngle);
  }

  if(pitch.data<MinAngle || pitch.data>MaxAngle)                         	//control the received angle
  {
    std::cout << "incorrect angle of the motor received: " << pitch.data << std::endl;
    return;
  }

  motorAngle = (int)(pitch.data-horizontal) * unit;

  tf::Transform translation;                                              //transformation with translation and rotation
        translation.setOrigin( tf::Vector3(0.0, 0.0, -r) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        translation.setRotation(q);


  tf::Transform rotation;
        rotation.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
        q.setRPY(0, motorAngle, 0);
        rotation.setRotation(q);

  _transform = rotation * translation;

  _br.sendTransform(tf::StampedTransform(_transform, ros::Time::now(), "motor", "laser"));     //broadcast transformation
}


void TransformationNode::callBackSettings(const tilt_scanner::MsgSettings& settings)
{
	_horizontalPosition = true;
}
