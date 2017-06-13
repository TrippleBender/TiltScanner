/*
 * TiltScannerNode.cpp
 *
 *  Created on: 08.12.2016
 *      Author: schaddebe
 */

#include "TiltScannerNode.h"

//Dynamixel Values

#define MaxAngle 230
#define MinAngle 90
#define MaxSpeed 500


#define PI 3.14159265
#define MotorIncrement 0.0879


TiltScannerNode::TiltScannerNode()
{
  _scanReceived = false;
  _scanFinish = false;
  _scanStart = false;

  // -- Service --
  _srv = _nh.advertiseService("settingsSrv", &TiltScannerNode::callBackService, this);

  // -- Subscriber --
  _subScan = _nh.subscribe("scan", 1, &TiltScannerNode::callBackLaserScan, this);
  _subState = _nh.subscribe("state", 1, &TiltScannerNode::callBackState, this);


  // -- Publisher --
  _pubPointCloud = _nh.advertise<sensor_msgs::PointCloud>("subCloud", 1);
  _pubSettings = _nh.advertise<tilt_scanner::MsgSettings>("settings", 1);
}



TiltScannerNode::~TiltScannerNode()
{

}



void TiltScannerNode::start(void)
{
  this->run();
}



void TiltScannerNode::run(void)
{
  ros::Rate loop_Rate(15);                                                            //frequency 15Hz
  _cloud.header.frame_id = "laser";
  clock_t beginTime = clock();


  while(!_scanReceived && ros::ok())                                                  //waiting for Setupdata of laserscanner
  {
    ros::spinOnce();
    loop_Rate.sleep();

    if(clock()-beginTime > 5000)
    {
      ROS_ERROR_STREAM("No Laserscan received!!");
      beginTime = clock();
    }

    continue;
  }

  _scanReceived = false;

  std::cout << __PRETTY_FUNCTION__ << " laser data:\n\tmin: " << _scan.angle_min << "\n\tmax: " << _scan.angle_max
      << "\n\tranges size: " << _scan.ranges.size() <<"\n\tangle increment: " << _scan.angle_increment
      << "\n\ttime increment: " << _scan.time_increment <<"\n\tscan time: " << _scan.scan_time <<std::endl;


  while(ros::ok())
  {
    sensor_msgs::PointCloud cloud;
    const unsigned int r = 102;                                                      //[mm] length of the lever

    ros::spinOnce();

    if (_scanReceived && _scanStart)
    {
      beginTime = clock();

      if(!_listener.waitForTransform(                                               //wait for transformation
          _scan.header.frame_id,
          "motor",
          _scan.header.stamp + ros::Duration().fromSec(_scan.ranges.size()*_scan.time_increment),
          ros::Duration(2)))
      {

        if(clock()-beginTime > 30000)
        {
          ROS_ERROR_STREAM("motor to laser tf not available");
        }

        continue;
      }

      try
      {
        _projector.transformLaserScanToPointCloud("motor", _scan, cloud, _listener);
        for(unsigned int i = 0; i < cloud.points.size(); i++)
              {
                _cloud.points.push_back(cloud.points.at(i));
              }
        std::cout << cloud.header.frame_id << std::endl;
      }

      catch (tf::TransformException& ex)
      {

        ROS_ERROR("%s",ex.what());
      }

      if(_scanFinish)
      {
    	  cloud.points = _cloud.points;
    	  _pubPointCloud.publish(cloud);
    	  _scanFinish = false;
    	  _scanStart = false;
      }
      _scanReceived = false;
    }
  }

  loop_Rate.sleep();
}


bool TiltScannerNode::callBackService(tilt_scanner::SrvSettings::Request& req, tilt_scanner::SrvSettings::Response& res)
{
  unsigned int speed = 0;
  float startPosition = 0.00;                                         //unit is degree
  float endPosition = 0.00;
  float unit = 1/MotorIncrement;

  _cloud.points.clear();                                              //set PointCloud back

  speed = req.speed;																									//take over settings
  startPosition = req.startPosition;
  endPosition = req.endPosition;

  if(speed == 0 || speed > MaxSpeed  || startPosition < MinAngle || startPosition > MaxAngle ||
     endPosition > MaxAngle || endPosition <= startPosition)           																 //check the input
  {
    ROS_ERROR_STREAM("wrong settings");

    if(speed == 0 || speed > MaxSpeed)
    {
      ROS_INFO_STREAM("change the value 'speed'!");
    }

    if(startPosition < MinAngle || startPosition > MaxAngle)
    {
      ROS_INFO_STREAM("change the value 'startPosition'!");
    }

    if(endPosition > MaxAngle || endPosition <= startPosition)
    {
      ROS_INFO_STREAM("change the value 'endPosition'!");
    }

    return false;
  }

  else
  {
    startPosition = startPosition * unit;                             //convert into the unit of the Dynamixel MX28-T and send them to the Arduino
    endPosition = endPosition * unit;

    _settings.speed = speed;
    _settings.startPosition = (uint16_t) round (startPosition);       //the casting is every time possible, because of the "if" startPosition can't be huger than 230
    _settings.endPosition = (uint16_t) round (endPosition);

    _pubSettings.publish(_settings);

    std::cout << __PRETTY_FUNCTION__ << "scan data:\n\\speed: " << speed << "\n\\startPosition: " << startPosition << "\n\endPosition: " << endPosition <<std::endl;
    return true;
  }
}

void TiltScannerNode::callBackLaserScan(const sensor_msgs::LaserScan& scan)
{
  _scanReceived = true;
  _scan = scan;
}

void TiltScannerNode::callBackState(const std_msgs::UInt16& state)
{
	const unsigned int scanStart = 3;
  const unsigned int scanFinish = 4;

  if(state.data == scanFinish)
  {
    _scanFinish = true;
  }

  if(state.data == scanStart)
  {
  	_scanStart = true;
  }
}
