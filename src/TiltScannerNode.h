/*
 * TiltScannerNode.h
 *
 *  Created on: 08.12.2016
 *      Author: schaddebe
 */

#ifndef SRC_TILTSCANNERNODE_H_
#define SRC_TILTSCANNERNODE_H_

#include <ros/ros.h>
#include <cmath>
#include <limits>
#include <std_msgs/UInt16.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>
#include <tilt_scanner/MsgSettings.h>
#include <tilt_scanner/SrvSettings.h>


class TiltScannerNode
{
public:

	// --Constructor--
	TiltScannerNode();

	// --Destructor--
	virtual ~TiltScannerNode();

	// --Start--
	void start(void);



private:

	// --Callbacks--

	void run(void);

	void callBackLaserScan(const sensor_msgs::LaserScan& scan);

	void callBackState(const std_msgs::UInt16& state);

	bool callBackService(tilt_scanner::SrvSettings::Request& req, tilt_scanner::SrvSettings::Response& res);


	ros::NodeHandle _nh;


	// --Services--

	ros::ServiceServer _srv;


	// --Subscriber--

	ros::Subscriber _subPointCloud;
	ros::Subscriber _subScan;
	ros::Subscriber _subState;



	// --Publisher--

	ros::Publisher _pubPointCloud;
	ros::Publisher _pubSettings;


	// --Members--

  bool _scanReceived;
  bool _scanFinish;
  bool _scanStart;

  tf::Transform _transform;
  tf::TransformListener _listener;
  tf::TransformBroadcaster _br;

  laser_geometry::LaserProjection _projector;

  sensor_msgs::PointCloud _cloud;
  sensor_msgs::LaserScan _scan;

  tilt_scanner::MsgSettings _settings;
};

#endif /* SRC_TILTSCANNERNODE_H_ */
