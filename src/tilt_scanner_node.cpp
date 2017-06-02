/*
 * tilt_scanner_node.cpp
 *
 *  Created on: 09.12.2016
 *      Author: schaddebe
 */

#include <ros/ros.h>

#include "TiltScannerNode.h"

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "tilt_scanner_node");
	TiltScannerNode heini;
	heini.start();

	//TiltScannerNode.start();
}


