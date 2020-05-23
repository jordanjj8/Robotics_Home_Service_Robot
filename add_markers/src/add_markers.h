// add_markers.h header file
// include this file in "add_markers.cpp"

#ifndef ADD_MARKERS_CLASS_H
#define ADD_MARKERS_CLASS_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "std_msgs/String.h"

class AddMarkers
{
public:
	AddMarkers(ros::NodeHandle* nodehandle, float pickUp_pos[], float dropOff_pos[]); // "main" needs to instantiate a ROS nodehandle and pass the pointer to the constructor 

private: 
	// private data only available to member functions of this class
	ros::NodeHandle n; 
	ros::Subscriber sub1;
	ros::Publisher marker_pub;
	float *pickUp_pos_; 
	float *dropOff_pos_;
	uint32_t shape;

	// member methods
	void initialize_subscribers();
	void initialize_publishers();
    void display_marker(float pos[]);
    void hide_marker();
	void robot_delivery_callback(const std_msgs::String::ConstPtr& msg);

}; 

#endif // to match with #ifndef

