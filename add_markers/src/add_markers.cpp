// add_marker.cpp 
// this node is in charge of displaying marker cube in Rviz at pick up and drop off locations 
#include "add_markers.h"

// constructor 
AddMarkers::AddMarkers(ros::NodeHandle* nodehandle, float pickUp_pos[], float dropOff_pos[]):n(*nodehandle), pickUp_pos_(pickUp_pos), dropOff_pos_(dropOff_pos)
{
  ROS_INFO("in class constructor of AddMarker");
  initialize_subscribers();
  initialize_publishers();
}

// initialize subscriber 
void AddMarkers::initialize_subscribers()
{
  ROS_INFO("Initializing Subscribers");
  sub1 = n.subscribe("/robot_delivery", 10, &AddMarkers::robot_delivery_callback, this);
  ROS_INFO("Subscribed to robot_delivery");
}

// initialize publishers
void AddMarkers::initialize_publishers()
{
  ROS_INFO("Initializing Publishers");
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

}

// actions to display the marker 
void AddMarkers::display_marker(float pos[])
{
  shape = visualization_msgs::Marker::CUBE;
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "cube";
  marker.id = 0;
  // Set the marker type.  Initially this is CUBE
  marker.type = shape;

  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = pos[0];
  marker.pose.position.y = pos[1];
  marker.pose.position.z = pos[2];
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1;
  // set the scale of the shape
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  // set the color of the shape 
  marker.color.r = 0.0f;
  marker.color.g = 1.0f;
  marker.color.b = 0.0f;
  marker.color.a = 1.0;

  marker.lifetime = ros::Duration();


  // Publish the marker
  while (marker_pub.getNumSubscribers() < 1)
  {
    ROS_WARN_ONCE("Please create a subscriber to the marker");
    sleep(1);
  }

  // publish the first marker
  ROS_INFO("Publishing Marker at Location");
  marker_pub.publish(marker);
}

void AddMarkers::hide_marker()
{
  visualization_msgs::Marker marker;
  // Set the frame ID and timestamp
  marker.header.frame_id = "/map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "cube";
  marker.id = 0;
  // Set the marker type.  Initially this is CUBE
  marker.type = shape;
  // hide the marker 
  ROS_INFO("Marker Deleted");
  marker.action = visualization_msgs::Marker::DELETE;
  marker_pub.publish(marker);
}

// robot delivery subscription call back 
void AddMarkers::robot_delivery_callback(const std_msgs::String::ConstPtr& msg)
{
        if (msg->data == "pickup") {
          ROS_INFO("The robot is at pickup");
          hide_marker();
        } else if (msg->data == "moving to pickup") {
            display_marker(pickUp_pos_);
            ROS_INFO("The robot is moving to pickup");
        } else if (msg->data == "moving to dropoff") {
            ROS_INFO("The robot is moving to dropoff");
        } else if (msg->data == "dropoff") {
            display_marker(dropOff_pos_);
            ROS_INFO("The robot is at dropoff");
        }
}


int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  float pickUp_pos[3] = {4.5, -1.5, 0.010};
  float dropOff_pos[3] = {3, 4, 0.010};
  ROS_INFO("main: instantiating an object of type AddMarker");
  AddMarkers AddMarkers(&n, pickUp_pos, dropOff_pos); // instantiate a AddMarkers object type and pass in the node handle pointer for the constructor to use 
  ros::spin();
  return 0;
}
