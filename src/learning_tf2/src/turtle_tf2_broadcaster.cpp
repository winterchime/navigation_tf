// purpose of this broadcaster is to publish changing coordinate frames of the laser_base and main_base to tf2 as the robot moves around

// create a tf2 broadcaster to publish pose of a turtle to tf2
// implement a TransformBroadcaster to publish transforms easier

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
// library for TransformBroadcaster
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

// represent a sequence of characters as an object of the class
// the class is called std::string
// std:string shorten to string resides in the namespace std which contains all C++ standard library functions, classes, and objects

// declare string variable turtle_name
std::string turtle_name;

void poseCallback(const turtlesim::PoseConstPtr& msg){
  // create a TransformBroadcaster object 
  // this object will be used to publish the transformations
  static tf2_ros::TransformBroadcaster br;
  // create a Transform object
  geometry_msgs::TransformStamped transformStamped;
  // assign appropriate meta data to the Transform object 
  // assign a timestamp to the published tranform
  // stamp with current time ros::Time::now()
  transformStamped.header.stamp = ros::Time::now();
  // set name of the parent frame of the link
  transformStamped.header.frame_id = "world";
  // set name of the child frame
  transformStamped.child_frame_id = turtle_name;
  // copy 3-D Pose information into 3-D transform
  transformStamped.transform.translation.x = msg->x;
  transformStamped.transform.translation.y = msg->y;
  transformStamped.transform.translation.z = 0.0;
  tf2::Quaternion q;
  q.setRPY(0, 0, msg->theta);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();
// send a transform using the Transform object
// sendTransform and transformStamped have opposite ordering of parent and child links
  br.sendTransform(transformStamped);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "my_tf2_broadcaster");

  ros::NodeHandle private_node("~");
  if (! private_node.hasParam("turtle"))
  {
    if (argc != 2){ROS_ERROR("need turtle name as argument"); return -1;};
    // assigning value to string variable turtle_name
    turtle_name = argv[1];
  }
  else
  {
    private_node.getParam("turtle", turtle_name);
  }

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe(turtle_name+"/pose", 10, &poseCallback);

  ros::spin();
  return 0;
};