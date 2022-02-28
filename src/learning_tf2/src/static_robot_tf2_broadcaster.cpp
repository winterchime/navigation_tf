// publish static reference frame to tf2
// broadcast the changing coordinate frames of the robot, as it moves around

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
// provide access to input and output functions in C++
#include <cstdio>
#include <tf2/LinearMath/Quaternion.h>

// declare a variable named static_robot_name
// assign the variable with a value of an empty string
std::string static_robot_name;

// begin the main function
// argc, arg count, number of input arguments
// argv, arg vector, array of input arguments
// zero indexed array, index starts at zero

int main(int argc, char **argv)
{
  // initialize ROS with the input arguments and name of the node
  ros::init(argc,argv, "my_static_tf2_broadcaster");
  // if there is not 8 input arguments
  if(argc != 8)
  {
    // errors out, must have 8 input arguments
    ROS_ERROR("Invalid number of parameters\nusage: static_robot_tf2_broadcaster child_frame_name x y z roll pitch yaw");
    // exit program with the error code
    return -1;
  }
  // 1st input argument must not be world
  // argv[1] is name of the robot
  if(strcmp(argv[1],"world")==0)
  {
    ROS_ERROR("Your static robot name cannot be 'world'");
    return -1;

  }
  // assigning 2nd input argument value to declared variable
  static_robot_name = argv[1];

  // create a StaticTransformBroadcaster variable
  // we will use this variable to send transformations to tf2
  static tf2_ros::StaticTransformBroadcaster static_broadcaster;
  // initialize values of transformations that will be sent to tf2 using the StaticTransformBroadcaster
  geometry_msgs::TransformStamped static_transformStamped;

  // header will contain the metadata 
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "world";
  static_transformStamped.child_frame_id = static_robot_name;
  static_transformStamped.transform.translation.x = atof(argv[2]);
  static_transformStamped.transform.translation.y = atof(argv[3]);
  static_transformStamped.transform.translation.z = atof(argv[4]);
  tf2::Quaternion quat;
  quat.setRPY(atof(argv[5]), atof(argv[6]), atof(argv[7]));
  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();

  // send the transform using sendTransform function of the StaticTransformBroadcaster 
  static_broadcaster.sendTransform(static_transformStamped);

  ROS_INFO("Spinning until killed publishing %s to world", 
  static_robot_name.c_str());
  ros::spin();
  return 0;
};