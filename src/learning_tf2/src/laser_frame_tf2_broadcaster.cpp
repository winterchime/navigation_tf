#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

// begin main function
// argc, argument count, number of input arguments
// argv, argument vector, arry of values from input arguments
int main(int argc, char** argv){
  // initialize ROS node with argument count, array values, and name of the node
  ros::init(argc, argv, "robot_tf2_broadcaster");
  // create a node
  ros::NodeHandle node;

  // create the transform broadcaster (tfb)
  tf2_ros::TransformBroadcaster tfb;
  // create a new transform, from the parent robot_base to the new child laser_base
  geometry_msgs::TransformStamped transformStamped;

  // parent is robot_base
  transformStamped.header.frame_id = "robot_base";
  // child is laser_base
  transformStamped.child_frame_id = "laser_base";
  transformStamped.transform.translation.x = 2.0;
  // child frame laser_base is 2 meters offset from parent frame robot_base
  transformStamped.transform.translation.y = 2.0;
  transformStamped.transform.translation.z = 2.0;
  
  // create an object q for rotations
  tf2::Quaternion q;
    q.setRPY(0, 0, 0);
  transformStamped.transform.rotation.x = q.x();
  transformStamped.transform.rotation.y = q.y();
  transformStamped.transform.rotation.z = q.z();
  transformStamped.transform.rotation.w = q.w();

  ros::Rate rate(10.0);
  while (node.ok()){
    transformStamped.header.stamp = ros::Time::now();
    tfb.sendTransform(transformStamped);
    rate.sleep();
    printf("sending\n");
  }

};