#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  // create a TransformBroadcaster object that will be used later to send the base_link to base_laser
  tf::TransformBroadcaster broadcaster;

  while(n.ok()){
  // sending a transform with a TransformBraodcaster requires five arguments
  // 1st, rotation transform specified by a btQuaternion for any rotation that needs to occur between the two coordinate frames.
  // constructed from pitch, roll, and yaw values
  // 2nd, btVector3 for any translation. laser offset
  // 3rd, give the transform being published a timestamp. stamp it with ros::Time::now(), current time
  // 4th, pass the name of the parent node of the link we are creating, in this case "base_link"
  // 5th, pass the name of the child node of the link we are creating, in this case "base_laser" 
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "base_laser"));
    r.sleep();
  }
}
