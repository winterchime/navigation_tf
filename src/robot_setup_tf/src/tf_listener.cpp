#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>
// a TransformListener object automatically subscribes to the transform message topic over ROS and manages all transform data coming in

// create a function that, given a TransformListener, takes a point in the "base_laser" frame and transforms it to the "base_link" frame
// this function will serve as a callback for the ros::Timer created in the main() of our program and will fire every second
void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  
  // create a point as a geometry_msgs::PointStamped
  // Stamped on the end of the message name means it includes a header allowing association of both a timestamp and a frame_id with the message
  // set the stamp filed of the laser_point message to be ros::Time(), a special time value that allows us to ask the TransformListener for the latest available transform
  geometry_msgs::PointStamped laser_point;

  // frame_id field of the header is set to "base_laser" because we are creating a point in the "base_laser" frame
  laser_point.header.frame_id = "base_laser";

  //we'll just use the most recent transform available for our simple example
  laser_point.header.stamp = ros::Time();

  //just an arbitrary point in space for our point
  laser_point.point.x = 1.0;
  laser_point.point.y = 0.2;
  laser_point.point.z = 0.0;

  // we have the point in the "base_laser" frame
  // we want to transform it into the "base_link" frame
  try{
    geometry_msgs::PointStamped base_point;
    // use TransformListener object, and call transformPoint() with three arguments:
    // 1st, name of the frame we want to transform the point to ("base_link")
    // 2nd, the point we are transforming
    // 3rd, storage for the transformed point
    // after the call to transformPoint(), base_point holds the same information as laser_point did but in the "base_link" frame
    listener.transformPoint("base_link", laser_point, base_point);

    ROS_INFO("base_laser: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        laser_point.point.x, laser_point.point.y, laser_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  }

  // 
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_laser\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(10));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
