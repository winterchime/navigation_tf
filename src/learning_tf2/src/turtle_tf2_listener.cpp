// implement a TransformListner to simplify receiving transforms
#include <ros/ros.h>
// include transform_listener library in order to use the TransformListner
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf2_listener");
  ros::NodeHandle node;

  ros::service::waitForService("spawn");
  ros::ServiceClient spawner = node.serviceClient<turtlesim::Spawn>("spawn");
  turtlesim::Spawn turtle;

  turtle.request.x = 4;
  turtle.request.y = 2;
  turtle.request.theta = 0;
  turtle.request.name = "turtle2";
  spawner.call(turtle);
  
  ros::Publisher turtle_vel =
    node.advertise<geometry_msgs::Twist>("turtle2/cmd_vel", 10);
// implement a TransformListner to simplify receiving transforms
// create a buffer for the received tf2 transformations 
  tf2_ros::Buffer tfBuffer;
// create a TransformListener object
// buff the TransformListener object (received tf2 transformation) 

// TransformListener object must be buffed to persist
// otherwise its cache will be empty and every query will fail
// a common solution is to make the TransformListener object a member variable of a class
  tf2_ros::TransformListener tfListener(tfBuffer);

// buffs the TransformListener object up to 10 seconds
  ros::Rate rate(10.0);
    
    while (node.ok()){
      geometry_msgs::TransformStamped transformStamped;
// --real work--
// try-catch block
// query the listener for a specific transformation
    // arg1, target frame
    // arg2, source frame
    // arg3, time at which the transform happens 
        // ros::Time(0) get the latest available transform
    // arg4, duration before time out
// when adding addtional frames, update the new frame here    
    try{
      transformStamped = tfBuffer.lookupTransform("turtle2", "carrot1",ros::Time(0));
    }
// end of new frame update

    catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
      continue;
    }

    geometry_msgs::Twist vel_msg;

    // transformStamped.transform.translation.x = 2.0*sin(ros::Time::now().toSec());
    // transformStamped.transform.translation.y = 2.0*cos(ros::Time::now().toSec());

    vel_msg.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    vel_msg.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));

    // broadcasts transformation
    turtle_vel.publish(vel_msg);

    rate.sleep();
    
  }
  return 0;
};
