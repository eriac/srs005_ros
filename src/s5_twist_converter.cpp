#include "ros/ros.h"

#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Twist.h"


ros::Publisher twist_pub;
void twist_callback(const geometry_msgs::TwistStamped& twist_msg){
  twist_pub.publish(twist_msg.twist);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "omni_driver");
  ros::NodeHandle n;

  //publish
  twist_pub = n.advertise<geometry_msgs::Twist>("twist_out", 10);

  //Subscribe
  ros::Subscriber twist_sub = n.subscribe("twist_in", 10, twist_callback); 

  ros::spin();
  return 0;
}

