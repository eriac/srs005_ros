#include "ros/ros.h"
  
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"

#include "tf/transform_broadcaster.h"

#include "math.h"
#include <sstream>
#include <string>

void odom_callback(const nav_msgs::Odometry& odom_msg){
    static tf::TransformBroadcaster br;

    geometry_msgs::Transform g_tf;
    g_tf.translation.x=odom_msg.pose.pose.position.x;
    g_tf.translation.y=odom_msg.pose.pose.position.y;
    g_tf.translation.z=odom_msg.pose.pose.position.z;
    g_tf.rotation.x=odom_msg.pose.pose.orientation.x;
    g_tf.rotation.y=odom_msg.pose.pose.orientation.y;
    g_tf.rotation.z=odom_msg.pose.pose.orientation.z;
    g_tf.rotation.w=odom_msg.pose.pose.orientation.w;

    tf::Transform transform;
    tf::transformMsgToTF(g_tf,transform);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s5_odom_tf");
	ros::NodeHandle n;

	//Subscribe
	ros::Subscriber odometry = n.subscribe("odom", 10, odom_callback); 
	
	ros::spin();
 	return 0;
}
