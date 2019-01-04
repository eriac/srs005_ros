#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Transform.h"

#include "tf/transform_broadcaster.h"

ros::Publisher pose_pub1;
ros::Publisher pose_pub2;
void poseCallback(const geometry_msgs::PoseStamped msg)
{
/*
  tf::Transform tf_test1;
  tf_test1.setOrigin (tf::Vector3(
    0,
    0,
    0));
  tf_test1.setRotation (tf::Quaternion(
    0,
    -0.70710678,
    0,
    0.70710678));

  tf::Transform tf_test2;
  tf_test2.setOrigin (tf::Vector3(
    0,
    0,
    0));
  tf_test2.setRotation (tf::Quaternion(
    -0.70710678,
    0,
    0,
    0.70710678));

  tf::Transform ans=tf_test1*tf_test2;

  geometry_msgs::Transform ans_g_tf;
  tf::transformTFToMsg (ans, ans_g_tf); 	

  printf("x:%f, y:%f, z:%f\n",
    (float)ans_g_tf.translation.x,
    (float)ans_g_tf.translation.y,
    (float)ans_g_tf.translation.z);
  printf("x:%f, y:%f, z:%f, w:%f\n",
    (float)ans_g_tf.rotation.x,
    (float)ans_g_tf.rotation.y,
    (float)ans_g_tf.rotation.z,
    (float)ans_g_tf.rotation.w);
*/

  tf::Transform a;
  a.setOrigin (tf::Vector3(
    msg.pose.position.x,
    msg.pose.position.y,
    msg.pose.position.z));
  a.setRotation (tf::Quaternion(
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w));

  tf::Transform world_tf;
  world_tf.setOrigin (tf::Vector3(
    0,
    0,
    0));
  world_tf.setRotation (tf::Quaternion(
    0.70710678,
    0,
    0,
    0.70710678));

  tf::Transform robot_tf;
  robot_tf.setOrigin (tf::Vector3(
    0.08,
    0,
    0));
  robot_tf.setRotation (tf::Quaternion(
    -0.5,
    -0.5,
    -0.5,
    0.5));


  tf::Transform b=world_tf*a*robot_tf;
  geometry_msgs::Transform g_tf;
  tf::transformTFToMsg (b, g_tf); 	
/*
  printf("x:%f, y:%f, z:%f\n",
    (float)b.getOrigin().getX(),
    (float)b.getOrigin().getY(),
    (float)b.getOrigin().getZ());
  printf("x:%f, y:%f, z:%f, w:%f\n",
    (float)b.getRotation().getAxis().getX(),
    (float)b.getRotation().getAxis().getY(),
    (float)b.getRotation().getAxis().getZ(),
    (float)b.getRotation().getW());
*/
  geometry_msgs::PoseStamped tmp_pose;
  tmp_pose.header.frame_id="map";
  tmp_pose.header.stamp=ros::Time::now();
  tmp_pose.pose.position.x=g_tf.translation.x;
  tmp_pose.pose.position.y=g_tf.translation.y;
  tmp_pose.pose.position.z=g_tf.translation.z;
  tmp_pose.pose.orientation.x=g_tf.rotation.x;
  tmp_pose.pose.orientation.y=g_tf.rotation.y;
  tmp_pose.pose.orientation.z=g_tf.rotation.z;
  tmp_pose.pose.orientation.w=g_tf.rotation.w;

  pose_pub1.publish(tmp_pose);

/*
  tf::Transform c=(world_tf*a)*robot_tf1;
  tf::transformTFToMsg()


  tmp_pose.header.frame_id="map";
  tmp_pose.header.stamp=ros::Time::now();
  tmp_pose.pose.position.x=c.getOrigin().getX();
  tmp_pose.pose.position.y=c.getOrigin().getY();
  tmp_pose.pose.position.z=c.getOrigin().getZ();
  tmp_pose.pose.orientation.x=c.getRotation().getAxis().getX();
  tmp_pose.pose.orientation.y=c.getRotation().getAxis().getY();
  tmp_pose.pose.orientation.z=c.getRotation().getAxis().getZ();
  tmp_pose.pose.orientation.w=c.getRotation().getW();

  pose_pub2.publish(tmp_pose);
*/
}

int main(int argc, char **argv){
  ros::init(argc, argv, "basic_timing_bridge");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  pose_pub1 = n.advertise<geometry_msgs::PoseStamped>("pose_out", 10);
  pose_pub2 = n.advertise<geometry_msgs::PoseStamped>("pose_out_conv", 10);
  ros::Subscriber chatter_sub = n.subscribe("pose_in", 10, poseCallback);
  
  ros::spin();
  return 0;
}

