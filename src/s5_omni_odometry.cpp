#include "ros/ros.h"
  
#include "std_msgs/Int32.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"

#include "math.h"
#include <sstream>
#include <string>

std::string joint0_name="joint0";
std::string joint1_name="joint1";
std::string joint2_name="joint2";

int wheel_position[3]={0};
void position0_callback(const std_msgs::Int32& int_msg){
	wheel_position[0]=int_msg.data;
}
void position1_callback(const std_msgs::Int32& int_msg){
	wheel_position[1]=int_msg.data;
}
void position2_callback(const std_msgs::Int32& int_msg){
	wheel_position[2]=int_msg.data;
}

float wheel_base=0.100;
float wheel_radius=0.20;
float wheel_ratio=800.0/(2*M_PI); //pulse/radian

void wheel_invert(float *out, float *in){
	float wheel_base=0.0972;
	float wheel_radius=0.019;
	float lv=1.0/wheel_radius;
	float av=wheel_base/wheel_radius;
	float r3=sqrt(3);
	out[0]=(+(1.0/r3/lv)*in[0] +0.0       *in[1] -(1.0/r3/lv)*in[2])/wheel_ratio;
	out[1]=(-(1.0/3/lv) *in[0] +(2.0/3/lv)*in[1] -(1.0/3/lv) *in[2])/wheel_ratio;
	out[2]=(-(1.0/3/av) *in[0] -(1.0/3/av)*in[1] -(1.0/3/av) *in[2])/wheel_ratio;
}

float pos_x=0.0;
float pos_y=0.0;
float pos_theta=0.0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s5_omni_odometry");
	ros::NodeHandle n;
	ros::NodeHandle pn("~");
	//publish
	ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
	ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 1000);

	//Subscribe
	ros::Subscriber odometry0 = n.subscribe("wheel0", 10, position0_callback); 
	ros::Subscriber odometry1 = n.subscribe("wheel1", 10, position1_callback); 
	ros::Subscriber odometry2 = n.subscribe("wheel2", 10, position2_callback); 

	pn.getParam("wheel_base",    wheel_base);
	pn.getParam("wheel_radius",  wheel_radius);
	pn.getParam("joint0_name",  joint0_name);
	pn.getParam("joint1_name",  joint1_name);
	pn.getParam("joint2_name",  joint2_name);
        pn.getParam("wheel_ratio", wheel_ratio);

	float dt=1.0/10;
	ros::Rate loop_rate(10); 
	while (ros::ok()){
		//publish joint states
		sensor_msgs::JointState js0;
		js0.header.stamp = ros::Time::now();
		js0.name.resize(3);
		js0.name[0]=joint0_name;
		js0.name[1]=joint1_name;
		js0.name[2]=joint2_name;
		js0.position.resize(3);
		js0.position[0]=wheel_position[0];
		js0.position[1]=wheel_position[1];
		js0.position[2]=wheel_position[2];
		joint_pub.publish(js0);

		//publish odm_vel
		float roll[3]={0};
		static float wheel_position_last[3]={0};
		for(int i=0;i<3;i++){
			roll[i]=wheel_position[i]-wheel_position_last[i];
			wheel_position_last[i]=wheel_position[i];
		}
		float move[3]={0};
		wheel_invert(move,roll);

        pos_x += (move[0]*cos(pos_theta)-move[1]*sin(pos_theta));
        pos_y += (move[0]*sin(pos_theta)-move[1]*cos(pos_theta));
        pos_theta += move[2];

		nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id="odom";
        odom_msg.header.stamp=ros::Time::now();

		odom_msg.twist.twist.linear.x=move[0]/dt;
		odom_msg.twist.twist.linear.y=move[1]/dt;
		odom_msg.twist.twist.angular.z=move[2]/dt;
		
		odom_msg.pose.pose.position.x=pos_x;
		odom_msg.pose.pose.position.y=pos_y;
		odom_msg.pose.pose.position.z=0;
		odom_msg.pose.pose.orientation.y=0;		
		odom_msg.pose.pose.orientation.z=0;		
		odom_msg.pose.pose.orientation.z=sin(pos_theta/2);		
		odom_msg.pose.pose.orientation.w=cos(pos_theta/2);		
		
		odom_pub.publish(odom_msg);
		
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
