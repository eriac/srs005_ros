#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

geometry_msgs::Twist cmd_vel;
float x_scale=1.0;
float y_scale=1.0;
float yaw_scale=1.0;
void joy_callback(const sensor_msgs::Joy& joy_msg){
    cmd_vel.linear.x =joy_msg.axes[1]*x_scale;
    cmd_vel.linear.y =joy_msg.axes[0]*y_scale;
    cmd_vel.angular.z=joy_msg.axes[2]*yaw_scale;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "basic_twist_publisher");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    pn.getParam("x_scale", x_scale);
    pn.getParam("y_scale", y_scale);
    pn.getParam("yaw_scale", yaw_scale);

    //publish
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

    //subscriibe
    ros::Subscriber joy_sub   = n.subscribe("joy", 10, joy_callback);

    ros::Rate loop_rate(10);
    while (ros::ok()){
        cmd_pub.publish(cmd_vel);
    	ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
