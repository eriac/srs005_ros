#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

ros::Publisher scan_pub;
void scan_callback(const sensor_msgs::LaserScan& scan_msg){
    sensor_msgs::LaserScan out;
    out=scan_msg;
    out.header.stamp=ros::Time::now();
    scan_pub.publish(out);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "s5_scan_restamp");
	ros::NodeHandle nh;

    //publisher
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan_restamp", 10);
	//Subscribe
	ros::Subscriber scan_sub = nh.subscribe("scan", 10, scan_callback); 
	
	ros::spin();
 	return 0;
}
