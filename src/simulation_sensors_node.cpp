#include <ros/ros.h>
#include <agribot_msgs/AgribotHeading.h>
#include <agribot_msgs/AgribotInclinometer.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/Vector3Stamped.h>

ros::Publisher heading_pub;
ros::Publisher inclinometer_pub;
ros::Publisher GPS_pub;
ros::Publisher IMU_pub;
ros::Publisher magentometer_pub;

void HeadingCallback(const agribot_msgs::AgribotHeading simulation_heading_msg)
{
    heading_pub.publish(simulation_heading_msg);
    return;
}

void InclinometerCallback(const agribot_msgs::AgribotInclinometer simulation_inclinometer_msg)
{
    inclinometer_pub.publish(simulation_inclinometer_msg);
    return;
}

void GPSCallback(const sensor_msgs::NavSatFix simulation_GPS_msg)
{
    GPS_pub.publish(simulation_GPS_msg);
    return;
}

void IMUCallback(const sensor_msgs::Imu simulation_IMU_msg)
{
    IMU_pub.publish(simulation_IMU_msg);
    return;
}

void MagnetometerCallback(const geometry_msgs::Vector3Stamped simulation_magentometer_msg)
{
    sensor_msgs::MagneticField magnetometer_msg;
    magnetometer_msg.header = simulation_magentometer_msg.header;
    magnetometer_msg.magnetic_field = simulation_magentometer_msg.vector;
    magentometer_pub.publish(magnetometer_msg);
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "simulation_sensors");
    ros::NodeHandle n;
	ros::Rate loop_rate(100);

    ros::Subscriber simulation_heading_sub = n.subscribe("/simulation/heading", 1000, HeadingCallback);
    ros::Subscriber simulation_inclinometer_sub = n.subscribe("/simulation/inclinometer", 1000, InclinometerCallback);
    ros::Subscriber simulation_GPS_sub = n.subscribe("/simulation/fix", 1000, GPSCallback);
    ros::Subscriber simulation_IMU_sub = n.subscribe("/simulation/imu", 1000, IMUCallback);
    ros::Subscriber simulation_magentometer_sub = n.subscribe("/simulation/magnetic_field", 1000, MagnetometerCallback);

    heading_pub = n.advertise<agribot_msgs::AgribotHeading>("agribot/heading",1);
    inclinometer_pub = n.advertise<agribot_msgs::AgribotInclinometer>("agribot/inclinometer",1);
    GPS_pub = n.advertise<sensor_msgs::NavSatFix>("agribot/gnss",1);
    IMU_pub = n.advertise<sensor_msgs::Imu>("agribot/imu",1);
    magentometer_pub = n.advertise<sensor_msgs::MagneticField>("agribot/magnetic_field",1);

    while(ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}