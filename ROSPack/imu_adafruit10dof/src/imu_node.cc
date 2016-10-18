#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"

#include "imu_adafruit10dof/IMU.h"

#include <sstream>
#include <boost/function.hpp>

#include "IMUDriver.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
ros::Publisher *pubPtr = NULL;
int imucount = 0;
double ScaleX, OffsetX, ScaleY, OffsetY, ScaleZ, OffsetZ;

void publishIMUData(float ax, float ay, float az, float gx, float gy, float gz, long long timeStamp)
{

    sensor_msgs::Imu imu_data;

    imu_data.linear_acceleration.x = -(ax - OffsetX)/ScaleX;
    imu_data.linear_acceleration.y = -(ay - OffsetY)/ScaleY;
    imu_data.linear_acceleration.z = -(az - OffsetZ)/ScaleZ;

    imu_data.linear_acceleration_covariance[0] = 0.0069;
    imu_data.linear_acceleration_covariance[1] = 0.00;
    imu_data.linear_acceleration_covariance[2] = 0.00;
    imu_data.linear_acceleration_covariance[3] = 0.00;
    imu_data.linear_acceleration_covariance[4] = 0.0069;
    imu_data.linear_acceleration_covariance[5] = 0.00;
    imu_data.linear_acceleration_covariance[6] = 0.00;
    imu_data.linear_acceleration_covariance[7] = 0.00;
    imu_data.linear_acceleration_covariance[8] = 0.0069;

    imu_data.angular_velocity.x = gx/180.*3.1415926;
    imu_data.angular_velocity.y = gy/180.*3.1415926;
    imu_data.angular_velocity.z = gz/180.*3.1415926;

    imu_data.angular_velocity_covariance[0] = 0.00017;
    imu_data.angular_velocity_covariance[1] = 0.000;
    imu_data.angular_velocity_covariance[2] = 0.000;
    imu_data.angular_velocity_covariance[3] = 0.000;
    imu_data.angular_velocity_covariance[4] = 0.00017;
    imu_data.angular_velocity_covariance[5] = 0.000;
    imu_data.angular_velocity_covariance[6] = 0.000;
    imu_data.angular_velocity_covariance[7] = 0.000;
    imu_data.angular_velocity_covariance[8] = 0.00017;

    imu_data.header.frame_id = "imu";
    imu_data.header.stamp = ros::Time::now();

    if (pubPtr == NULL)
        ROS_ERROR("NULL Ptr of Publisher\n");
    pubPtr->publish(imu_data);

    if (imucount == 200) {
        ROS_INFO("publish imu data: %f %f %f %f %f %f\n", ax, ay, az, gx, gy, gz);
        imucount = 0; 
    }

    ros::spinOnce();
}

int main(int argc, char **argv)
{

    // Init IMU Driver
    IMUDriver IMUDevice;
    char port[50] = "/dev/ttyACM0";
    IMUDevice.initIMU(port, 230400);
    IMUDevice.begin();

    // Init ros
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle n;
    n.getParam("ScaleX", ScaleX);
    n.getParam("ScaleY", ScaleY);
    n.getParam("ScaleZ", ScaleZ);
    n.getParam("OffsetX", OffsetX);
    n.getParam("OffsetY", OffsetY);
    n.getParam("OffsetZ", OffsetZ);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/adafruit_imu/imu_data", 1000);
    pubPtr = &imu_pub;

    // Set Callback
    ROS_INFO("Set IMU Callback...\n");
    boost::function<void (float, float, float, float, float, float, long long)> 
            callback(&publishIMUData);
    IMUDevice.setIMUCallback(callback);
    ROS_INFO("Set IMU Callback done\n");

    ros::spin();

    return 0;
}
