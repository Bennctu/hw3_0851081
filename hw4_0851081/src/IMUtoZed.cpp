#include <Eigen/Dense>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
using namespace Eigen;
ros::Publisher imu_pub;
void tfImuToZed(const sensor_msgs::Imu::ConstPtr &msgs) {
    sensor_msgs::Imu imu_data = *msgs;
    Matrix3d imuToCam(3,3);
    Matrix3d camToZed(3,3);
    imuToCam <<0.0225226,0.999745,0.0017194,
               0.0648765,-0.00317777,0.997888,
               0.997639 ,-0.0263635 ,-0.0649315;
    camToZed << 0,0,1,
               -1,0,0,
               0,-1,0;
    Quaterniond Q_imuToCam(imuToCam);
    Quaterniond Q_camToZed(camToZed);
    Quaterniond imuOri(imu_data.orientation.w,imu_data.orientation.x,imu_data.orientation.y,imu_data.orientation.z);
    Vector3d acc(imu_data.linear_acceleration.x,imu_data.linear_acceleration.y,imu_data.linear_acceleration.z);
    Vector3d angular_vel(imu_data.angular_velocity.x,imu_data.angular_velocity.y,imu_data.angular_velocity.z);
    acc = camToZed * imuToCam * acc;
    angular_vel = camToZed * imuToCam * angular_vel;
    imuOri = Q_camToZed * Q_imuToCam * imuOri;
    imu_data.linear_acceleration.x = acc.x();
    imu_data.linear_acceleration.y = acc.y();
    imu_data.linear_acceleration.z = acc.z();
    imu_data.angular_velocity.x = angular_vel.x();
    imu_data.angular_velocity.y = angular_vel.y();
    imu_data.angular_velocity.z = angular_vel.z();
    imu_data.orientation.w = imuOri.w();
    imu_data.orientation.x = imuOri.x();
    imu_data.orientation.y = imuOri.y();
    imu_data.orientation.z = imuOri.z();
    imu_pub.publish(imu_data);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"IMUtoZed");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>("/imu/data",20,tfImuToZed);
    imu_pub = nh.advertise<sensor_msgs::Imu>("/imu_data",20);
    ros::spin();
    return 0;
}
