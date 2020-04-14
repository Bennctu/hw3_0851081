#ifndef imuData_H
#define imuData_H
#include <Eigen/Dense>
#include "sensor_msgs/Imu.h"
using namespace Eigen;
class IMU_content
{
    public:
        IMU_content ()
        {
        }
        IMU_content (const sensor_msgs::Imu::ConstPtr &msg)
        {
            sensor_msgs::Imu imu_data = *msg;
            moment = imu_data.header.stamp.toSec();
            acc.x() = imu_data.linear_acceleration.x;
            acc.y() = imu_data.linear_acceleration.y;
            acc.z() = imu_data.linear_acceleration.z;
            angular_vel.x() = imu_data.angular_velocity.x;
            angular_vel.y() = imu_data.angular_velocity.y;
            angular_vel.z() = imu_data.angular_velocity.z;
        }
        Vector3d acc;
        Vector3d angular_vel;
        double moment;
    private:
        
};
#endif