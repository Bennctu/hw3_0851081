#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <cmath>
#include "nav_msgs/Odometry.h"
//#include "nav_msgs/Path.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
using namespace std;
//using namespace nav_msgs;
ros::Publisher GT_pub;
ros::Publisher ekf_pub;
vector<nav_msgs::Odometry> odom_Buf;
vector<geometry_msgs::PoseWithCovarianceStamped> ekf_Buf;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msgs) {
    nav_msgs::Odometry zed_pose = *msgs;
    visualization_msgs::Marker line_strip_zed;
    line_strip_zed.header.frame_id = "/world";
    line_strip_zed.header.stamp = zed_pose.header.stamp;
    line_strip_zed.ns = "lines2";
    line_strip_zed.action = visualization_msgs::Marker::ADD;
    line_strip_zed.pose.orientation.w= 1.0;
    line_strip_zed.id = 2;
    line_strip_zed.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip_zed.scale.x = 0.1;
    // Line strip is red
    line_strip_zed.color.r = 1.0;
    line_strip_zed.color.a = 1.0;
    odom_Buf.push_back(zed_pose);
    vector<nav_msgs::Odometry>::iterator begin = odom_Buf.begin();
    vector<nav_msgs::Odometry>::iterator end = odom_Buf.end();
    vector<nav_msgs::Odometry>::iterator it;
    //Output total position data in odom_Buf
    for (it = begin ; it != end; it++) {
        geometry_msgs::Point p;
        nav_msgs::Odometry pose = *it;
        p.x = pose.pose.pose.position.x;
        p.y = pose.pose.pose.position.y;
        p.z = pose.pose.pose.position.z;
        line_strip_zed.points.push_back(p);
    }
    GT_pub.publish(line_strip_zed);

}
void ekf_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msgs) {
    geometry_msgs::PoseWithCovarianceStamped ekf_pose = *msgs;
    visualization_msgs::Marker line_strip_ekf;
    line_strip_ekf.header.frame_id = "/world";
    line_strip_ekf.header.stamp = ekf_pose.header.stamp;
    line_strip_ekf.ns = "lines3";
    line_strip_ekf.action = visualization_msgs::Marker::ADD;
    line_strip_ekf.pose.orientation.w= 1.0;
    line_strip_ekf.id = 3;
    line_strip_ekf.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip_ekf.scale.x = 0.1;
    // Line strip is green
    line_strip_ekf.color.g = 1.0;
    line_strip_ekf.color.a = 1.0;
    ekf_Buf.push_back(ekf_pose);
    vector<geometry_msgs::PoseWithCovarianceStamped>::iterator begin = ekf_Buf.begin();
    vector<geometry_msgs::PoseWithCovarianceStamped>::iterator end = ekf_Buf.end();
    vector<geometry_msgs::PoseWithCovarianceStamped>::iterator it;
    //Output total position data in odom_Buf
    for (it = begin ; it != end; it++) {
        geometry_msgs::Point p;
        geometry_msgs::PoseWithCovarianceStamped pose = *it;
        p.x = pose.pose.pose.position.x;
        p.y = pose.pose.pose.position.y;
        p.z = pose.pose.pose.position.z;
        line_strip_ekf.points.push_back(p);
    }
    ekf_pub.publish(line_strip_ekf);

}
int main( int argc, char** argv )
{
    ros::init(argc, argv, "zedGT");
    ros::NodeHandle n;
    ros::Subscriber visual_sub = n.subscribe<nav_msgs::Odometry>("/zed/odom",52,odom_callback);
    GT_pub = n.advertise<visualization_msgs::Marker>("/GT_path",52);
    ros::Subscriber ekf_sub = n.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/robot_pose_ekf/odom_combined",10,ekf_callback);
    ekf_pub = n.advertise<visualization_msgs::Marker>("/ekf_path",10);
    ros::spin();
    return 0;
}

