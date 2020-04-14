#include <ros/ros.h>
#include <visualization_msgs/Marker.h> 
#include <cmath>
int main( int argc, char** argv )
{
    ros::init(argc, argv, "points_and_lines");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    ros::Rate r(30);
 
    float f = 0.0;
    while (ros::ok()) { 
	/*initialization*/        
	visualization_msgs::Marker line_strip;
	/*不知道要給哪個時間跟id*/
        line_strip.header.frame_id = "/my_frame";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "lines";
        line_strip.action = visualization_msgs::Marker::ADD;
	/*應該跟旋轉無關吧*/
        line_strip.pose.orientation.w= 1.0; 
        line_strip.id = 1;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP; 
        // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
        line_strip.scale.x = 0.1;
        // Line strip is blue
        line_strip.color.b = 1.0;
        line_strip.color.a = 1.0;
        // Create the vertices for the points and lines
        for (uint32_t i = 0; i < 100; ++i) {
            float y = 5 * sin(f + i / 100.0f * 2 * M_PI);
            float z = 5 * cos(f + i / 100.0f * 2 * M_PI);
 	    //float type
        
            //line_strip.points.push_back(??);
            geometry_msgs::Point p;
            p.x = (int32_t)i - 50;
            p.y = y;
            p.z = z;
            line_strip.points.push_back(p);
            // The line list needs two points for each line
            p.z += 1.0;
        }
        marker_pub.publish(line_strip);
        r.sleep(); 
        f += 0.04;
    }
}
