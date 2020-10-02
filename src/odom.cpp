#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "std_msgs/Float32.h"
#include <cmath>

double dist_left;
double dist_right;

double encLeft = 0;
double encRight = 0 ;
double encLeft_old = 0;
double encRight_old = 0;
double DistancePerCount = (2.00 * 3.14159265 * 0.0762) / 1600.0;
double lengthBetweenTwoWheels;

//..................................................................................
void Enc_L_Callback(const std_msgs::Float32::ConstPtr& msg)
{
    encLeft = -1.0 * (double)msg->data;
}
//..................................................................................
void Enc_R_Callback(const std_msgs::Float32::ConstPtr& msg)
{
    encRight = -1.0 * (double)msg->data;
}
//..................................................................................
int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");
    ROS_INFO("HELLO WORLD");
    ros::NodeHandle n;
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
    ros::Subscriber subL = n.subscribe("Enc_L", 10, Enc_L_Callback);
    ros::Subscriber subR = n.subscribe("Enc_R", 10, Enc_R_Callback);
    ros::param::param<double>("~wheel_separation",lengthBetweenTwoWheels, 0.5);
    double wheel_radius = 0.0;
    ros::param::param<double>("~wheel_radius",wheel_radius, 0.0762);
    DistancePerCount = (2.00 * 3.14159265 * wheel_radius) / 1600.0;

    tf::TransformBroadcaster odom_broadcaster;
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    double x = 0;
    double y = 0;
    double theta = 0.0;
    ros::Rate r(10.0);
    while (n.ok())
    {
        ros::spinOnce(); // check for incoming messages
        current_time = ros::Time::now();

        double dt = (current_time - last_time).toSec();
        double delta_left = (encLeft - encLeft_old) * DistancePerCount;
        double delta_right = (encRight - encRight_old) * DistancePerCount;
        // Memory After use in upper two lines
        encLeft_old = encLeft;
        encRight_old = encRight;

        // Assume the robot is a point
        double robot_dist = (delta_left + delta_right) / 2.0;
        double robot_rotate = (delta_right - delta_left) / lengthBetweenTwoWheels;

        // Integrate small robot_dist and robot_rotate
        if(robot_dist != 0){
            /// [TF+ODOM] Calculate Distance Traveled in (x,y) Format
            double small_x =  cos(robot_rotate) * robot_dist;
            double small_y = -sin(robot_rotate) * robot_dist;

            /// Calculate Final position
            x += ( cos(theta) * small_x - sin(theta) * small_y );
            y += ( sin(theta) * small_x + cos(theta) * small_y );
        }
        if(robot_rotate !=0){
            theta += robot_rotate;
        }

        //since all odometry is 6DOF we'll need a quaternion created from yaw
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

        //first, we'll publish the transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = ros::Time::now();
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //next, we'll publish the odometry message over ROS
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = robot_dist/dt;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = robot_rotate/dt;

        //publish the message
        odom_pub.publish(odom);

        // Memory Last Variable
        last_time = current_time;
        r.sleep();
    }
}



/*

 /
*/