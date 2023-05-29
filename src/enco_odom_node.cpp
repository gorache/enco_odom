#include <std_msgs/Float64.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

double prev_left_data;
double prev_right_data;

double now_left_data;
double now_right_data;

double del_left = 0;
double del_right = 0;


ros::Publisher odom_pub;
double x = 0.0;
double y = 0.0;
double th = 0.0;
double dt = 0.01; // time delta
double delta_x = 0;
double delta_y = 0;
double degree = 0;

double left_encoder_diff = 0.0;
double right_encoder_diff = 0.0;

// Kalman Filter variables
double filtered_left_encoder_diff = 0.0;
double filtered_right_encoder_diff = 0.0;
double prev_left_encoder_diff = 0.0;
double prev_right_encoder_diff = 0.0;
double Q = 0.01; // process noise covariance
double R = 0.1; // measurement noise covariance

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // Assuming the IMU is mounted in such a way that the Z axis aligns with yaw
    th = msg->angular_velocity.y;
    degree += msg->angular_velocity.y * dt * 180.0 / M_PI / 2.0;

}

void leftEncoderDiffCallback(const std_msgs::Float64::ConstPtr& msg4)
{
    // Apply Kalman Filter
    double current_left_encoder_diff = msg4->data;
    // double predicted_left_encoder_diff = filtered_left_encoder_diff;
    // double error_left_encoder_diff = prev_left_encoder_diff - predicted_left_encoder_diff;
    // double kalman_gain_left_encoder_diff = predicted_left_encoder_diff / (predicted_left_encoder_diff + R);
    // filtered_left_encoder_diff = predicted_left_encoder_diff + kalman_gain_left_encoder_diff * error_left_encoder_diff;
    // prev_left_encoder_diff = filtered_left_encoder_diff;

    // left_encoder_diff = filtered_left_encoder_diff;
    left_encoder_diff = current_left_encoder_diff;
    now_left_data = left_encoder_diff;
    // ROS_INFO("left_diff: %f", left_encoder_diff);
}

void rightEncoderDiffCallback(const std_msgs::Float64::ConstPtr& msg5)
{
    // Apply Kalman Filter
    double current_right_encoder_diff = msg5->data;
    // ROS_INFO("curr_right_diff: %f", current_right_encoder_diff);

    // double predicted_right_encoder_diff = filtered_right_encoder_diff;
    // double error_right_encoder_diff = prev_right_encoder_diff - predicted_right_encoder_diff;
    // double kalman_gain_right_encoder_diff = predicted_right_encoder_diff / (predicted_right_encoder_diff + R);
    // filtered_right_encoder_diff = predicted_right_encoder_diff + kalman_gain_right_encoder_diff * error_right_encoder_diff;
    // prev_right_encoder_diff = filtered_right_encoder_diff;

    // right_encoder_diff = filtered_right_encoder_diff;
    right_encoder_diff = current_right_encoder_diff;
    now_right_data = right_encoder_diff;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "odometry_publisher");

    ros::NodeHandle n;

    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

    tf::TransformBroadcaster odom_broadcaster;
    

    // Subscribing to the gyroscope topic of the D435i
    ros::Subscriber imu_sub = n.subscribe("/camera/imu", 50, imuCallback);
    ros::Subscriber left_encoder_sub = n.subscribe("left_encoder_diff_data", 50, leftEncoderDiffCallback);
    ros::Subscriber right_encoder_sub = n.subscribe("right_encoder_diff_data", 50, rightEncoderDiffCallback);

    ros::Rate r(100.0);
    
    
    while (n.ok()) {
        prev_left_data = left_encoder_diff;
        prev_right_data = right_encoder_diff;
        
        ros::spinOnce();

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

        nav_msgs::Odometry odom;
        del_left = now_left_data - prev_left_data;
        del_right = now_right_data - prev_right_data;
    
    
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";

        // Set the position
        delta_x = (del_left + del_right) / 2.0 * cos(th) * dt;
        delta_y = (del_left + del_right) / 2.0 * sin(th) * dt;
        x += delta_x;
        y += delta_y;


        ROS_INFO("x_position: %f", x);
        ROS_INFO("y_position: %f", y);
        ROS_INFO("degree: %f", degree);

        // Set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        // Set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = (del_left + del_right) / 2.0 / dt;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = th;
        

        // Publish the message
        odom_pub.publish(odom);

        r.sleep();
    }

    return 0;
}
