#ifndef FIRST_CHALLENGE_MAEDA_H
#define FIRST_CHALLENGE_MAEDA_H

#include <ros/ros.h>
#include <roomba_500driver_meiji/RoombaCtrl.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/tf.h>

class FirstChallenge
{
    public:
        FirstChallenge();
        void process();

    private:
        void odometry_callback(const nav_msgs::Odometry::ConstPtr&);
        void laser_callback(const sensor_msgs::LaserScan::ConstPtr&);
        float getyaw();
        float Getrange_min();



        void run();
        void rotate();
        void stop();
        void show_odom();
        void show_yaw();
        void show_scan();
        int hz_;

        nav_msgs::Odometry odometry_;
        sensor_msgs::LaserScan laser_;
        roomba_500driver_meiji::RoombaCtrl cmd_vel_;

        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;
        ros::Subscriber odom_sub_;
        ros::Subscriber laser_sub_;
        ros::Publisher cmd_vel_pub_;

};

#endif
