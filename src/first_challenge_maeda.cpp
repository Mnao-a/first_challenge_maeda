#include "first_challenge_maeda/first_challenge_maeda.h"

FirstChallenge::FirstChallenge():private_nh_("~")
{
    private_nh_.param("hz", hz_, {10});
    odom_sub_ = nh_.subscribe("/roomba/odometry", 1, &FirstChallenge::odometry_callback, this);
    laser_sub_ = nh_.subscribe("/scan", 1, &FirstChallenge::laser_callback, this);
    cmd_vel_pub_ = nh_.advertise<roomba_500driver_meiji::RoombaCtrl>("/roomba/control", 1);
}

void FirstChallenge::odometry_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    odometry_ = *msg;
}

void FirstChallenge::laser_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_ = *msg;
}

float FirstChallenge::getyaw()
{
        double roll, pitch, yaw;
        tf::Quaternion quat(odometry_.pose.pose.orientation.x, odometry_.pose.pose.orientation.y, odometry_.pose.pose.orientation.z, odometry_.pose.pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        return float(yaw);
}

float FirstChallenge::Getrange_min()
{
        float range_min = 1e6;
        int size_medium = laser_.ranges.size()/2;
        int size_min = size_medium - 20;
        int size_max = size_medium + 20;
        for(int i=size_min; i<size_max; i++) {
            if(laser_.ranges[i] < range_min) {
                range_min = laser_.ranges[i];
            }
        }
        return float(range_min);
}

void FirstChallenge::run()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.2;
    cmd_vel_.cntl.angular.z = 0.0;

    cmd_vel_pub_.publish(cmd_vel_);
}

void FirstChallenge::rotate()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = M_PI/6;


    cmd_vel_pub_.publish(cmd_vel_);
}

void FirstChallenge::stop()
{
    cmd_vel_.mode = 11;
    cmd_vel_.cntl.linear.x = 0.0;
    cmd_vel_.cntl.angular.z = 0.0;

    cmd_vel_pub_.publish(cmd_vel_);
}

void FirstChallenge::show_odom()
{
    printf("odom x:%lf y:%lf z:%lf\n",odometry_.pose.pose.position.x,odometry_.pose.pose.position.y,odometry_.pose.pose.position.z);
}

void FirstChallenge::show_yaw()
{
    printf("yaw: %f\n",getyaw());
}

void FirstChallenge::show_scan()
{
        std::cout << "scan: min:" << Getrange_min() << std::endl;
}

void FirstChallenge::process()
{
    int count = 0;
    int phase = 1;
    ros::Rate loop_rate(hz_);
    while(ros::ok())
    {
        if(phase == 1){
            run();
            show_odom();
            ros::spinOnce();
            loop_rate.sleep();

            if(odometry_.pose.pose.position.x >= 1.0) {
                phase = 2;
            }
        }

        else if(phase == 2){
            rotate();
            show_yaw();
            ros::spinOnce();
            loop_rate.sleep();
            count += 1;

            if((getyaw() < 0.03) && (getyaw() > -0.03) && (count > 50)) {
                phase = 3;
            }
        }

        else if(phase == 3){
            run();
            show_scan();
            ros::spinOnce();
            loop_rate.sleep();

            if(Getrange_min() <= 0.55) {
                phase = 4;
            }
        }

        else if(phase == 4){

            stop();

            show_scan();

            ros::spinOnce();

            loop_rate.sleep();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "first_cihallenge_maeda");
    FirstChallenge first_challenge;
    first_challenge.process();
    return 0;
}
