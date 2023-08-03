#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/LaserScan.h"
#include "scenario_navigation/Scenario.h"
#include <unistd.h>
#include <cmath>
#include <vector>
#include <iostream>

class localPlanner {
     public:
        localPlanner();
        geometry_msgs::Twist vel_;
        double IMU_HZ = 10.0;
        double OBSTACLE_AVOIDANCE_DISTANCE_THRE = 0.4;
        double OBSTACLE_AVOIDANCE_RAD = 0.3;
        float rotate_rad_ = 0.0;
        void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data);
        void turnRadCallback(const std_msgs::Float32::ConstPtr& turn_rad);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
        void stopCallback(const std_msgs::Bool::ConstPtr& stop);

     private:
        ros::NodeHandle node_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher turn_finish_flg_pub_;
        ros::Subscriber imu_sub_;
        ros::Subscriber rotate_rad_sub_;
        ros::Subscriber scan_sub_;
        ros::Subscriber stop_sub_;
        bool big_modified_flg_left_ = false;
        bool big_modified_flg_right_ = false;

        bool turn_flg_ = false;
        bool stop_flg_ = true;
        double target_yaw_rad_ = 0;
        double current_yaw_rad_ = 0;
};

localPlanner::localPlanner(){
    node_.getParam("localPlanner/IMU_HZ", IMU_HZ);
    node_.getParam("localPlanner/OBSTACLE_AVOIDANCE_DISTANCE_THRE", OBSTACLE_AVOIDANCE_DISTANCE_THRE);
    node_.getParam("localPlanner/OBSTACLE_AVOIDANCE_RAD", OBSTACLE_AVOIDANCE_RAD);

    cmd_vel_pub_ = node_.advertise<geometry_msgs::Twist>("cmd_vel", 1, false);
    turn_finish_flg_pub_ = node_.advertise<std_msgs::Bool>("turn_finish_flg", 1, false);
    imu_sub_ = node_.subscribe<sensor_msgs::Imu> ("imu_data", 1, &localPlanner::imuCallback, this);
    rotate_rad_sub_ = node_.subscribe<std_msgs::Float32> ("rotate_rad", 1, &localPlanner::turnRadCallback, this);
    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan>("scan", 1, &localPlanner::scanCallback, this);
    stop_sub_ = node_.subscribe<std_msgs::Bool> ("stop", 1, &localPlanner::stopCallback, this);
}

void localPlanner::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_data){
    current_yaw_rad_ -= imu_data->angular_velocity.z / IMU_HZ;
    
    if(stop_flg_){

        vel_.linear.x = 0.0;
        vel_.angular.z = 0.0;
        //cmd_vel_pub_.publish(vel_);
    } else if(turn_flg_){
        rotate_rad_ -= imu_data->angular_velocity.z / IMU_HZ;
        if(std::abs(rotate_rad_) < 3.14/180){
        // 1.57){
        // if(std::abs(rotate_rad_) < 1.57){
        // if(std::abs(rotate_rad_) < 1.57){
        
            turn_flg_ = false;
            std_msgs::Bool turn_finish;
            turn_finish.data = true;
            turn_finish_flg_pub_.publish(turn_finish);
            rotate_rad_ = 0;
        }
        vel_.linear.x = 0.0;
        //vel_.angular.z = rotate_rad_ > 0 ? 0.5 : -0.5;
        vel_.angular.z = 0;
        //cmd_vel_pub_.publish(vel_);
     } else {
        //vel_.linear.x = 0.55;
        vel_.linear.x = 0;
        //vel_.angular.z = -(target_yaw_rad_ - current_yaw_rad_);
        vel_.angular.z = 0;
        //cmd_vel_pub_.publish(vel_);
     }
}

void localPlanner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    // obstacle avoidance
    std::vector<float> scan_copy(scan->ranges.size());
    std::copy(scan->ranges.begin(), scan->ranges.end(), scan_copy.begin());
    for(int i=0; i<scan->ranges.size(); i++){
        if(scan_copy[i] <= scan->range_min){
            scan_copy[i] = 1000.0;
        }
    }
    int index_scan_min_right = std::min_element(std::next(scan_copy.begin(), (-3*M_PI_4 - scan->angle_min)/scan->angle_increment), std::next(scan_copy.begin(), (-M_PI/18 - scan->angle_min)/scan->angle_increment)) - scan_copy.begin();
    int index_scan_min_left = std::min_element(std::next(scan_copy.begin(), (M_PI/18 - scan->angle_min)/scan->angle_increment), std::next(scan_copy.begin(), (3*M_PI_4 - scan->angle_min)/scan->angle_increment)) - scan_copy.begin();
    double distance_scan_min_right = scan->ranges[index_scan_min_right];
    double distance_scan_min_left = scan->ranges[index_scan_min_left];
    if(distance_scan_min_right <= OBSTACLE_AVOIDANCE_DISTANCE_THRE && distance_scan_min_right < distance_scan_min_left){
        if(!big_modified_flg_right_){
          ROS_WARN("Modified robot-direction to left");
          big_modified_flg_right_ = true;
          target_yaw_rad_ -= OBSTACLE_AVOIDANCE_RAD;
        }
    }
    else if(distance_scan_min_left <= OBSTACLE_AVOIDANCE_DISTANCE_THRE && distance_scan_min_left < distance_scan_min_right){
        if(!big_modified_flg_left_){
          ROS_WARN("Modified robot-direction to right");
          big_modified_flg_left_ = true;
          target_yaw_rad_ += OBSTACLE_AVOIDANCE_RAD;
        }
    }
    else{
        if(big_modified_flg_right_){
            big_modified_flg_right_ = false;
            target_yaw_rad_ += OBSTACLE_AVOIDANCE_RAD * 9 / 10;
        }
        else if(big_modified_flg_left_){
            big_modified_flg_left_ = false;
            target_yaw_rad_ -= OBSTACLE_AVOIDANCE_RAD * 9 / 10;
        }
    }
}

void localPlanner::turnRadCallback(const std_msgs::Float32::ConstPtr& turn_rad){
    rotate_rad_ += turn_rad->data;
    target_yaw_rad_ -= turn_rad->data;
    turn_flg_ = rotate_rad_ != 0.0 ? true : false;
    printf("%d\n", turn_flg_);
    ROS_INFO("turnradcall");
}

void localPlanner::stopCallback(const std_msgs::Bool::ConstPtr& stop){
    stop_flg_ = stop->data;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "local_planner");
    localPlanner local_planner;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
