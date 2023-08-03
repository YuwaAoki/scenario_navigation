#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/String.h"
#include "scenario_navigation/PassageType.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cstdlib>
#include <cmath>
#include <bits/stdc++.h>
#include <vector>

class passageRecognition {
    public:
        passageRecognition();
        int SCAN_HZ = 10;
        float ROBOT_RADIUS = 0.5;
        float MIN_WALL_DISTANCE = 1.0;
        float distance_thresh = 2.0;
        float aisle_width_thresh_min = 0.6;
        float aisle_width_thresh_max = 5.4;
        float laser_choice_thresh = 9.0;
        std::string robot_frame_ = "base_link";
        scenario_navigation::PassageType generate_publish_variable(bool center_flg, bool back_flg, bool left_flg, bool right_flg,
                                                                        int center_angle, int back_angle, int left_angle, int right_angle);
        void checkRobotCollision(std::vector<double> *x, std::vector<double> *y, std::vector<int> scan_index, std::vector<float*> distance_list);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

    private:
        ros::NodeHandle node_;
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        ros::Publisher passage_type_pub_;
};

passageRecognition::passageRecognition(){
    node_.getParam("passageRecognition/SCAN_HZ", SCAN_HZ);
    node_.getParam("passageRecognition/robot_frame", robot_frame_);
    node_.getParam("passageRecognition/ROBOT_RADIUS", ROBOT_RADIUS);
    node_.getParam("passageRecognition/MIN_WALL_DISTANCE", MIN_WALL_DISTANCE);

    scan_sub_ = node_.subscribe<sensor_msgs::LaserScan> ("/scan", 1, &passageRecognition::scanCallback, this);
    marker_pub_ = node_.advertise<visualization_msgs::MarkerArray>("corridor_visualization", 1);
    passage_type_pub_ = node_.advertise<scenario_navigation::PassageType>("passage_type", 1);
}

// assume there is no passage if a collision is likely to occur
void passageRecognition::checkRobotCollision(std::vector<double> *x, std::vector<double> *y, std::vector<int> scan_index, std::vector<float*> distance_list){
    int cnt;
    int index_low, index_high;
    int scan_num = x->size();
    int skip = 10;
    for(int i = 0; i < scan_index.size(); i++){
        cnt = 1;
        do{
            index_low  = (scan_num + scan_index[i] - skip * cnt) % scan_num;
            index_high = (scan_num + scan_index[i] + skip * cnt) % scan_num;
            if(MIN_WALL_DISTANCE > std::hypot(x->at(index_low), y->at(index_low)) ||
                    MIN_WALL_DISTANCE > std::hypot(x->at(index_high), y->at(index_high))){
                *distance_list[i] = 0.0;
                break;
            }
            cnt++;
        } while(skip * cnt >= scan_num / 4 || 2 * ROBOT_RADIUS > std::hypot(x->at(index_low) - x->at(index_high), y->at(index_low) - y->at(index_high)));
    }
}

void passageRecognition::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    int num_scan = scan->ranges.size();
    std::vector<double> x(num_scan), y(num_scan);
    std::vector<int> angle_bin(90);
    std::vector<int> toe_index_list;
    std::vector<float> scan_cp(num_scan);
    std::copy(scan->ranges.begin(), scan->ranges.end(), scan_cp.begin());

    // convert to cartesian coordinate system corrdinate and skip inf
    int index_prev = 0;
    double last_scan = 0;
    while((std::isnan(scan_cp[index_prev]) || std::isinf(scan_cp[index_prev]) || scan_cp[index_prev] > scan->range_max) && index_prev < num_scan) index_prev++;
    last_scan = scan_cp[index_prev];

    double scan_range;
    for(int i = 0; i < num_scan; i ++) {
        double angle = scan->angle_min + scan->angle_increment * i;
        if(std::isnan(scan_cp[i]) || std::isinf(scan_cp[i]) || scan_cp[i] > scan->range_max){
            scan_range = last_scan;
            scan_cp[i] = last_scan;
        }
        else{
            scan_range = scan_cp[i];
            last_scan = scan_cp[i];
        }
        x[i] = scan_range * cos(angle);
        y[i] = scan_range * sin(angle);
    }

    // detect wall angle
    for(int i = 1; i + 10 < num_scan - 1; i ++) {
        double angle = atan2(y[i] - y[i+10-1], x[i] - x[i+10-1]);
        while(angle <     0.0) angle += M_PI_2;
        while(angle >= M_PI_2) angle -= M_PI_2;
        angle_bin[static_cast<int>(angle/M_PI*180)] += 1;
    }

    int max_angle = 0, max_bin = 0;
    max_angle = std::max_element(angle_bin.begin(), angle_bin.end()) - angle_bin.begin();
    max_bin = angle_bin[max_angle];

    double scan_angle = static_cast<double>((max_angle <= 45) ? max_angle : max_angle - 90)/180.0*M_PI;

    // measure distance
    // kanari kitanai kakugo ha iika ? ore ha dekiteru
    int scan_left   = static_cast<int>((scan_angle + M_PI_2 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_left_front = static_cast<int>((scan_angle + M_PI / 36 * 17 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_center_left = static_cast<int>((scan_angle + M_PI / 36 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_center = static_cast<int>((scan_angle          - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_center_right  = static_cast<int>((scan_angle - M_PI / 36 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_right_front = static_cast<int>((scan_angle - M_PI / 36 * 17 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_right   = static_cast<int>((scan_angle - M_PI_2 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_right_back = static_cast<int>((scan_angle - M_PI / 36 * 19 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_back_right = static_cast<int>((scan_angle - M_PI / 36 * 35 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_back = static_cast<int>((scan_angle - M_PI - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_back_left = static_cast<int>((scan_angle + M_PI / 36 * 35 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_left_back = static_cast<int>((scan_angle + M_PI / 36 * 19 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;

    float distance_left   = std::sqrt(x[scan_left]   * x[scan_left]   + y[scan_left]   * y[scan_left]  );
    float distance_left_front = std::sqrt(x[scan_left_front]   * x[scan_left_front]   + y[scan_left_front]   * y[scan_left_front]  );
    float distance_center_left = std::sqrt(x[scan_center_left]   * x[scan_center_left]   + y[scan_center_left]   * y[scan_center_left]  );
    float distance_center = std::sqrt(x[scan_center] * x[scan_center] + y[scan_center] * y[scan_center]);
    float distance_center_right = std::sqrt(x[scan_center_right]   * x[scan_center_right]   + y[scan_center_right]   * y[scan_center_right]  );
    float distance_right_front = std::sqrt(x[scan_right_front]   * x[scan_right_front]   + y[scan_right_front]   * y[scan_right_front]  );
    float distance_right  = std::sqrt(x[scan_right]  * x[scan_right]  + y[scan_right]  * y[scan_right] );
    float distance_right_back = std::sqrt(x[scan_right_back]   * x[scan_right_back]   + y[scan_right_back]   * y[scan_right_back]  );
    float distance_back_right = std::sqrt(x[scan_back_right]   * x[scan_back_right]   + y[scan_back_right]   * y[scan_back_right]  );
    float distance_back   = std::sqrt(x[scan_back]   * x[scan_back]   + y[scan_back]   * y[scan_back]  );
    float distance_back_left = std::sqrt(x[scan_back_left]   * x[scan_back_left]   + y[scan_back_left]   * y[scan_back_left]  );
    float distance_left_back = std::sqrt(x[scan_left_back]   * x[scan_left_back]   + y[scan_left_back]   * y[scan_left_back]  );

    std::vector<int> scan_index = {scan_left, scan_center, scan_right, scan_back};
    std::vector<float*> distance_list(11);
    distance_list[0] = &distance_left;
    distance_list[1] = &distance_left_front;
    distance_list[2] = &distance_center_left;
    distance_list[3] = &distance_center;
    distance_list[4] = &distance_center_right;
    distance_list[5] = &distance_right_front;
    distance_list[6] = &distance_right;
    distance_list[7] = &distance_right_back;
    distance_list[8] = &distance_back_right;
    distance_list[9] = &distance_back;
    distance_list[10] = &distance_back_left;
    distance_list[11] = &distance_left_back;

    float distance_left_front_to_left_back = std::abs(x[scan_left_front] - x[scan_left_back]);
    float distance_center_left_to_back_left = std::abs(x[scan_center_left] - x[scan_back_left]);
    float distance_center_left_to_center_right = std::abs(y[scan_center_left] - y[scan_center_right]);
    float distance_left_front_to_right_front = std::abs(y[scan_left_front] - y[scan_right_front]);
    float distance_right_front_right_back = std::abs(x[scan_right_front] - x[scan_right_back]);
    float distance_center_right_to_back_right = std::abs(x[scan_center_right] - x[scan_back_right]);
    float distance_back_left_to_back_right = std::abs(y[scan_back_left] - y[scan_back_right]);
    float distance_left_back_to_right_back = std::abs(y[scan_left_back] - y[scan_right_back]);

    ROS_INFO("distance_center1 is %f\n", distance_center);

    // assume there is no passage if a collision is likely to occur
    checkRobotCollision(&x, &y, scan_index, distance_list);
    //ROS_INFO("distance_center2 is %f\n", distance_center);

    // publish passage_type of passage recognition
    scenario_navigation::PassageType passage_type;
	    
    if(distance_left > laser_choice_thresh){
        passage_type.left_flg = distance_left > distance_thresh && distance_left_front_to_left_back < aisle_width_thresh_max && distance_left_front_to_left_back > aisle_width_thresh_min ? true : false;
        passage_type.left_angle = scan_angle + M_PI_2 - scan->angle_min;
    }else{
        passage_type.left_flg = distance_left > distance_thresh && distance_center_left_to_back_left < aisle_width_thresh_max && distance_center_left_to_back_left > aisle_width_thresh_min ? true : false;
        passage_type.left_angle = scan_angle + M_PI_2 - scan->angle_min;
    };

    if(distance_center > laser_choice_thresh){
        passage_type.center_flg = distance_center > distance_thresh && distance_center_left_to_center_right < aisle_width_thresh_max && distance_center_left_to_center_right > aisle_width_thresh_min ? true : false;
        passage_type.center_angle = scan_angle - scan->angle_min;
    }else{
        passage_type.center_flg = distance_center > distance_thresh && distance_left_front_to_right_front < aisle_width_thresh_max && distance_left_front_to_right_front > aisle_width_thresh_min ? true : false;
        passage_type.center_angle = scan_angle - scan->angle_min;
    };

    if(distance_right > laser_choice_thresh){
        passage_type.right_flg = distance_right > distance_thresh && distance_right_front_right_back < aisle_width_thresh_max && distance_right_front_right_back > aisle_width_thresh_min? true : false;
        passage_type.right_angle = scan_angle - M_PI_2 - scan->angle_min;
    }else{
        passage_type.right_flg = distance_right > distance_thresh && distance_center_right_to_back_right < aisle_width_thresh_max && distance_center_right_to_back_right > aisle_width_thresh_min? true : false;
        passage_type.right_angle = scan_angle - M_PI_2 - scan->angle_min;
    };

    if(distance_back > laser_choice_thresh){
        passage_type.back_flg = distance_back > distance_thresh && distance_back_left_to_back_right < aisle_width_thresh_max && distance_back_left_to_back_right > aisle_width_thresh_min ? true : false;
        passage_type.back_angle = scan_angle - M_PI - scan->angle_min;
    }else{
        passage_type.back_flg = distance_back > distance_thresh && distance_left_back_to_right_back < aisle_width_thresh_max && distance_left_back_to_right_back > aisle_width_thresh_min ? true : false;
        passage_type.back_angle = scan_angle - M_PI - scan->angle_min;
    };

    // publish passage_type of passage recognition
    passage_type_pub_.publish(passage_type);

    // publish line for rviz
    if (passage_type.left_flg  ) toe_index_list.push_back(scan_left  );
    if (passage_type.center_flg) toe_index_list.push_back(scan_center);
    if (passage_type.right_flg ) toe_index_list.push_back(scan_right );
    if (passage_type.back_flg  ) toe_index_list.push_back(scan_back  );
    toe_index_list.push_back(scan_left_front);
    toe_index_list.push_back(scan_center_left);
    toe_index_list.push_back(scan_center_right);
    toe_index_list.push_back(scan_right_front);
    toe_index_list.push_back(scan_right_back);
    toe_index_list.push_back(scan_back_right);
    toe_index_list.push_back(scan_back_left);
    toe_index_list.push_back(scan_left_back);
    visualization_msgs::MarkerArray marker_line;
    marker_line.markers.resize(toe_index_list.size());
    geometry_msgs::Point linear_start;
    geometry_msgs::Point linear_end;
    double line_lifetime = 1 / static_cast<double>(SCAN_HZ);

    for(int i = 0; i < toe_index_list.size(); i++){
        marker_line.markers[i].header.frame_id = robot_frame_;
        marker_line.markers[i].header.stamp = ros::Time::now();
        marker_line.markers[i].ns = "toe";
        marker_line.markers[i].id = i;
        marker_line.markers[i].lifetime = ros::Duration(line_lifetime);

        marker_line.markers[i].type = visualization_msgs::Marker::LINE_LIST;
        marker_line.markers[i].action = visualization_msgs::Marker::ADD;

        linear_start.x = 0;
        linear_start.y = 0;
        linear_start.z = 0;

        double range = scan_cp[toe_index_list[i]];
        linear_end.x = range;
        linear_end.y = 0;
        linear_end.z = 0;

        marker_line.markers[i].points.resize(2);
        marker_line.markers[i].points[0] = linear_start;
        marker_line.markers[i].points[1] = linear_end;

        marker_line.markers[i].scale.x = 0.1;

        float yaw_rad = scan->angle_min + toe_index_list[i] * scan->angle_increment;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, yaw_rad);
        geometry_msgs::Quaternion geometry_quat;
        geometry_quat = tf2::toMsg(quat);
        marker_line.markers[i].pose.orientation = geometry_quat;

        marker_line.markers[i].color.r = 0.0f;
        marker_line.markers[i].color.g = 1.0f;
        marker_line.markers[i].color.b = 0.0f;
        marker_line.markers[i].color.a = 1.0f;
    }
    marker_pub_.publish(marker_line);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "passageRecognition");
    passageRecognition recognition;
    ros::Rate loop_rate(recognition.SCAN_HZ);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
