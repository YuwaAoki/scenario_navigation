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
        float distance_thresh = 3.0;
        float aisle_width_thresh_min = 0.6;
        float aisle_width_thresh_max = 5.4;
        float front_back_laser_choice_thresh = 3.0;
        float left_right_laser_choice_thresh = 13.0;
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
    int scan_plus_90   = static_cast<int>((scan_angle + M_PI_2 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_plus_85 = static_cast<int>((scan_angle + M_PI / 36 * 17 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_plus_45 = static_cast<int>((scan_angle + M_PI_4 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_plus_5 = static_cast<int>((scan_angle + M_PI / 36 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_0 = static_cast<int>((scan_angle          - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_minus_5  = static_cast<int>((scan_angle - M_PI / 36 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_minus_45 = static_cast<int>((scan_angle - M_PI_4 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_minus_85 = static_cast<int>((scan_angle - M_PI / 36 * 17 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_minus_90   = static_cast<int>((scan_angle - M_PI_2 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_minus_95 = static_cast<int>((scan_angle - M_PI / 36 * 19 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_minus_135 = static_cast<int>((scan_angle - M_PI_4 * 3 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_minus_175 = static_cast<int>((scan_angle - M_PI / 36 * 35 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_180 = static_cast<int>((scan_angle - M_PI - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_plus_175 = static_cast<int>((scan_angle + M_PI / 36 * 35 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_plus_135 = static_cast<int>((scan_angle + M_PI_4 *3 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_plus_95 = static_cast<int>((scan_angle + M_PI / 36 * 19 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;

    float distance_plus_90   = std::sqrt(x[scan_plus_90]   * x[scan_plus_90]   + y[scan_plus_90]   * y[scan_plus_90]  );
    float distance_plus_85 = std::sqrt(x[scan_plus_85]   * x[scan_plus_85]   + y[scan_plus_85]   * y[scan_plus_85]  );
    float distance_plus_45 = std::sqrt(x[scan_plus_45]   * x[scan_plus_45]   + y[scan_plus_45]   * y[scan_plus_45]  );
    float distance_plus_5 = std::sqrt(x[scan_plus_5]   * x[scan_plus_5]   + y[scan_plus_5]   * y[scan_plus_5]  );
    float distance_0 = std::sqrt(x[scan_0] * x[scan_0] + y[scan_0] * y[scan_0]);
    float distance_minus_5 = std::sqrt(x[scan_minus_5]   * x[scan_minus_5]   + y[scan_minus_5]   * y[scan_minus_5]  );
    float distance_minus_45 = std::sqrt(x[scan_minus_45]   * x[scan_minus_45]   + y[scan_minus_45]   * y[scan_minus_45]  );
    float distance_minus_85 = std::sqrt(x[scan_minus_85]   * x[scan_minus_85]   + y[scan_minus_85]   * y[scan_minus_85]  );
    float distance_minus_90  = std::sqrt(x[scan_minus_90]  * x[scan_minus_90]  + y[scan_minus_90]  * y[scan_minus_90] );
    float distance_minus_95 = std::sqrt(x[scan_minus_95]   * x[scan_minus_95]   + y[scan_minus_95]   * y[scan_minus_95]  );
    float distance_minus_135 = std::sqrt(x[scan_minus_135]   * x[scan_minus_135]   + y[scan_minus_135]   * y[scan_minus_135]  );
    float distance_minus_175 = std::sqrt(x[scan_minus_175]   * x[scan_minus_175]   + y[scan_minus_175]   * y[scan_minus_175]  );
    float distance_180   = std::sqrt(x[scan_180]   * x[scan_180]   + y[scan_180]   * y[scan_180]  );
    float distance_plus_175 = std::sqrt(x[scan_plus_175]   * x[scan_plus_175]   + y[scan_plus_175]   * y[scan_plus_175]  );
    float distance_plus_135 = std::sqrt(x[scan_plus_135]   * x[scan_plus_135]   + y[scan_plus_135]   * y[scan_plus_135]  );
    float distance_plus_95 = std::sqrt(x[scan_plus_95]   * x[scan_plus_95]   + y[scan_plus_95]   * y[scan_plus_95]  );

    std::vector<int> scan_index = {scan_plus_90, scan_0, scan_minus_90, scan_180};
    std::vector<float*> distance_list(16);
    distance_list[0] = &distance_plus_90;
    distance_list[1] = &distance_plus_85;
    distance_list[2] = &distance_plus_45;
    distance_list[3] = &distance_plus_5;
    distance_list[4] = &distance_0;
    distance_list[5] = &distance_minus_5;
    distance_list[6] = &distance_minus_45;
    distance_list[7] = &distance_minus_85;
    distance_list[8] = &distance_minus_90;
    distance_list[9] = &distance_minus_95;
    distance_list[10] = &distance_minus_135;
    distance_list[11] = &distance_minus_175;
    distance_list[12] = &distance_180;
    distance_list[13] = &distance_plus_175;
    distance_list[14] = &distance_plus_135;
    distance_list[15] = &distance_plus_95;
    

    float distance_plus_45_to_plus_135 = std::abs(x[scan_plus_45] - x[scan_plus_135]);
    float distance_plus_85_to_plus_95 = std::abs(x[scan_plus_85] - x[scan_plus_95]);
    float distance_plus_5_to_plus_175 = std::abs(x[scan_plus_5] - x[scan_plus_175]);
    float distance_plus_5_to_minus_5 = std::abs(y[scan_plus_5] - y[scan_minus_5]);
    float distance_plus_45_to_minus_45 = std::abs(y[scan_plus_45] - y[scan_minus_45]);
    float distance_plus_85_to_minus_85 = std::abs(y[scan_plus_85] - y[scan_minus_85]);
    float distance_minus_45_to_minus_135 = std::abs(x[scan_minus_45] - x[scan_minus_135]);
    float distance_minus_85_to_minus_95 = std::abs(x[scan_minus_85] - x[scan_minus_95]);
    float distance_minus_5_to_minus_175 = std::abs(x[scan_minus_5] - x[scan_minus_175]);
    float distance_plus_175_to_minus_175 = std::abs(y[scan_plus_175] - y[scan_minus_175]);
    float distance_plus_135_to_minus_135 = std::abs(y[scan_plus_135] - y[scan_minus_135]);
    float distance_plus_95_to_minus_95 = std::abs(y[distance_plus_95] - y[scan_minus_95]);

    float distance_plus_90_to_minus_90 = std::abs(y[scan_plus_90] - y[scan_minus_90]);


    // assume there is no passage if a collision is likely to occur
    checkRobotCollision(&x, &y, scan_index, distance_list);

    ROS_INFO("distance_0 is %f\n", distance_0);
    ROS_INFO("distance_plus_90 is %f\n", distance_plus_90);
    ROS_INFO("distance_minus_90 is %f\n", distance_minus_90);
    ROS_INFO("distance_180 is %f\n", distance_180);

    // publish passage_type of passage recognition
    scenario_navigation::PassageType passage_type;
	    
    if(distance_plus_90 > left_right_laser_choice_thresh || distance_plus_90_to_minus_90 > 30){
        passage_type.left_flg = distance_plus_90 > distance_thresh && distance_plus_85_to_plus_95 < aisle_width_thresh_max && distance_plus_85_to_plus_95 > aisle_width_thresh_min ? true : false;
        passage_type.left_angle = scan_angle + M_PI_2 - scan->angle_min;
    }else{
        passage_type.left_flg = distance_plus_90 > distance_thresh && distance_plus_45_to_plus_135 < aisle_width_thresh_max && distance_plus_45_to_plus_135 > aisle_width_thresh_min ? true : false;
        passage_type.left_angle = scan_angle + M_PI_2 - scan->angle_min;
    };

    if(distance_0 > 7.0 && distance_plus_90 > 10.0 && distance_minus_90 < 7.0 && distance_minus_90 > 3.0 && distance_180 > 9.0){
        passage_type.center_flg = distance_0 > distance_thresh && distance_plus_45_to_minus_45 < aisle_width_thresh_max && distance_plus_45_to_minus_45 > aisle_width_thresh_min ? true : false;
        passage_type.center_angle = scan_angle - scan->angle_min;
    }else if(distance_0 > front_back_laser_choice_thresh){
        passage_type.center_flg = distance_0 > distance_thresh && distance_plus_5_to_minus_5 < aisle_width_thresh_max && distance_plus_5_to_minus_5 > aisle_width_thresh_min ? true : false;
        passage_type.center_angle = scan_angle - scan->angle_min;
    }else{
        passage_type.center_flg = distance_0 > distance_thresh && distance_plus_45_to_minus_45 < aisle_width_thresh_max && distance_plus_45_to_minus_45 > aisle_width_thresh_min ? true : false;
        passage_type.center_angle = scan_angle - scan->angle_min;
    };

    

    if(distance_minus_90 > left_right_laser_choice_thresh || distance_plus_90_to_minus_90 > 30){
        passage_type.right_flg = distance_minus_90 > distance_thresh && distance_minus_85_to_minus_95 < aisle_width_thresh_max && distance_minus_85_to_minus_95 > aisle_width_thresh_min? true : false;
        passage_type.right_angle = scan_angle - M_PI_2 - scan->angle_min;
    }else{
        passage_type.right_flg = distance_minus_90 > distance_thresh && distance_minus_45_to_minus_135 < aisle_width_thresh_max && distance_minus_45_to_minus_135 > aisle_width_thresh_min? true : false;
        passage_type.right_angle = scan_angle - M_PI_2 - scan->angle_min;
    };

    if(distance_0 > 9.0 && distance_plus_90 < 7.0 && distance_plus_90 > 3.0 && distance_minus_90 > 10.0 && distance_180 > 7.0){
        passage_type.back_flg = distance_180 > distance_thresh && distance_plus_135_to_minus_135 < aisle_width_thresh_max && distance_plus_135_to_minus_135 > aisle_width_thresh_min ? true : false;
        passage_type.back_angle = scan_angle - M_PI - scan->angle_min;
    }else if(distance_180 > front_back_laser_choice_thresh){
        passage_type.back_flg = distance_180 > distance_thresh && distance_plus_175_to_minus_175 < aisle_width_thresh_max && distance_plus_175_to_minus_175 > aisle_width_thresh_min ? true : false;
        passage_type.back_angle = scan_angle - M_PI - scan->angle_min;
    }else{
        passage_type.back_flg = distance_180 > distance_thresh && distance_plus_135_to_minus_135 < aisle_width_thresh_max && distance_plus_135_to_minus_135 > aisle_width_thresh_min ? true : false;
        passage_type.back_angle = scan_angle - M_PI - scan->angle_min;
    };

    // publish passage_type of passage recognition
    passage_type_pub_.publish(passage_type);

    // publish line for rviz
    if (passage_type.left_flg  ) toe_index_list.push_back(scan_plus_90  );
    if (passage_type.center_flg) toe_index_list.push_back(scan_0);
    if (passage_type.right_flg ) toe_index_list.push_back(scan_minus_90 );
    if (passage_type.back_flg  ) toe_index_list.push_back(scan_180  );
    //toe_index_list.push_back(scan_plus_85);
    //toe_index_list.push_back(scan_plus_5);
    //toe_index_list.push_back(scan_minus_5);
    //toe_index_list.push_back(scan_minus_85);
    //toe_index_list.push_back(scan_minus_95);
    //toe_index_list.push_back(scan_minus_175);
    //toe_index_list.push_back(scan_plus_175);
    //toe_index_list.push_back(scan_plus_95);
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
