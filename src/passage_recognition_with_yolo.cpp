#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/String.h"
#include "scenario_navigation/PassageType.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <actionlib/client/simple_action_client.h>
#include <scenario_navigation/BoundingBoxesAction.h>
#include <scenario_navigation/BoundingBoxesGoal.h>
#include <scenario_navigation/BoundingBoxesResult.h>
#include <scenario_navigation/BoundingBoxesFeedback.h>
#include <sensor_msgs/image_encodings.h>
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
        std::string robot_frame_ = "base_link";
        scenario_navigation::PassageType generate_publish_variable(bool center_flg, bool back_flg, bool left_flg, bool right_flg,
                                                                        int center_angle, int back_angle, int left_angle, int right_angle);
        void checkRobotCollision(std::vector<double> *x, std::vector<double> *y, std::vector<int> scan_index, std::vector<float*> distance_list);
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);

        std::vector<scenario_navigation::BoundingBox> yolo_result_;
        void actionYoloCallback(const actionlib::SimpleClientGoalState& state, const scenario_navigation::BoundingBoxesResultConstPtr& result);
        void merge_yolo_result(int width, double scan_angle, float *distance_left, float *distance_center, float *distance_right, float *distance_back);
        void scanAndImageCallback(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::Image::ConstPtr& image_msg);
        void imageCallback(const sensor_msgs::Image::ConstPtr& image_msg);

    private:
        ros::NodeHandle node_;
        message_filters::Subscriber<sensor_msgs::LaserScan> scan_sub_;
        ros::Publisher marker_pub_;
        ros::Publisher passage_type_pub_;

        actionlib::SimpleActionClient<scenario_navigation::BoundingBoxesAction> action_client_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> sync_policy_;
        typedef message_filters::Synchronizer<sync_policy_> Sync;
        ros::Subscriber image_sub_;

        boost::shared_ptr<Sync> sync_;
        double door_size_thresh = 10;
};

passageRecognition::passageRecognition() : action_client_("yolov5_action", true) {
    node_.getParam("passageRecognition/SCAN_HZ", SCAN_HZ);
    node_.getParam("passageRecognition/robot_frame", robot_frame_);
    node_.getParam("passageRecognition/ROBOT_RADIUS", ROBOT_RADIUS);
    node_.getParam("passageRecognition/MIN_WALL_DISTANCE", MIN_WALL_DISTANCE);

    scan_sub_.subscribe(node_, "merged_scan", 1);
    marker_pub_ = node_.advertise<visualization_msgs::MarkerArray>("corridor_visualization", 1);
    passage_type_pub_ = node_.advertise<scenario_navigation::PassageType>("passage_type", 1);

    //image_sub_.subscribe(node_, "image/mercator", 1);
    image_sub_ = node_.subscribe<sensor_msgs::Image> ("image/mercator", 1, &passageRecognition::imageCallback, this);
    
    //sync_.reset(new Sync(sync_policy_(10), scan_sub_, image_sub_));
    //sync_->registerCallback(boost::bind(&passageRecognition::scanAndImageCallback, this, _1, _2));

    bool server_exists = false;
    while(!server_exists){
        ROS_INFO("waiting for server: ");
        server_exists = action_client_.waitForServer(ros::Duration(5.0));
        if (!server_exists) ROS_WARN("could not connect to server");
    }
    ROS_INFO("connected to action server");
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

void passageRecognition::actionYoloCallback(const actionlib::SimpleClientGoalState& state, const scenario_navigation::BoundingBoxesResultConstPtr& result){
    yolo_result_ = result->yolo_result.bounding_boxes;
}

void passageRecognition::merge_yolo_result(int width, double scan_angle, float *distance_left, float *distance_center, float *distance_right, float *distance_back){
    std::string direction_name_[2] = {"LEFT", "RIGHT"};
    std::vector<float*> corridor_distance(2);
    int corridor_direction[2][2] = {{40, 140}, {350, 450}};
    corridor_distance[0] = distance_left;
    corridor_distance[1] = distance_right;
    
    float probability_thresh = 0.5;
    for(const auto obj : yolo_result_){
        if((obj.Class == "door" || obj.Class == "square" || obj.Class == "step") && obj.probability >= probability_thresh){
	    double object_size = obj.xmax - obj.xmin;
	    if(object_size > door_size_thresh) {
  	        double x_center = (obj.xmin + obj.xmax)/2;
		double y_center = (obj.ymin + obj.ymax)/2;
		for(int i = 0; i < corridor_distance.size(); i++){
		    if(corridor_direction[i][0] <= x_center && x_center <= corridor_direction[i][1]){
			std::cout << "Detect a "<< obj.Class <<" on the " << direction_name_[i] << std::endl;
			*corridor_distance[i] = 0;
		    }
		}
            }
	}
    }
}

void passageRecognition::imageCallback(const sensor_msgs::Image::ConstPtr& image_msg){
    // execute YOLO
    scenario_navigation::BoundingBoxesGoal goal;
    goal.image = *image_msg;
    action_client_.sendGoal(goal, boost::bind(&passageRecognition::actionYoloCallback, this, _1, _2));
    bool finished_before_timeout = action_client_.waitForResult(ros::Duration(1.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on Yolo result");
        return ;
    } else {
        for(const auto obj : yolo_result_){
            printf("detect\r\n");
	}
        printf("PASS\r\n");
    }
}

void passageRecognition::scanAndImageCallback(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::Image::ConstPtr& image_msg){
    // execute YOLO
    scenario_navigation::BoundingBoxesGoal goal;
    goal.image = *image_msg;
    action_client_.sendGoal(goal, boost::bind(&passageRecognition::actionYoloCallback, this, _1, _2));

    // compute corridor with LiDAR
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
    int scan_left   = static_cast<int>((scan_angle + M_PI_2 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_center = static_cast<int>((scan_angle          - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_right  = static_cast<int>((scan_angle - M_PI_2 - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    int scan_back   = static_cast<int>((scan_angle - M_PI   - scan->angle_min) / scan->angle_increment + num_scan) % num_scan;
    
    float distance_left   = std::sqrt(x[scan_left]   * x[scan_left]   + y[scan_left]   * y[scan_left]  );
    float distance_center = std::sqrt(x[scan_center] * x[scan_center] + y[scan_center] * y[scan_center]);
    float distance_right  = std::sqrt(x[scan_right]  * x[scan_right]  + y[scan_right]  * y[scan_right] );
    float distance_back   = std::sqrt(x[scan_back]   * x[scan_back]   + y[scan_back]   * y[scan_back]  );

    std::vector<int> scan_index = {scan_left, scan_center, scan_right, scan_back};
    std::vector<float*> distance_list(4);
    distance_list[0] = &distance_left;
    distance_list[1] = &distance_center;
    distance_list[2] = &distance_right;
    distance_list[3] = &distance_back;

    // assume there is no passage if a collision is likely to occur
    checkRobotCollision(&x, &y, scan_index, distance_list);

    // merge YOLO result
    bool finished_before_timeout = action_client_.waitForResult(ros::Duration(1.0));
    if (!finished_before_timeout) {
        ROS_WARN("giving up waiting on Yolo result");
        return ;
    } else {
        passageRecognition::merge_yolo_result(image_msg->width, scan_angle, &distance_left, &distance_center, &distance_right, &distance_back);
    }

    // publish passage_type of passage recognition
    scenario_navigation::PassageType passage_type;
	    
    passage_type.left_flg = distance_left > distance_thresh ? true : false;
    passage_type.left_angle = scan_angle + M_PI_2 - scan->angle_min;

    passage_type.center_flg = distance_center > distance_thresh ? true : false;
    passage_type.center_angle = scan_angle - scan->angle_min;

    passage_type.right_flg = distance_right > distance_thresh ? true : false;
    passage_type.right_angle = scan_angle - M_PI_2 - scan->angle_min;

    passage_type.back_flg = distance_back > distance_thresh ? true : false;
    passage_type.back_angle = scan_angle - M_PI - scan->angle_min;

    // publish passage_type of passage recognition
    passage_type_pub_.publish(passage_type);

    // publish line for rviz
    if (passage_type.left_flg  ) toe_index_list.push_back(scan_left  );
    if (passage_type.center_flg) toe_index_list.push_back(scan_center);
    if (passage_type.right_flg ) toe_index_list.push_back(scan_right );
    if (passage_type.back_flg  ) toe_index_list.push_back(scan_back  );
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
