#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "scenario_navigation/Scenario.h"
#include "scenario_navigation/PassageType.h"
#include <unistd.h>
#include <cmath>
#include <vector>

#include <iostream>

class cmdVelController {
     public:
        cmdVelController();
        int SCENARIO_MAX = 10;
        void getRosParam(void);
        bool compareScenarioAndPassageType(const scenario_navigation::PassageType::ConstPtr& passage_type);
        void loadNextScenario(void);
        void updateLastNode(bool center_flg, bool back_flg, bool left_flg, bool right_flg);
        bool compareLastNodeAndCurrentNode(const scenario_navigation::PassageType::ConstPtr& passage_type);

        void turnFinishFlgCallback(const std_msgs::Bool::ConstPtr& turn_finish_flg);
        void passageTypeCallback(const scenario_navigation::PassageType::ConstPtr& passage_type);
        void stopCallback(const std_msgs::Bool::ConstPtr& stop);
        bool scenarioCallback(scenario_navigation::Scenario::Request& scenario,
                              scenario_navigation::Scenario::Response& res);
     private:
        ros::NodeHandle node_;
        ros::Publisher stop_pub_;
        ros::Publisher rotate_rad_pub_;
        ros::Subscriber passage_type_sub_;
        ros::Subscriber stop_sub_;
        ros::Subscriber turn_finish_flg_sub_;
        ros::ServiceServer scenario_server_;

        std::list<std::string> target_type_;
        std::list<std::int16_t> target_order_;
        std::list<std::string> target_direction_;
        std::list<std::string> target_action_;

        std::list<std::string>::iterator target_type_itr_begin_;
        std::list<std::int16_t>::iterator target_order_itr_begin_;
        std::list<std::string>::iterator target_direction_itr_begin_;
        std::list<std::string>::iterator target_action_itr_begin_;

        int scenario_num_ = 0;
        int scenario_progress_cnt_ = 0;
        int scenario_order_cnt_ = 0;
        int reach_target_type_cnt_ = 0;
        int reach_different_type_cnt_ = 0;
        int reach_target_type_cnt_margin_ = 6;
        int reach_different_type_cnt_margin_ = 6;
        float rotate_rad_ = 0.0;
        bool turn_flg_ = false;
        bool change_node_flg_ = false;
        bool satisfy_conditions_flg_ = false;
        bool stop_flg_ = true;
        bool request_update_last_node_flg = true;
        std_msgs::Float32 rotate_rad_for_pub_;
        scenario_navigation::PassageType last_node_;
};

cmdVelController::cmdVelController(){
    stop_pub_ = node_.advertise<std_msgs::Bool>("stop", 1, false);
    rotate_rad_pub_ = node_.advertise<std_msgs::Float32>("rotate_rad", 1, false);

    turn_finish_flg_sub_ = node_.subscribe<std_msgs::Bool> ("turn_finish_flg", 1, &cmdVelController::turnFinishFlgCallback, this);
    passage_type_sub_ = node_.subscribe<scenario_navigation::PassageType> ("passage_type", 1, &cmdVelController::passageTypeCallback, this);
    stop_sub_ = node_.subscribe<std_msgs::Bool> ("stop", 1, &cmdVelController::stopCallback, this);
    scenario_server_ = node_.advertiseService("scenario", &cmdVelController::scenarioCallback, this);

    updateLastNode(false, false, false, false);
    getRosParam();
}

void cmdVelController::getRosParam(void){
    SCENARIO_MAX = 10;
    node_.getParam("scenario_executor/scenario_max", SCENARIO_MAX);
}

bool cmdVelController::compareScenarioAndPassageType(const scenario_navigation::PassageType::ConstPtr& passage_type){
    std::string target_type = *std::next(target_type_itr_begin_, scenario_progress_cnt_);
    std::string target_direction = *std::next(target_direction_itr_begin_, scenario_progress_cnt_);

// check "straight_road"
    if(target_type == "straight_road"){
        if(passage_type->center_flg && passage_type->back_flg && !passage_type->left_flg && !passage_type->right_flg){
            return true;
        }
    }

// check 3_way_left and 3_way_right when 3_way is designated by scenario
    if(target_type == "3_way"){
    // 3_way_left
        if(passage_type->center_flg && passage_type->back_flg && passage_type->left_flg && !passage_type->right_flg){
            return true;
        }
    // 3_way_right
        if(passage_type->center_flg && passage_type->back_flg && !passage_type->left_flg && passage_type->right_flg){
            return true;
        }
     // 3_way_center
        if(!passage_type->center_flg && passage_type->back_flg && passage_type->left_flg && passage_type->right_flg){
            return true;
        }
   }

// check "end"(= 突き当り)
    if(target_type == "end"){
    // dead_end
        if(!passage_type->center_flg && passage_type->back_flg && !passage_type->left_flg && !passage_type->right_flg){
            return true;
        }
    // right
        if(!passage_type->center_flg && passage_type->back_flg && !passage_type->left_flg && passage_type->right_flg){
            return true;
        }
    // left
        if(!passage_type->center_flg && passage_type->back_flg && passage_type->left_flg && !passage_type->right_flg){
            return true;
        }
    // 3_way_center
        if(!passage_type->center_flg && passage_type->back_flg && passage_type->left_flg && passage_type->right_flg){
            return true;
        }
    }

// check "corridor"(ex, 交差点， 通路)
    if(target_type == "corridor"){
        if(target_direction == "left"){
            if(passage_type->left_flg){
                return true;
            }
        }
        else if(target_direction == "right"){
            if(passage_type->right_flg){
                return true;
            }
        }
    // if target_direction is not designated, do below
        else{
            if(passage_type->left_flg || passage_type->right_flg){
                return true;
            }
        }
    }
    return false;
}

void cmdVelController::loadNextScenario(void){
    std::string action = *std::next(target_action_itr_begin_, scenario_progress_cnt_);

// stop robot
    stop_flg_ = true;

    if(action == "stop"){
        ROS_INFO("Robot gets a goal");
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
    }
    else{
        ROS_INFO("Execute next action(%s)", action.c_str());
        stop_flg_ = false;
        change_node_flg_ = false;
        if(action.find("turn") != std::string::npos){
            turn_flg_ = true;

            //if(action.find("left")){
            if(action == "turn_left"){
                rotate_rad_for_pub_.data = M_PI_2;
                std::cout <<"L" << rotate_rad_for_pub_.data << std::endl;
            }
            //else if(action.find("right")){
            else if(action == "turn_right"){
              rotate_rad_for_pub_.data = -M_PI_2;
                std::cout <<"R" << rotate_rad_for_pub_.data << std::endl;
            }
            else{
                rotate_rad_for_pub_.data = M_PI;
            }
            rotate_rad_pub_.publish(rotate_rad_for_pub_);
        }
    }
}

void cmdVelController::updateLastNode(bool center_flg, bool back_flg, bool left_flg, bool right_flg){
    ROS_INFO("update last node");
    ROS_INFO("last node is front(%d) back(%d) left(%d) right(%d)", static_cast<int>(center_flg), static_cast<int>(back_flg),
                                                                      static_cast<int>(left_flg), static_cast<int>(right_flg));
    last_node_.center_flg = center_flg;
    last_node_.back_flg = back_flg;
    last_node_.left_flg = left_flg;
    last_node_.right_flg = right_flg;
}

bool cmdVelController::compareLastNodeAndCurrentNode(const scenario_navigation::PassageType::ConstPtr& passage_type){
    if(last_node_.center_flg == passage_type->center_flg && last_node_.back_flg == passage_type->back_flg &&
        last_node_.left_flg == passage_type->left_flg && last_node_.right_flg == passage_type->right_flg){
            return true;
        }
    else{
        return false;
    }
}

void cmdVelController::passageTypeCallback(const scenario_navigation::PassageType::ConstPtr& passage_type){
    if(! stop_flg_){
        if(request_update_last_node_flg){
            updateLastNode(passage_type->center_flg, passage_type->back_flg, passage_type->left_flg, passage_type->right_flg);
            request_update_last_node_flg = false;
        }
        if(! turn_flg_){
            if(change_node_flg_){
                satisfy_conditions_flg_ = compareScenarioAndPassageType(passage_type);
                if(satisfy_conditions_flg_){
                    ROS_INFO("find target node");
                    reach_target_type_cnt_++;
                    if(reach_target_type_cnt_margin_ <= reach_target_type_cnt_){
                        reach_target_type_cnt_ = 0;
                        scenario_order_cnt_++;
                        change_node_flg_ = false;
                        updateLastNode(passage_type->center_flg, passage_type->back_flg, passage_type->left_flg, passage_type->right_flg);
                        int order = *std::next(target_order_itr_begin_, scenario_progress_cnt_);
                        if(order <= scenario_order_cnt_){
                            ROS_INFO("Robot reaches target_node!!");
                            scenario_order_cnt_ = 0;
                            scenario_progress_cnt_++;
                            loadNextScenario();
                        }
                    }
                }
                else{
                    reach_target_type_cnt_ = 0;
                }
            }
            else{
                if(! compareLastNodeAndCurrentNode(passage_type)){
                    reach_different_type_cnt_++;
                    if(reach_different_type_cnt_margin_ <= reach_different_type_cnt_){
                        reach_different_type_cnt_ = 0;
                        updateLastNode(passage_type->center_flg, passage_type->back_flg, passage_type->left_flg, passage_type->right_flg);
                        change_node_flg_ = true;
                    }
                }
                else{
                    reach_different_type_cnt_ = 0;
                }
            }
        }
    }
}

void cmdVelController::turnFinishFlgCallback(const std_msgs::Bool::ConstPtr& turn_finish_flg){
    ROS_INFO("finish turn");
    turn_flg_ = false;
    scenario_progress_cnt_++;
    loadNextScenario();
    request_update_last_node_flg = true;
    change_node_flg_ = false;
}

void cmdVelController::stopCallback(const std_msgs::Bool::ConstPtr& stop_flg){
    stop_flg_ = stop_flg->data;
}

bool cmdVelController::scenarioCallback(scenario_navigation::Scenario::Request& scenario,
                                        scenario_navigation::Scenario::Response& res){
    scenario_num_++;
    target_type_.push_back(scenario.type);
    target_order_.push_back(scenario.order);
    target_direction_.push_back(scenario.direction);
    target_action_.push_back(scenario.action);

    std::string last_action = *std::next(target_action_.begin(), scenario_num_ - 1);
// check whether scenario is loaded
    if(last_action == "stop"){
        ROS_INFO("Completed loading scenario");

        target_type_itr_begin_ = target_type_.begin();
        target_order_itr_begin_ = target_order_.begin();
        target_direction_itr_begin_ = target_direction_.begin();
        target_action_itr_begin_ = target_action_.begin();

        stop_flg_ = false;
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
        loadNextScenario();
    }
    else{
        stop_flg_ = true;
        std_msgs::Bool stop_flg_for_pub;
        stop_flg_for_pub.data = stop_flg_;
        stop_pub_.publish(stop_flg_for_pub);
    }


//  debug
    std::cout << "####################################" << std::endl;
    std::cout << "type is " << scenario.type << std::endl;
    std::cout << "order is "  << std::hex << scenario.order << std::endl;
    std::cout << "direction is " << scenario.direction << std::endl;
    std::cout << "action is " << scenario.action << std::endl;
    std::cout << "####################################" << std::endl;

    return true;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "scenario_executor");
    cmdVelController cmd_vel_controller;
    while(ros::ok()){
        ros::spin();
    }

    return 0;
}
