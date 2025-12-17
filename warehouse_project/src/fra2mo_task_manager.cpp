#include <memory>
#include <chrono>
#include <string>
#include <map>
#include <thread>
#include <functional>
#include <cmath>
#include <algorithm> 

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/empty.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

enum class MissionState {
    IDLE,
    NAV_TO_TARGET,     
    DOCKING_TARGET,    
    DROPPING,          
    NAV_TO_HOME,       
    DOCKING_HOME,      
    WAITING_FOR_LOAD   
};

class Fra2moTaskManager : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;

    Fra2moTaskManager() : Node("fra2mo_task_manager")
    {
        // --- 1. PUBLISHERS & SUBSCRIBERS ---
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/fra2mo/status", 10);
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/fra2mo/cmd_vel", 10);

        goal_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/fra2mo/goal", 10, std::bind(&Fra2moTaskManager::goal_callback, this, _1));

        nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        client_set_pose_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/warehouse/set_pose");

        // --- 2. SOTTOSCRIZIONI ARUCO ---
        sub_aruco_iiwa_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_iiwa/aruco_single_iiwa/pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                aruco_callback(msg, "iiwa");
            });

        sub_aruco_red_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_red/aruco_single_red/pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                aruco_callback(msg, "medicine");
            });

        sub_aruco_green_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_green/aruco_single_green/pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                aruco_callback(msg, "toys");
            });

        sub_aruco_blue_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_blue/aruco_single_blue/pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                aruco_callback(msg, "clothes");
            });

        // --- 3. GRIPPER ---
        pub_attach_med_ = this->create_publisher<std_msgs::msg::Empty>("/model/fra2mo/detachable_joint/box_medicine/attach", 10);
        pub_detach_med_ = this->create_publisher<std_msgs::msg::Empty>("/model/fra2mo/detachable_joint/box_medicine/detach", 10);
        pub_attach_toys_ = this->create_publisher<std_msgs::msg::Empty>("/model/fra2mo/detachable_joint/box_toys/attach", 10);
        pub_detach_toys_ = this->create_publisher<std_msgs::msg::Empty>("/model/fra2mo/detachable_joint/box_toys/detach", 10);
        pub_attach_clothes_ = this->create_publisher<std_msgs::msg::Empty>("/model/fra2mo/detachable_joint/box_clothes/attach", 10);
        pub_detach_clothes_ = this->create_publisher<std_msgs::msg::Empty>("/model/fra2mo/detachable_joint/box_clothes/detach", 10);

        // --- 4. NAV2 GOALS ---
        tf2::Quaternion q_home;
        q_home.setRPY(0, 0, 3.14159); 

        home_pose_.header.frame_id = "map";
        home_pose_.pose.position.x = 0.5; 
        home_pose_.pose.position.y = -19.10; 
        home_pose_.pose.orientation = tf2::toMsg(q_home);

        // Depositi
        goals_["medicine"].header.frame_id = "map";
        goals_["medicine"].pose.position.x = 9.35; 
        goals_["medicine"].pose.position.y = 5.42; 
        goals_["medicine"].pose.orientation.w = 1.0;

        goals_["toys"].header.frame_id = "map";
        goals_["toys"].pose.position.x = 9.35; 
        goals_["toys"].pose.position.y = -13.79; 
        goals_["toys"].pose.orientation.w = 1.0;

        goals_["clothes"].header.frame_id = "map";
        goals_["clothes"].pose.position.x = -9.35; 
        goals_["clothes"].pose.position.y = -0.54; 
        goals_["clothes"].pose.orientation.w = 1.0;
        
        // --- 5. TIMER ---
        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&Fra2moTaskManager::control_loop, this));

        mission_state_ = MissionState::IDLE;
        tag_visible_ = false;
        last_tag_time_ = this->now();

        // Safety start
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(3)); 
            RCLCPP_WARN(this->get_logger(), "!!! SAFETY RESET: Detach all cargos !!!");
            for(int i=0; i<3; i++) {
                control_cargo("medicine", false); control_cargo("toys", false); control_cargo("clothes", false);
                std::this_thread::sleep_for(200ms);
            }
            publish_status("ready_for_order");
            RCLCPP_INFO(this->get_logger(), "--- Fra2Mo Task Manager READY ---");
        }).detach();
    }

private:
    
    void goal_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string cmd = msg->data;
        if (cmd == "start_mission_medicine" || cmd == "medicine") start_delivery("medicine");
        else if (cmd == "start_mission_toys" || cmd == "toys") start_delivery("toys");
        else if (cmd == "start_mission_clothes" || cmd == "clothes") start_delivery("clothes");
    }

    void start_delivery(std::string cargo_type) {
        RCLCPP_INFO(this->get_logger(), "RICEVUTO ORDINE: %s", cargo_type.c_str());
        current_cargo_ = cargo_type;
        control_cargo(current_cargo_, true);
        std::this_thread::sleep_for(0.5s);
        mission_state_ = MissionState::NAV_TO_TARGET;
        send_nav_goal(goals_[current_cargo_]);
    }

    void aruco_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, std::string tag_type)
    {
        last_aruco_pose_ = *msg;
        tag_visible_ = true;
        last_tag_time_ = this->now();

        // SWITCH NAV2 -> VISUAL SERVOING
        if (mission_state_ == MissionState::NAV_TO_TARGET && tag_type == current_cargo_) {
            double dist = msg->pose.position.z; 
            if (dist < 1.5) {
                RCLCPP_WARN(this->get_logger(), "TAG DEPOSITO VISTO. SWITCH TO DOCKING.");
                nav_client_->async_cancel_all_goals(); 
                mission_state_ = MissionState::DOCKING_TARGET;
            }
        }
        else if (mission_state_ == MissionState::NAV_TO_HOME && tag_type == "iiwa") {
            double dist = msg->pose.position.z;
            if (dist < 1.5) {
                RCLCPP_WARN(this->get_logger(), "TAG HOME VISTO. SWITCH TO DOCKING.");
                nav_client_->async_cancel_all_goals();
                mission_state_ = MissionState::DOCKING_HOME;
            }
        }
    }

    void control_loop()
    {
        if ((this->now() - last_tag_time_).seconds() > 2.0) tag_visible_ = false;

        if (mission_state_ == MissionState::DOCKING_TARGET || mission_state_ == MissionState::DOCKING_HOME) {
            
            // RECOVERY
            if (!tag_visible_) {
                auto cmd = geometry_msgs::msg::Twist();
                double search_rot_speed = 0.3; 
                double last_known_lateral = last_aruco_pose_.pose.position.x;
                cmd.angular.z = (last_known_lateral > 0) ? -search_rot_speed : search_rot_speed;
                cmd_vel_pub_->publish(cmd);
                return; 
            }

            // PID LOGIC
            double current_dist = last_aruco_pose_.pose.position.z;
            double lateral_error = last_aruco_pose_.pose.position.x; 
            
            double target_dist = 0.0;
            if (mission_state_ == MissionState::DOCKING_HOME) {
                target_dist = 0.27; 
            } else {
                target_dist = 0.60;
            }

            double dist_error = current_dist - target_dist;

            // Gains
            double kp_dist = 0.5; 
            double kp_ang = 1.0;   

            auto cmd = geometry_msgs::msg::Twist();

            // Velocità Minima
            if (std::abs(dist_error) > 0.01) {
                double v_calc = kp_dist * dist_error;
                double min_vel = 0.06; 
                
                if (std::abs(v_calc) < min_vel) {
                    v_calc = (v_calc > 0) ? min_vel : -min_vel;
                }
                cmd.linear.x = v_calc;
            } else {
                cmd.linear.x = 0.0;
            }

            if (std::abs(lateral_error) > 0.01) cmd.angular.z = -kp_ang * lateral_error; 
            else cmd.angular.z = 0.0;

            cmd.linear.x = std::clamp(cmd.linear.x, -0.25, 0.25); 
            cmd.angular.z = std::clamp(cmd.angular.z, -0.5, 0.5); 

            cmd_vel_pub_->publish(cmd);

            // STOP & CHECK
            if (std::abs(dist_error) < 0.01 && std::abs(lateral_error) < 0.01) {
                // Stop Totale
                cmd.linear.x = 0; cmd.angular.z = 0;
                cmd_vel_pub_->publish(cmd);
                tag_visible_ = false; 

                RCLCPP_INFO(this->get_logger(), ">>> ARRIVATO AL TARGET. Dist: %.3f", current_dist);

                if (mission_state_ == MissionState::DOCKING_TARGET) {
                    perform_drop_sequence();
                } else if (mission_state_ == MissionState::DOCKING_HOME) {
                    RCLCPP_WARN(this->get_logger(), ">>> SKIP ROTAZIONE (HACK). READY FOR IIWA.");
                    finish_mission();
                }
            }
        }
    }

    void perform_drop_sequence()
    {
        mission_state_ = MissionState::DROPPING;
        RCLCPP_INFO(this->get_logger(), ">>> DROP SCATOLA...");
        std::this_thread::sleep_for(1.0s);
        
        for(int i=0; i<5; i++) {
            control_cargo(current_cargo_, false);
            std::this_thread::sleep_for(100ms);
        }
        
        if (client_set_pose_->service_is_ready()) {
            auto req = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
            req->entity.name = "box_" + current_cargo_; 
            req->pose = goals_[current_cargo_].pose;
            req->pose.position.z = 0.05; 
            client_set_pose_->async_send_request(req);
        }
        std::this_thread::sleep_for(1.5s); 

        RCLCPP_INFO(this->get_logger(), ">>> DROP FINITO. TORNO A CASA.");
        mission_state_ = MissionState::NAV_TO_HOME;
        send_nav_goal(home_pose_);
    }

    void finish_mission()
    {
        mission_state_ = MissionState::IDLE;
        current_cargo_ = "";
        RCLCPP_INFO(this->get_logger(), ">>> READY FOR IIWA.");
        publish_status("ready"); 
    }

    void send_nav_goal(geometry_msgs::msg::PoseStamped target)
    {
        if (!nav_client_->wait_for_action_server(2s)) return;
        auto goal_msg = NavigateToPose::Goal();
        goal_msg.pose = target;
        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        nav_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void control_cargo(std::string category, bool attach)
    {
        auto msg = std_msgs::msg::Empty();
        if (category == "medicine") (attach ? pub_attach_med_ : pub_detach_med_)->publish(msg);
        else if (category == "toys") (attach ? pub_attach_toys_ : pub_detach_toys_)->publish(msg);
        else if (category == "clothes") (attach ? pub_attach_clothes_ : pub_detach_clothes_)->publish(msg);
    }

    void publish_status(std::string status)
    {
        auto msg = std_msgs::msg::String();
        msg.data = status;
        status_pub_->publish(msg);
    }

    // Variabili
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_sub_;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_aruco_iiwa_, sub_aruco_red_, sub_aruco_green_, sub_aruco_blue_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_set_pose_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_attach_med_, pub_detach_med_, pub_attach_toys_, pub_detach_toys_, pub_attach_clothes_, pub_detach_clothes_;

    geometry_msgs::msg::PoseStamped home_pose_;
    std::map<std::string, geometry_msgs::msg::PoseStamped> goals_;
    geometry_msgs::msg::PoseStamped last_aruco_pose_;
    rclcpp::Time last_tag_time_;
    bool tag_visible_;
    
    MissionState mission_state_;
    std::string current_cargo_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Fra2moTaskManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
