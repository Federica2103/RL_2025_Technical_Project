#include <memory>
#include <chrono>
#include <map>
#include <vector>
#include <string>
#include <iostream>
#include <thread> 
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/string.hpp" 
#include "std_msgs/msg/empty.hpp" 
#include "ros2_kdl_package/action/move_arm.hpp"
#include <kdl/frames.hpp>

#include "ros_gz_interfaces/srv/set_entity_pose.hpp"

using MoveArm = ros2_kdl_package::action::MoveArm;
using GoalHandleMoveArm = rclcpp_action::ClientGoalHandle<MoveArm>;
using namespace std::chrono_literals;

class TaskManager : public rclcpp::Node
{
public:
    TaskManager() : Node("task_manager_node")
    {
        // --- 1. SETUP INIZIALE ---
        detected_["medicine"] = false;
        detected_["toys"]     = false;
        detected_["clothes"]  = false;

        fra2mo_position_ = KDL::Vector(0.15, -19.31, 0.60); 
        rest_position_ = KDL::Vector(-0.10, -19.10, 0.80);

        // --- 2. COMUNICAZIONE ---
        pub_fra2mo_goal_ = this->create_publisher<std_msgs::msg::String>("/fra2mo/goal", 10);

        sub_fra2mo_status_ = this->create_subscription<std_msgs::msg::String>(
            "/fra2mo/status", 10, 
            [this](const std_msgs::msg::String::SharedPtr msg) {
                this->fra2mo_status_callback(msg);
            });

        // --- PUBLISHER GRIPPER ---
        pub_grip_med_attach_ = this->create_publisher<std_msgs::msg::Empty>("/model/iiwa/detachable_joint/box_medicine/attach", 10);
        pub_grip_med_detach_ = this->create_publisher<std_msgs::msg::Empty>("/model/iiwa/detachable_joint/box_medicine/detach", 10);
        
        pub_grip_toys_attach_ = this->create_publisher<std_msgs::msg::Empty>("/model/iiwa/detachable_joint/box_toys/attach", 10);
        pub_grip_toys_detach_ = this->create_publisher<std_msgs::msg::Empty>("/model/iiwa/detachable_joint/box_toys/detach", 10);
        
        pub_grip_cloth_attach_ = this->create_publisher<std_msgs::msg::Empty>("/model/iiwa/detachable_joint/box_clothes/attach", 10);
        pub_grip_cloth_detach_ = this->create_publisher<std_msgs::msg::Empty>("/model/iiwa/detachable_joint/box_clothes/detach", 10);

        // --- CLIENT TELETRASPORTO ---
        client_set_pose_ = this->create_client<ros_gz_interfaces::srv::SetEntityPose>("/world/warehouse/set_pose");

        // --- 3. SAFETY RESET THREAD ---
        std::thread([this]() {
            std::this_thread::sleep_for(3s);
            RCLCPP_WARN(this->get_logger(), "!!! SAFETY RESET: FORZATURA DISTACCO TUTTI I CUBI !!!");
            auto msg = std_msgs::msg::Empty();
            for(int i=0; i<3; i++) {
                pub_grip_med_detach_->publish(msg);
                pub_grip_toys_detach_->publish(msg);
                pub_grip_cloth_detach_->publish(msg);
                std::this_thread::sleep_for(100ms);
            }
            RCLCPP_INFO(this->get_logger(), "Safety Reset Completato. Gripper pronto.");
        }).detach();

        // --- 4. SUBSCRIBERS ARUCO ---
        sub_medicine_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_medicine/aruco_single_med/pose", 10, 
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (!detected_["medicine"]) {
                    boxes_world_["medicine"] = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                    RCLCPP_INFO(this->get_logger(), "DETECTED MEDICINE");
                    detected_["medicine"] = true;
                }
            });

        sub_toys_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_toys/aruco_single_toys/pose", 10, 
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (!detected_["toys"]) {
                    boxes_world_["toys"] = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                    RCLCPP_INFO(this->get_logger(), "DETECTED TOYS");
                    detected_["toys"] = true;
                }
            });

        sub_clothes_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/aruco_clothes/aruco_single_clothes/pose", 10, 
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                if (!detected_["clothes"]) {
                    boxes_world_["clothes"] = KDL::Vector(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                    RCLCPP_INFO(this->get_logger(), "DETECTED CLOTHES");
                    detected_["clothes"] = true;
                }
            });

        client_ptr_ = rclcpp_action::create_client<MoveArm>(this, "move_arm");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() { this->check_and_start(); });
    }

private:
    // --- TELETRASPORTO PRESA (PICK) ---
    void teleport_box_to_effector(KDL::Vector effector_pos)
    {
        std::string current_obj = objects_sequence_[obj_idx_];
        std::string gazebo_model_name; 
        if (current_obj == "medicine") gazebo_model_name = "box_medicine";
        else if (current_obj == "toys") gazebo_model_name = "box_toys";
        else if (current_obj == "clothes") gazebo_model_name = "box_clothes";

        auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        request->entity.name = gazebo_model_name;
        request->pose.position.x = effector_pos.x();
        request->pose.position.y = effector_pos.y();
        request->pose.position.z = effector_pos.z() - 0.03; 
        request->pose.orientation.w = 1.0; 

        if (client_set_pose_->service_is_ready()) {
            client_set_pose_->async_send_request(request);
            RCLCPP_INFO(this->get_logger(), "SNAP: %s preso.", gazebo_model_name.c_str());
        }
    }

    // --- TELETRASPORTO DEPOSITO ---
    void teleport_box_to_fra2mo()
    {
        std::string current_obj = objects_sequence_[obj_idx_];
        std::string gazebo_model_name; 
        if (current_obj == "medicine") gazebo_model_name = "box_medicine";
        else if (current_obj == "toys") gazebo_model_name = "box_toys";
        else if (current_obj == "clothes") gazebo_model_name = "box_clothes";

        auto request = std::make_shared<ros_gz_interfaces::srv::SetEntityPose::Request>();
        request->entity.name = gazebo_model_name;
        
        // --- CALCOLO POSIZIONE BLIND SPOT---
        
        if (obj_idx_ == 0) {
            request->pose.position.x = 0.15 - 0.09; // ~0.06
            RCLCPP_INFO(this->get_logger(), "MAGIC DROP (Originale): %s -> X=0.06", gazebo_model_name.c_str());
        } else {
            // Metto la scatola a (0.15 + 0.09)
            request->pose.position.x = 0.15 + 0.09; 
            RCLCPP_INFO(this->get_logger(), "MAGIC DROP (Hack 0.5+0.09): %s -> X=0.59", gazebo_model_name.c_str());
        }

        request->pose.position.y = -19.31; 
        
        // Altezza 
        request->pose.position.z = 0.28; 
        request->pose.orientation.w = 1.0; 

        if (client_set_pose_->service_is_ready()) {
            client_set_pose_->async_send_request(request);
        }
    }

    void control_gripper(bool attach)
    {
        std_msgs::msg::Empty msg;
        std::string current_obj = objects_sequence_[obj_idx_];

        if (current_obj == "medicine") {
            if(attach) pub_grip_med_attach_->publish(msg);
            else       pub_grip_med_detach_->publish(msg);
        }
        else if (current_obj == "toys") {
            if(attach) pub_grip_toys_attach_->publish(msg);
            else       pub_grip_toys_detach_->publish(msg);
        }
        else if (current_obj == "clothes") {
            if(attach) pub_grip_cloth_attach_->publish(msg);
            else       pub_grip_cloth_detach_->publish(msg);
        }
        RCLCPP_INFO(this->get_logger(), "GRIPPER: %s -> %s", current_obj.c_str(), attach ? "ATTACH" : "DETACH");
    }

    void fra2mo_status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (waiting_for_fra2mo_ && msg->data == "ready") {
            RCLCPP_INFO(this->get_logger(), "FRA2MO E' TORNATO!");
            waiting_for_fra2mo_ = false;
            obj_idx_++;
            current_step_++; 
            process_next_goal();
        }
    }

    void check_and_start()
    {
        if (routine_started_) return;
        if (detected_["medicine"] && detected_["toys"] && detected_["clothes"]) {
            routine_started_ = true;
            timer_->cancel();
            timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() { this->timer_->cancel(); this->start_routine(); });
        }
    }

    void start_routine()
    {
        objects_sequence_ = {"medicine", "toys", "clothes"};
        obj_idx_ = 0;
        RCLCPP_INFO(this->get_logger(), "INIZIO CICLO COMPLETO.");

        for (const auto& obj_name : objects_sequence_) {
            KDL::Vector target_pos = boxes_world_[obj_name];
            double detected_z = target_pos.z(); 
            
            // Parametri Z
            double z_approach = detected_z + 0.25; 
            double z_pick = detected_z + 0.05; 

            // 1. Approach
            KDL::Vector approach_vec = target_pos; approach_vec.z(z_approach); 
            task_queue_.push_back(approach_vec); task_types_.push_back("APPROACH_BOX");

            // 2. Pick Down
            KDL::Vector pick_vec = target_pos; pick_vec.z(z_pick); 
            task_queue_.push_back(pick_vec); task_types_.push_back("PICK_DOWN");

            // 3. Lift
            task_queue_.push_back(approach_vec); task_types_.push_back("PICK_UP");

            // --- FASE FRA2MO ---
            // 4. Approach Fra2Mo
            KDL::Vector fra2mo_approach = fra2mo_position_;
            fra2mo_approach.z(0.70); 
            task_queue_.push_back(fra2mo_approach);
            task_types_.push_back("APPROACH_FRA2MO");

            // 5. Drop
            KDL::Vector drop_pos = fra2mo_position_;
            drop_pos.z(0.40); 
            task_queue_.push_back(drop_pos);
            task_types_.push_back("DROP_DOWN");

            // 6. Lift Drop
            task_queue_.push_back(fra2mo_approach); task_types_.push_back("DROP_UP");

            // 7. Rest
            task_queue_.push_back(rest_position_); task_types_.push_back("GO_REST");
        }
        process_next_goal();
    }

    void process_next_goal()
    {
        if (current_step_ >= task_queue_.size()) {
            RCLCPP_INFO(this->get_logger(), "MISSIONE COMPLETATA!");
            return;
        }
        KDL::Vector target = task_queue_[current_step_];
        std::string type = task_types_[current_step_];
        RCLCPP_INFO(this->get_logger(), "Step %lu/%lu [%s]", current_step_ + 1, task_queue_.size(), type.c_str());
        send_goal(target, type);
    }

    void send_goal(KDL::Vector target, std::string type)
    {
        if (!client_ptr_->wait_for_action_server(std::chrono::seconds(2))) return;

        MoveArm::Goal goal_msg;
        goal_msg.target_position.x = target.x();
        goal_msg.target_position.y = target.y();
        goal_msg.target_position.z = target.z();

        auto send_goal_options = rclcpp_action::Client<MoveArm>::SendGoalOptions();
        
        send_goal_options.result_callback = 
            [this, type, target](const GoalHandleMoveArm::WrappedResult & result) {
                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    
                    if (type == "PICK_DOWN") {
                        teleport_box_to_effector(target);
                        std::this_thread::sleep_for(std::chrono::milliseconds(200)); 
                        control_gripper(true); 
                        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
                    }
                    else if (type == "DROP_DOWN") {
                        // 1. Stacca il magnete
                        control_gripper(false); 
                        std::this_thread::sleep_for(std::chrono::milliseconds(200));

                        // 2. Sposta il pacco su Fra2Mo
                        teleport_box_to_fra2mo();

                        std::this_thread::sleep_for(std::chrono::milliseconds(500));
                    }
                    
                    if (type == "GO_REST") {
                        // 1. Siamo in posizione di riposo, abbiamo consegnato il pacco.
                        // 2. Diciamo a Fra2Mo cosa deve portare.
                        std_msgs::msg::String msg;
                        msg.data = objects_sequence_[obj_idx_]; // es: "medicine"
                        pub_fra2mo_goal_->publish(msg);
                        
                        RCLCPP_INFO(this->get_logger(), "Ordine inviato a Fra2Mo: %s. Attendo il ritorno...", msg.data.c_str());

                        // 3. Attiviamo il blocco. 
                        waiting_for_fra2mo_ = true;
                        
                        return; 
                    } 
                    
                    current_step_++;     
                    process_next_goal(); 
                }
            };
        client_ptr_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<MoveArm>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<ros_gz_interfaces::srv::SetEntityPose>::SharedPtr client_set_pose_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_fra2mo_goal_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_fra2mo_status_;

    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_grip_med_attach_, pub_grip_med_detach_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_grip_toys_attach_, pub_grip_toys_detach_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pub_grip_cloth_attach_, pub_grip_cloth_detach_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_medicine_, sub_toys_, sub_clothes_;

    std::map<std::string, KDL::Vector> boxes_world_;
    std::map<std::string, bool> detected_;
    bool routine_started_ = false;
    bool waiting_for_fra2mo_ = false; 

    KDL::Vector fra2mo_position_;
    KDL::Vector rest_position_;
    std::vector<KDL::Vector> task_queue_;
    std::vector<std::string> task_types_;
    std::vector<std::string> objects_sequence_;
    size_t current_step_ = 0;
    size_t obj_idx_ = 0; 
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TaskManager>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
