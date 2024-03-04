#include "odrive_remote_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>
#include "rclcpp/node.hpp"
#include "odrive_remote/msg/control_message.hpp"
#include "odrive_remote/srv/axis_state.hpp"

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

using namespace std;
using std::placeholders::_1;

class odriveRemoteNode: public rclcpp::Node {
    public: 
        odriveRemoteNode(): Node("odrive_remote") {

            subscriber_L = this->create_subscription<std_msgs::msg::Float32>(
                "vel_L", 10, 
                std::bind(&odriveRemoteNode::vel_L_callback, this, _1));

            subscriber_R = this->create_subscription<std_msgs::msg::Float32>(
                "vel_R", 10, 
                std::bind(&odriveRemoteNode::vel_R_callback, this, _1));

            // publisher_ = this->create_publisher<odrive_remote::msg::ControlMessage>("odrive_axis0", 10);

            pub_left = {
              this->create_publisher<odrive_remote::msg::ControlMessage>("odrive_axis10", 10),
              this->create_publisher<odrive_remote::msg::ControlMessage>("odrive_axis11", 10),
              this->create_publisher<odrive_remote::msg::ControlMessage>("odrive_axis12", 10),
            };

            pub_right = {
              this->create_publisher<odrive_remote::msg::ControlMessage>("odrive_axis13", 10),
              this->create_publisher<odrive_remote::msg::ControlMessage>("odrive_axis14", 10),
              this->create_publisher<odrive_remote::msg::ControlMessage>("odrive_axis15", 10),
            };
            // timer_ = this->create_wall_timer(
            // 500ms, std::bind(&MinimalPublisher::timer_callback, this));
            RCLCPP_INFO(this->get_logger(), "Node Running");
            
        }

        
    private: 
        void publish_odrive_control(float velocity, int direction = -1) {
          auto pub_msg = odrive_remote::msg::ControlMessage();

          size_t control_mode = 2;
          size_t input_mode = 1;
          float input_pos = 0.0;
          float soft_limit = 0.8;
          float input_torque = 0.0;

          pub_msg.control_mode = control_mode;
          pub_msg.input_mode = input_mode;
          pub_msg.input_pos = input_pos;
          pub_msg.input_vel = velocity * soft_limit * direction;
          pub_msg.input_torque = input_torque;
          // RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
          // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", pub_msg.input_vel);
          if (direction < 0) {
            for (auto it: pub_left) {
              it->publish(pub_msg);
            }
          }
          else {
            for (auto it: pub_right) {
              it->publish(pub_msg);
            }
          }        


        }
        void vel_R_callback(const std_msgs::msg::Float32::SharedPtr msg) {
          float velocity = float(msg->data);
          publish_odrive_control(velocity, 1);
        }

        void vel_L_callback(const std_msgs::msg::Float32::SharedPtr msg)  {
          float velocity = float(msg->data);
          publish_odrive_control(velocity, -1);
        }

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_L;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_R;
        // rclcpp::Publisher<odrive_remote::msg::ControlMessage>::SharedPtr publisher_;
        std::vector<rclcpp::Publisher<odrive_remote::msg::ControlMessage>::SharedPtr> pub_left;
        std::vector<rclcpp::Publisher<odrive_remote::msg::ControlMessage>::SharedPtr> pub_right;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<odriveRemoteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}