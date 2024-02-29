#include "odrive_remote_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <string>
#include "rclcpp/node.hpp"
#include "odrive_remote/msg/control_message.hpp"
#include "odrive_remote/srv/axis_state.hpp"

using namespace std;
using std::placeholders::_1;

class odriveRemoteNode: public rclcpp::Node {
    public: 
        odriveRemoteNode(): Node("odrive_remote") {

            subscriber = this->create_subscription<std_msgs::msg::Float32>(
                "vel_L", 10, 
                std::bind(&odriveRemoteNode::topic_callback, this, _1));

            RCLCPP_INFO(this->get_logger(), "Node Running");
            
        }

        
    private: 
        void topic_callback(const std_msgs::msg::Float32::SharedPtr msg)  {
             RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg->data);
        }

        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<odriveRemoteNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}