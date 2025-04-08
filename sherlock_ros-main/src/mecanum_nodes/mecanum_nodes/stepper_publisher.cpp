#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include<chrono>
#include<functional>

using namespace std::chrono_literals;

class CameraFakerNode : public rclcpp::Node{

    public:
    CameraFakerNode() : Node("stepper_motor_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("outlet_position", 10);
        timer_ = this->create_wall_timer(1s,std::bind(&CameraFakerNode::publish_outlet_position,this));

    }

    private:
    void publish_outlet_position()
    {
        auto message = std_msgs::msg::Float32();
        message.data = 20.0;
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;




};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraFakerNode>());
    rclcpp::shutdown();

    return 0;
}